/**
 * Transparente Brücke: USB Serial <-> HW-UART (RS485, Halbduplex).
 *
 * RS485 = ein Sender zu jeder Zeit. Kollisionen entstehen, wenn der Controller
 * PC-Daten auf den Bus schickt, während der Rotor noch eine Antwort sendet —
 * oder wenn der Controller (rotor_rs485 / hw_send) zu früh nach einer Slave-
 * Antwort wieder sendet, bevor Halbduplex/MAX485 sauber umgeschaltet ist.
 *
 * Schutz: Zeitbasierte Bus-Idle — Controller-TX (hw_send) wartet kBusIdleUs;
 * USB->RS485 (pump_usb_to_hw) nutzt kuerzeres kBusIdleUsUsbForward, damit PC-Polling fluessig bleibt.
 */

#include "serial_bridge.h"
#include "rotor_rs485.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

namespace serial_bridge {

static SemaphoreHandle_t s_uart_mutex;
static QueueHandle_t s_tx_q_pc = nullptr;
static QueueHandle_t s_tx_q_ctrl = nullptr;
static TaskHandle_t s_task_usb_ingress = nullptr;
static TaskHandle_t s_task_rs485_rx = nullptr;
static TaskHandle_t s_task_arbiter = nullptr;

// --- Konfiguration (Board / Verdrahtung) ---------------------------------

/** TX-Leitung zum RS485-Wandler */
static constexpr int kPinUartTx = 43;
/** RX vom RS485-Wandler */
static constexpr int kPinUartRx = 44;
/** DE/RE bzw. Sende-/Empfangsumschaltung (typ. MAX485: HIGH = Senden aktiv) */
static constexpr int kPinRs485Dir = 39;

/** true: Dir-Pin HIGH = Senden; false: invertierte Ansteuerung */
static constexpr bool kDirTransmitLevel = true;

/** HW -> USB + Parser: Blockgröße */
static constexpr size_t kHwToUsbBuf = 512;
/** Max. RS485-Telegramm inkl. '$' */
static constexpr size_t kFrameMax = 320;
/** Queue-Tiefe je Quelle */
static constexpr size_t kTxQueueDepth = 32;

/** Wartezeit nach letztem gesendeten Byte, bevor wieder Empfang (Bus-Freigabe) */
static constexpr uint32_t kTurnaroundMicros = 250;

/**
 * Mindest-Stille vor erneutem Senden (nur Controller-hw_send). 12 ms ≈ 138 Byte-Zeiten
 * @ 115200 — groessere Marge fuer MAX485/Slave-Umschaltung (sonst lange GETREF-Timeouts).
 */
static constexpr uint32_t kBusIdleUs = 12000;
/** SETPOSCC-Priorität: deutlich kürzere Mindest-Stille vor Senden. */
static constexpr uint32_t kBusIdleUsPriority = 1800;
/**
 * USB (PC) → RS485: kürzeres Idle als Controller-TX — der PC pollt oft (GETPOSDG);
 * 12 ms hier würde mit loop()-Takt zu spürbar langsamen seriellen Antworten führen.
 * Nach letztem RX-Byte ist der Slave typisch schon im Empfang; ~2 ms reichen als Marge.
 */
static constexpr uint32_t kBusIdleUsUsbForward = 1400;

/** Max. Warten auf Bus-Idle (ms); muss deutlich > kBusIdleUs sein, sonst Senden ohne Pause. */
static constexpr uint32_t kBusIdleWaitCapMs = 200;
static constexpr uint32_t kBusIdleWaitCapPriorityMs = 35;
static constexpr uint32_t kBusIdleWaitCapPcMs = 25;
static constexpr uint32_t kPcProxyAutoSilenceMs = 12000;
static constexpr uint32_t kPcProxyInFlightTimeoutMs = 320;

static HardwareSerial *s_hw = &Serial2;
static uint32_t s_baud = 115200;
static volatile BridgeMode s_mode = BridgeMode::LocalMaster;
static volatile uint32_t s_last_pc_frame_ms = 0;
static volatile bool s_pc_inflight = false;
static volatile uint32_t s_pc_inflight_since_ms = 0;

/** Zeitstempel der letzten Bus-Aktivität (TX oder RX), micros(). */
static uint32_t s_last_bus_activity_us = 0;

/** Ausstehende USB-Spiegel-Bytes (verlustfrei, auch wenn CDC kurz voll ist). */
static constexpr size_t kUsbMirrorPendingCap = 12288;
static uint8_t s_usb_mirror_pending[kUsbMirrorPendingCap];
static size_t s_usb_mirror_pending_len = 0;

void set_baud(uint32_t baud) { s_baud = baud; }

static inline void dir_receive()
{
    digitalWrite(kPinRs485Dir, kDirTransmitLevel ? LOW : HIGH);
}

static inline void dir_transmit()
{
    digitalWrite(kPinRs485Dir, kDirTransmitLevel ? HIGH : LOW);
}

void uart_lock()
{
    if (s_uart_mutex) {
        xSemaphoreTake(s_uart_mutex, portMAX_DELAY);
    }
}

void uart_unlock()
{
    if (s_uart_mutex) {
        xSemaphoreGive(s_uart_mutex);
    }
}

static void mirror_usb_pending_push(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return;
    }
    if (len >= kUsbMirrorPendingCap) {
        data += (len - kUsbMirrorPendingCap);
        len = kUsbMirrorPendingCap;
        s_usb_mirror_pending_len = 0;
    } else if (s_usb_mirror_pending_len + len > kUsbMirrorPendingCap) {
        const size_t drop = s_usb_mirror_pending_len + len - kUsbMirrorPendingCap;
        if (drop >= s_usb_mirror_pending_len) {
            s_usb_mirror_pending_len = 0;
        } else {
            memmove(s_usb_mirror_pending, s_usb_mirror_pending + drop, s_usb_mirror_pending_len - drop);
            s_usb_mirror_pending_len -= drop;
        }
    }
    memcpy(s_usb_mirror_pending + s_usb_mirror_pending_len, data, len);
    s_usb_mirror_pending_len += len;
}

static void drain_usb_mirror_pending_to_cdc()
{
    while (s_usb_mirror_pending_len > 0) {
        const int space = Serial.availableForWrite();
        if (space <= 0) return;
        const size_t chunk = (size_t)space < s_usb_mirror_pending_len ? (size_t)space : s_usb_mirror_pending_len;
        const size_t w = Serial.write(s_usb_mirror_pending, chunk);
        if (w == 0) {
            return;
        }
        if (w < s_usb_mirror_pending_len) {
            memmove(s_usb_mirror_pending, s_usb_mirror_pending + w, s_usb_mirror_pending_len - w);
        }
        s_usb_mirror_pending_len -= w;
    }
}

/** USB-Monitor: nonblocking, Überhang in Pending-FIFO statt Drop. */
static void mirror_usb_nonblocking(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return;
    }
    drain_usb_mirror_pending_to_cdc();
    size_t off = 0;
    while (off < len) {
        const int space = Serial.availableForWrite();
        if (space <= 0) {
            mirror_usb_pending_push(data + off, len - off);
            return;
        }
        const size_t chunk =
            (size_t)space < (len - off) ? (size_t)space : (len - off);
        const size_t w = Serial.write(data + off, chunk);
        if (w == 0) {
            mirror_usb_pending_push(data + off, len - off);
            return;
        }
        off += w;
    }
}

/**
 * Vor Controller-Telegramm (GETREF, GETPOS, …): gleiche Idle-Regel wie bei USB->Bus,
 * damit nicht im selben loop()-Takt direkt nach Slave-TX wieder gesendet wird.
 */
static void wait_bus_idle_for_controller_tx(uint32_t min_idle_us, uint32_t cap_ms)
{
    const uint32_t deadline_ms = millis() + cap_ms;
    for (;;) {
        uart_lock();
        const bool has_rx = (s_hw->available() > 0);
        uart_unlock();
        if (has_rx) {
            delayMicroseconds(200);
            yield();
            continue;
        }

        if ((uint32_t)(micros() - s_last_bus_activity_us) >= min_idle_us) {
            return;
        }

        if ((int32_t)(millis() - deadline_ms) >= 0) {
            /* Kein volles Idle erreicht (z. B. Dauer-RX): trotzdem Mindestluecke erzwingen */
            delayMicroseconds(min_idle_us);
            return;
        }

        delayMicroseconds(200);
        yield();
    }
}

struct TxFrame {
    uint16_t len;
    uint8_t flags;
    uint8_t data[kFrameMax];
};

enum class PcReqKind : uint8_t {
    Unknown = 0,
    GetPosDg = 1,
};

static volatile PcReqKind s_pc_inflight_kind = PcReqKind::Unknown;

static PcReqKind detect_pc_req_kind(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return PcReqKind::Unknown;
    }
    if (len >= 12 && strstr(reinterpret_cast<const char *>(data), ":GETPOSDG:")) {
        return PcReqKind::GetPosDg;
    }
    return PcReqKind::Unknown;
}

static bool enqueue_frame(QueueHandle_t q, const uint8_t *data, size_t len, uint8_t flags)
{
    if (!q || !data || len == 0) {
        return false;
    }
    TxFrame f{};
    if (len > kFrameMax) {
        data += (len - kFrameMax);
        len = kFrameMax;
    }
    f.len = static_cast<uint16_t>(len);
    f.flags = flags;
    memcpy(f.data, data, len);
    return xQueueSend(q, &f, pdMS_TO_TICKS(8)) == pdPASS;
}

static bool enqueue_pc_frame_blocking(const uint8_t *data, size_t len)
{
    if (!s_tx_q_pc || !data || len == 0) {
        return false;
    }
    const PcReqKind kind = detect_pc_req_kind(data, len);
    if (kind == PcReqKind::GetPosDg && s_pc_inflight &&
        s_pc_inflight_kind == PcReqKind::GetPosDg) {
        /* GETPOSDG entkoppeln: keinen zweiten Poll stauen, solange Antwort offen ist. */
        return true;
    }

    TxFrame f{};
    if (len > kFrameMax) {
        data += (len - kFrameMax);
        len = kFrameMax;
    }
    f.len = static_cast<uint16_t>(len);
    f.flags = 0x2u;
    memcpy(f.data, data, len);
    while (xQueueSend(s_tx_q_pc, &f, pdMS_TO_TICKS(20)) != pdPASS) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return true;
}

void hw_send(const uint8_t *data, size_t len)
{
    (void)enqueue_frame(s_tx_q_ctrl, data, len, 0u);
}

void hw_send_priority(const uint8_t *data, size_t len)
{
    (void)enqueue_frame(s_tx_q_ctrl, data, len, 1u);
}

void set_mode(BridgeMode mode)
{
    s_mode = mode;
}

BridgeMode get_mode()
{
    return s_mode;
}

static void rs485_send_frame(const TxFrame &f, bool from_pc)
{
    const bool prio = (f.flags & 0x1u) != 0;
    if (from_pc) {
        wait_bus_idle_for_controller_tx(kBusIdleUsUsbForward, kBusIdleWaitCapPcMs);
    } else if (prio) {
        wait_bus_idle_for_controller_tx(kBusIdleUsPriority, kBusIdleWaitCapPriorityMs);
    } else {
        wait_bus_idle_for_controller_tx(kBusIdleUs, kBusIdleWaitCapMs);
    }

    uart_lock();
    dir_transmit();
    s_hw->write(f.data, f.len);
    s_hw->flush();
    if (kTurnaroundMicros) {
        delayMicroseconds(kTurnaroundMicros);
    }
    dir_receive();
    uart_unlock();
    s_last_bus_activity_us = micros();

    if (from_pc) {
        /* PC-Befehle sofort auch lokal verarbeiten (Sniffer/State), ohne den Hot-Path zu blockieren. */
        rotor_rs485_rx_bytes(f.data, f.len);
    } else {
        /* Controller-TX auf USB spiegeln, damit PC denselben Strom sieht wie der Bus. */
        mirror_usb_nonblocking(f.data, f.len);
    }
}

static bool try_receive_frame(QueueHandle_t q, TxFrame *out)
{
    if (!q || !out) {
        return false;
    }
    return xQueueReceive(q, out, 0) == pdPASS;
}

static void task_arbiter(void *)
{
    for (;;) {
        TxFrame f{};
        bool have = false;
        bool from_pc = false;
        const BridgeMode mode_now = get_mode();

        if (mode_now == BridgeMode::PcProxyMaster) {
            have = try_receive_frame(s_tx_q_pc, &f);
            from_pc = have;
            if (!have) {
                have = try_receive_frame(s_tx_q_ctrl, &f);
                from_pc = false;
            }
        } else {
            have = try_receive_frame(s_tx_q_ctrl, &f);
            from_pc = false;
            if (!have) {
                have = try_receive_frame(s_tx_q_pc, &f);
                from_pc = have;
            }
        }

        if (!have) {
            /* Blockierend auf PC-Queue warten, dann erneut priorisieren. */
            if (xQueueReceive(s_tx_q_pc, &f, pdMS_TO_TICKS(8)) != pdPASS) {
                taskYIELD();
                continue;
            }
            from_pc = true;
        }

        if (mode_now == BridgeMode::PcProxyMaster && from_pc) {
            if (s_pc_inflight &&
                (uint32_t)(millis() - s_pc_inflight_since_ms) < kPcProxyInFlightTimeoutMs) {
                /* Noch Antwort auf vorheriges PC-Telegramm offen: Frame kurz zurückstellen. */
                (void)xQueueSendToFront(s_tx_q_pc, &f, 0);
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
        }
        rs485_send_frame(f, from_pc);
        if (mode_now == BridgeMode::PcProxyMaster && from_pc) {
            s_pc_inflight = true;
            s_pc_inflight_since_ms = millis();
            s_pc_inflight_kind = detect_pc_req_kind(f.data, f.len);
        }
    }
}

static void task_rs485_rx(void *)
{
    static uint8_t s_rx_frame[kFrameMax];
    static size_t s_rx_frame_len = 0;
    static bool s_rx_in_frame = false;

    auto observe_rx_frame = [](const uint8_t *data, size_t len) {
        if (!data || len == 0) {
            return;
        }
        for (size_t i = 0; i < len; ++i) {
            const uint8_t b = data[i];
            if (!s_rx_in_frame) {
                if (b == '#') {
                    s_rx_in_frame = true;
                    s_rx_frame_len = 0;
                    s_rx_frame[s_rx_frame_len++] = b;
                }
                continue;
            }

            if (s_rx_frame_len < kFrameMax) {
                s_rx_frame[s_rx_frame_len++] = b;
            } else {
                s_rx_in_frame = false;
                s_rx_frame_len = 0;
                continue;
            }

            if (b == '$') {
                /* In-Flight erst bei komplettem RX-Telegramm freigeben (nicht bei Teil-Chunk). */
                if (s_mode == BridgeMode::PcProxyMaster) {
                    if (s_pc_inflight_kind == PcReqKind::GetPosDg) {
                        const char *line = reinterpret_cast<const char *>(s_rx_frame);
                        if (strstr(line, ":ACK_GETPOSDG:") || strstr(line, ":NAK_GETPOSDG:") ||
                            strstr(line, ":ERR:")) {
                            s_pc_inflight = false;
                            s_pc_inflight_kind = PcReqKind::Unknown;
                        }
                    } else {
                        s_pc_inflight = false;
                        s_pc_inflight_kind = PcReqKind::Unknown;
                    }
                }
                s_rx_in_frame = false;
                s_rx_frame_len = 0;
            } else if (b == '#') {
                /* Resync auf neues Startzeichen */
                s_rx_frame_len = 1;
                s_rx_frame[0] = '#';
            }
        }
    };

    for (;;) {
        uint8_t buf[kHwToUsbBuf];
        size_t n = 0;
        uart_lock();
        while (s_hw->available() && n < sizeof(buf)) {
            int c = s_hw->read();
            if (c < 0) {
                break;
            }
            buf[n++] = static_cast<uint8_t>(c);
        }
        uart_unlock();
        if (n == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        s_last_bus_activity_us = micros();
        observe_rx_frame(buf, n);
        rotor_rs485_rx_bytes(buf, n);
        mirror_usb_nonblocking(buf, n);
    }
}

static void flush_usb_rx_frame(uint8_t *buf, size_t *len)
{
    if (!buf || !len || *len == 0) {
        return;
    }
    s_last_pc_frame_ms = millis();
    s_mode = BridgeMode::PcProxyMaster;
    (void)enqueue_pc_frame_blocking(buf, *len);
    *len = 0;
}

static void task_usb_ingress(void *)
{
    uint8_t frame[kFrameMax];
    size_t frame_len = 0;
    bool in_frame = false;
    for (;;) {
        bool read_any = false;
        while (Serial.available()) {
            int c = Serial.read();
            if (c < 0) {
                break;
            }
            read_any = true;
            s_last_pc_frame_ms = millis();
            s_mode = BridgeMode::PcProxyMaster;
            if ((uint32_t)(millis() - s_pc_inflight_since_ms) >= kPcProxyInFlightTimeoutMs) {
                s_pc_inflight = false;
                s_pc_inflight_kind = PcReqKind::Unknown;
            }
            uint8_t b = static_cast<uint8_t>(c);
            if (!in_frame) {
                if (b == '#') {
                    in_frame = true;
                    frame_len = 0;
                    frame[frame_len++] = b;
                }
                continue;
            }

            if (frame_len < kFrameMax) {
                frame[frame_len++] = b;
            } else {
                /* Oversize-Frame verwerfen bis nächstes '#' */
                in_frame = false;
                frame_len = 0;
                continue;
            }

            if (b == '$' && frame_len >= 5) {
                flush_usb_rx_frame(frame, &frame_len);
                in_frame = false;
            } else if (b == '#') {
                /* Resync auf neues Startzeichen */
                frame_len = 1;
                frame[0] = '#';
            }
        }
        vTaskDelay(read_any ? 0 : pdMS_TO_TICKS(1));
    }
}

void begin()
{
    if (!s_uart_mutex) {
        s_uart_mutex = xSemaphoreCreateMutex();
    }
    if (!s_tx_q_pc) {
        s_tx_q_pc = xQueueCreate(kTxQueueDepth, sizeof(TxFrame));
    }
    if (!s_tx_q_ctrl) {
        s_tx_q_ctrl = xQueueCreate(kTxQueueDepth, sizeof(TxFrame));
    }
    pinMode(kPinRs485Dir, OUTPUT);
    dir_receive();

    /* Muss vor begin() — sonst [E] HardwareSerial „can't be resized when already running" */
    /* Großer RX: bei vielen GETPOSDG-ACKs + LVGL darf der Puffer nicht überlaufen (sonst Nachlauf-Anzeige). */
    s_hw->setRxBufferSize(4096);
    s_hw->setTxBufferSize(2048);
    s_hw->begin(s_baud, SERIAL_8N1, kPinUartRx, kPinUartTx);
    if (!s_task_usb_ingress) {
        xTaskCreatePinnedToCore(task_usb_ingress, "usb_ingress", 4096, nullptr, 3, &s_task_usb_ingress, 1);
    }
    if (!s_task_rs485_rx) {
        xTaskCreatePinnedToCore(task_rs485_rx, "rs485_rx", 4096, nullptr, 4, &s_task_rs485_rx, 1);
    }
    if (!s_task_arbiter) {
        xTaskCreatePinnedToCore(task_arbiter, "rs485_arb", 4096, nullptr, 4, &s_task_arbiter, 1);
    }
}

void poll()
{
    drain_usb_mirror_pending_to_cdc();
    if (s_mode == BridgeMode::PcProxyMaster) {
        if ((uint32_t)(millis() - s_last_pc_frame_ms) > kPcProxyAutoSilenceMs) {
            s_mode = BridgeMode::LocalMaster;
        }
    }
}

} // namespace serial_bridge
