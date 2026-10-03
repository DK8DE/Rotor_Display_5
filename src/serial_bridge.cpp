/**
 * Transparente Bruecke: USB Serial <-> HW-UART (RS485, Halbduplex).
 * Transceiver schaltet Senden/Empfang selbst — kein DE/RE-GPIO.
 *
 * Architektur:
 * - USB-RX liest komplette #...$ Frames vom PC und legt sie in die RS485-TX-Queue.
 * - Ein einziger RS485-TX-Task sendet alle PC- und Controller-Frames auf den Bus.
 * - RS485-RX liest den Bus schnell aus und verteilt Bytes parallel an USB-TX und Sniffer.
 * - USB-TX schreibt unabhaengig zum PC.
 * - Sniffer ruft rotor_rs485_rx_bytes() auf, damit Display/State aktualisiert werden.
 *
 * Damit blockieren Parser, LVGL/UI und USB-CDC-Backpressure nicht mehr den RS485-RX-Hotpath.
 */

#include "serial_bridge.h"
#include "pwm_config.h"
#include "rotor_rs485.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

namespace serial_bridge {

/** TX-Leitung zum RS485-Wandler */
static constexpr int kPinUartTx = 43;
/** RX vom RS485-Wandler */
static constexpr int kPinUartRx = 44;

/** Max. RS485-Telegramm inkl. '$' */
static constexpr size_t kFrameMax = 320;
/** UART-RX Lese-Chunk; Frames bleiben im Stream erhalten. */
static constexpr size_t kIoChunkMax = 256;

/** Queue-Tiefen: genug Puffer fuer kurze PC-Bursts, ohne RX/USB dauerhaft zu blockieren. */
static constexpr size_t kTxQueueDepth = 64;
static constexpr size_t kTxPriorityQueueDepth = 12;
static constexpr size_t kUsbTxQueueDepth = 96;
/* Sniffer darf keine RS485-Zeilen verlieren, sonst sieht rotor_rs485 keine ACKs
 * und meldet sporadisch Verbindungstimeout trotz korrekter Busantworten. */
static constexpr size_t kSniffQueueDepth = 192;

/** Kurze Pause nach TX-Flush (Bus-Echo / Transceiver-Umschaltung abklingen lassen). */
static constexpr uint32_t kTurnaroundMicros = 250;
/** Mindest-Stille vor PC- oder Controller-Frames. */
static constexpr uint32_t kBusIdleUsPc = 1400;
static constexpr uint32_t kBusIdleUsCtrl = 12000;
static constexpr uint32_t kBusIdleUsPriority = 1800;
static constexpr uint32_t kBusIdleWaitCapPcMs = 25;
static constexpr uint32_t kBusIdleWaitCapCtrlMs = 200;
static constexpr uint32_t kBusIdleWaitCapPriorityMs = 35;
static constexpr uint32_t kPcProxyAutoSilenceMs = 12000;
/** Remote-USB: Link für Relink erst nach langer Pause weg (kurze Software-Pausen → kein Reboot). */
static constexpr uint32_t kRemoteUsbLinkDownMs = 15000;
/** Remote-USB: UI „Warten auf PC“ nach USB-Stille.
 * Länger als typische Homing-/Poll-Pausen, damit Meldetext nicht flackert.
 * Soft-10 greift bei remote_usb_active() ohnehin nicht mehr. */
static constexpr uint32_t kRemoteUsbUiSilenceMs = 4500;

struct TxFrame {
    uint16_t len;
    uint8_t flags;
    uint8_t data[kFrameMax];
};

struct IoChunk {
    uint16_t len;
    uint8_t data[kIoChunkMax];
};

static constexpr uint8_t kTxFlagFromPc = 0x01u;
static constexpr uint8_t kTxFlagPriority = 0x02u;

static HardwareSerial *s_hw = &Serial2;
static uint32_t s_baud = 115200;
static volatile BridgeMode s_mode = BridgeMode::LocalMaster;
static volatile uint32_t s_last_pc_frame_ms = 0;
static volatile bool s_remote_usb = false;
static volatile bool s_remote_usb_had_pc = false;
/** True solange kürzlich USB-Aktivität (PC online). Relink nur bei Übergang false→true. */
static volatile bool s_remote_usb_link_up = false;

static SemaphoreHandle_t s_uart_mutex = nullptr;
static QueueHandle_t s_tx_q = nullptr;
static QueueHandle_t s_tx_prio_q = nullptr;
static QueueHandle_t s_usb_tx_q = nullptr;
static QueueHandle_t s_sniff_q = nullptr;

static TaskHandle_t s_task_usb_rx = nullptr;
static TaskHandle_t s_task_rs485_tx = nullptr;
static TaskHandle_t s_task_rs485_rx = nullptr;
static TaskHandle_t s_task_usb_tx = nullptr;
static TaskHandle_t s_task_sniffer = nullptr;

/** Zeitstempel der letzten Bus-Aktivität (TX oder RX), micros(). */
static volatile uint32_t s_last_bus_activity_us = 0;

void set_baud(uint32_t baud) { s_baud = baud; }

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

void set_mode(BridgeMode mode)
{
    s_mode = mode;
}

BridgeMode get_mode()
{
    return s_mode;
}

static void force_rs485_tx_hi_z()
{
    pinMode(kPinUartTx, INPUT);
}

void set_remote_usb(bool on)
{
    const bool was = s_remote_usb;
    s_remote_usb = on;
    if (!on) {
        s_remote_usb_had_pc = false;
        s_remote_usb_link_up = false;
        s_last_pc_frame_ms = 0;
        /* UART wieder voll (RX+TX) für normalen RS485-Betrieb. */
        if (was) {
            TxFrame drop{};
            while (s_tx_prio_q && xQueueReceive(s_tx_prio_q, &drop, 0) == pdPASS) {
            }
            while (s_tx_q && xQueueReceive(s_tx_q, &drop, 0) == pdPASS) {
            }
            s_hw->end();
            s_hw->begin(s_baud, SERIAL_8N1, kPinUartRx, kPinUartTx);
        }
        return;
    }
    /* Remote: TX-Pin hochohmig — physisch kein RS485-Senden mehr. */
    if (!was) {
        /* Offene TX-Queue verwerfen, damit nichts nach dem Umschalten noch rausgeht. */
        TxFrame drop{};
        while (s_tx_prio_q && xQueueReceive(s_tx_prio_q, &drop, 0) == pdPASS) {
        }
        while (s_tx_q && xQueueReceive(s_tx_q, &drop, 0) == pdPASS) {
        }
        s_hw->end();
        force_rs485_tx_hi_z();
        /* Nur RX (Sniffer aus); TX-Pin = -1 → kein UART-TX auf den Bus. */
        s_hw->begin(s_baud, SERIAL_8N1, kPinUartRx, -1);
        force_rs485_tx_hi_z();
        s_remote_usb_link_up = false;
        s_remote_usb_had_pc = false;
        s_last_pc_frame_ms = 0;
    } else {
        force_rs485_tx_hi_z();
    }
}

bool remote_usb_active()
{
    return s_remote_usb;
}

bool remote_usb_pc_seen()
{
    return s_remote_usb && s_remote_usb_link_up &&
           s_last_pc_frame_ms != 0 &&
           (uint32_t)(millis() - s_last_pc_frame_ms) <= kRemoteUsbUiSilenceMs;
}

bool remote_usb_link_session()
{
    /* Session bleibt bis Link-Down (~15 s) — Homing/UI nicht bei jeder Poll-Pause kippen. */
    return s_remote_usb && s_remote_usb_link_up;
}

bool remote_usb_waiting_for_pc()
{
    if (!s_remote_usb) {
        return false;
    }
    /* Nie verbunden, Link lange weg, oder Programm gerade geschlossen (USB-Stille). */
    if (!s_remote_usb_link_up || s_last_pc_frame_ms == 0) {
        return true;
    }
    return (uint32_t)(millis() - s_last_pc_frame_ms) > kRemoteUsbUiSilenceMs;
}

static bool enqueue_tx_frame(const uint8_t *data, size_t len, uint8_t flags)
{
    if (!data || len == 0) {
        return false;
    }
    QueueHandle_t q = (flags & kTxFlagPriority) ? s_tx_prio_q : s_tx_q;
    if (!q) {
        return false;
    }
    TxFrame f{};
    if (len > sizeof(f.data)) {
        data += (len - sizeof(f.data));
        len = sizeof(f.data);
    }
    f.len = (uint16_t)len;
    f.flags = flags;
    memcpy(f.data, data, len);
    return xQueueSend(q, &f, pdMS_TO_TICKS(5)) == pdPASS;
}

static void enqueue_chunk_lossy(QueueHandle_t q, const uint8_t *data, size_t len)
{
    if (!q || !data || len == 0) {
        return;
    }
    size_t off = 0;
    while (off < len) {
        IoChunk c{};
        const size_t n = (len - off) < sizeof(c.data) ? (len - off) : sizeof(c.data);
        c.len = (uint16_t)n;
        memcpy(c.data, data + off, n);
        if (xQueueSend(q, &c, 0) != pdPASS) {
            /* Transparenz darf den RS485-RX nicht blockieren. Wenn der Host/Parser nicht
             * nachkommt, aeltesten Chunk verwerfen und den neuesten behalten. */
            IoChunk drop{};
            (void)xQueueReceive(q, &drop, 0);
            (void)xQueueSend(q, &c, 0);
        }
        off += n;
    }
}

static bool is_controller_poll_frame(const uint8_t *data, size_t len)
{
    if (!data || len < 8 || data[0] != '#') {
        return false;
    }
    int colon_count = 0;
    size_t cmd_start = 0;
    for (size_t i = 1; i < len; ++i) {
        if (data[i] == ':') {
            colon_count++;
            if (colon_count == 2) {
                cmd_start = i + 1;
                break;
            }
        } else if (data[i] == '$') {
            return false;
        }
    }
    if (cmd_start == 0 || cmd_start >= len) {
        return false;
    }
    const size_t remaining = len - cmd_start;
    if (remaining >= 3 && memcmp(data + cmd_start, "GET", 3) == 0) {
        return true;
    }
    return remaining >= 4 && memcmp(data + cmd_start, "TEST", 4) == 0;
}

/** Einmalige Boot-Leseabfragen (Versatz/Dipol/Oeffnungswinkel/Encoder/Namen): GETANTOFF, GETANTDP, GETANGLE, GETENCTYPE, GETMAXDG, GETANTNAME. */
static bool is_antenna_boot_read_frame(const uint8_t *data, size_t len)
{
    if (!data || len < 8 || data[0] != '#') {
        return false;
    }
    int colon_count = 0;
    size_t cmd_start = 0;
    for (size_t i = 1; i < len; ++i) {
        if (data[i] == ':') {
            colon_count++;
            if (colon_count == 2) {
                cmd_start = i + 1;
                break;
            }
        } else if (data[i] == '$') {
            return false;
        }
    }
    if (cmd_start == 0 || cmd_start >= len) {
        return false;
    }
    const size_t remaining = len - cmd_start;
    return (remaining >= 9 && memcmp(data + cmd_start, "GETANTOFF", 9) == 0) ||
           (remaining >= 8 && memcmp(data + cmd_start, "GETANTDP", 8) == 0) ||
           (remaining >= 8 && memcmp(data + cmd_start, "GETANGLE", 8) == 0) ||
           (remaining >= 10 && memcmp(data + cmd_start, "GETANTNAME", 10) == 0) ||
           (remaining >= 10 && memcmp(data + cmd_start, "GETENCTYPE", 10) == 0) ||
           (remaining >= 8 && memcmp(data + cmd_start, "GETMAXDG", 8) == 0) ||
           (remaining >= 12 && memcmp(data + cmd_start, "GETROTORTYPE", 12) == 0);
}

static void wait_bus_idle(uint32_t min_idle_us, uint32_t cap_ms)
{
    const uint32_t deadline_ms = millis() + cap_ms;
    for (;;) {
        uart_lock();
        const bool has_rx = (s_hw->available() > 0);
        uart_unlock();
        if (has_rx) {
            delayMicroseconds(120);
            taskYIELD();
            /* Fremd-Master pollt ohne Pause: UART nie leer → ohne Deadline-Abbruch wuerde
             * SETPOSCC (hw_send_priority) nie den TX erreichen. */
            if ((int32_t)(millis() - deadline_ms) >= 0) {
                delayMicroseconds(min_idle_us);
                return;
            }
            continue;
        }

        if ((uint32_t)(micros() - s_last_bus_activity_us) >= min_idle_us) {
            return;
        }

        if ((int32_t)(millis() - deadline_ms) >= 0) {
            delayMicroseconds(min_idle_us);
            return;
        }

        delayMicroseconds(120);
        taskYIELD();
    }
}

static void send_rs485_frame(const TxFrame &f)
{
    const bool from_pc = (f.flags & kTxFlagFromPc) != 0;
    const bool priority = (f.flags & kTxFlagPriority) != 0;

    if (s_remote_usb) {
        /* Hartes Mute: niemals UART schreiben — nur USB↔Parser. */
        force_rs485_tx_hi_z();
        if (from_pc) {
            enqueue_chunk_lossy(s_sniff_q, f.data, f.len);
        } else {
            enqueue_chunk_lossy(s_usb_tx_q, f.data, f.len);
        }
        return;
    }

    if (priority) {
        wait_bus_idle(kBusIdleUsPriority, kBusIdleWaitCapPriorityMs);
    } else if (from_pc) {
        wait_bus_idle(kBusIdleUsPc, kBusIdleWaitCapPcMs);
    } else {
        wait_bus_idle(kBusIdleUsCtrl, kBusIdleWaitCapCtrlMs);
    }

    uart_lock();
    s_hw->write(f.data, f.len);
    s_hw->flush();
    if (kTurnaroundMicros != 0) {
        delayMicroseconds(kTurnaroundMicros);
    }
    uart_unlock();
    s_last_bus_activity_us = micros();

    if (from_pc) {
        /* PC-Frames lokal mitsniffen: SETPOSDG/SETREF/SETASELECT und Config-Kommandos
         * aktualisieren State/UI, ohne den transparenten TX-Pfad zu blockieren. */
        enqueue_chunk_lossy(s_sniff_q, f.data, f.len);
    } else {
        /* Eigene Controller-Frames zum PC spiegeln, damit der PC denselben Busstrom sieht. */
        enqueue_chunk_lossy(s_usb_tx_q, f.data, f.len);
    }
}

static void task_rs485_tx(void *)
{
    for (;;) {
        TxFrame f{};
        if (s_tx_prio_q && xQueueReceive(s_tx_prio_q, &f, 0) == pdPASS) {
            send_rs485_frame(f);
            continue;
        }
        if (s_tx_q && xQueueReceive(s_tx_q, &f, pdMS_TO_TICKS(2)) == pdPASS) {
            send_rs485_frame(f);
            continue;
        }
        taskYIELD();
    }
}

static void task_rs485_rx(void *)
{
    for (;;) {
        uint8_t buf[kIoChunkMax];
        size_t n = 0;
        uart_lock();
        while (s_hw->available() && n < sizeof(buf)) {
            const int c = s_hw->read();
            if (c < 0) {
                break;
            }
            buf[n++] = (uint8_t)c;
        }
        uart_unlock();

        if (n == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        s_last_bus_activity_us = micros();
        if (s_remote_usb) {
            /* Remote: RS485-RX ignorieren — nur USB-Proxy darf den Parser füttern. */
            continue;
        }
        enqueue_chunk_lossy(s_usb_tx_q, buf, n);
        enqueue_chunk_lossy(s_sniff_q, buf, n);
    }
}

static void task_usb_tx(void *)
{
    IoChunk c{};
    for (;;) {
        if (!s_usb_tx_q || xQueueReceive(s_usb_tx_q, &c, pdMS_TO_TICKS(5)) != pdPASS) {
            continue;
        }
        size_t off = 0;
        while (off < c.len) {
            const int space = Serial.availableForWrite();
            if (space <= 0) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            size_t n = (size_t)space;
            if (n > (size_t)c.len - off) {
                n = (size_t)c.len - off;
            }
            const size_t w = Serial.write(c.data + off, n);
            if (w == 0) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            off += w;
        }
    }
}

static void task_sniffer(void *)
{
    IoChunk c{};
    for (;;) {
        if (!s_sniff_q || xQueueReceive(s_sniff_q, &c, pdMS_TO_TICKS(10)) != pdPASS) {
            continue;
        }
        rotor_rs485_rx_bytes(c.data, c.len);
    }
}

static bool looks_like_rs485_protocol_frame(const uint8_t *buf, size_t len)
{
    /* Mind. #n:n:CMD…$ — CDC-Rauschen / Enumeration darf keine PC-Session starten. */
    if (!buf || len < 10 || buf[0] != '#' || buf[len - 1] != '$') {
        return false;
    }
    for (size_t i = 1; i + 2 < len; i++) {
        if (buf[i] == ':') {
            const uint8_t c = buf[i + 1];
            if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
                return true;
            }
        }
    }
    return false;
}

static void flush_usb_rx_frame(uint8_t *buf, size_t *len)
{
    if (!buf || !len || *len == 0) {
        return;
    }
    if (s_remote_usb && !looks_like_rs485_protocol_frame(buf, *len)) {
        *len = 0;
        return;
    }
    const uint32_t now = millis();
    if (s_remote_usb) {
        /* Relink NUR wenn der Link vorher weg war (lange Pause / erster Kontakt) —
         * nicht bei jeder kurzen Sendepause der Software (sonst „Neustart“ alle paar Sekunden). */
        if (!s_remote_usb_link_up) {
            s_remote_usb_had_pc = true;
            s_remote_usb_link_up = true;
            rotor_rs485_relink();
        }
    }
    s_last_pc_frame_ms = now;
    if (!s_remote_usb) {
        s_mode = BridgeMode::PcProxyMaster;
    }
    (void)enqueue_tx_frame(buf, *len, kTxFlagFromPc);
    *len = 0;
}

static void task_usb_rx(void *)
{
    uint8_t frame[kFrameMax];
    size_t frame_len = 0;
    bool in_frame = false;

    for (;;) {
        bool read_any = false;
        while (Serial.available()) {
            const int c = Serial.read();
            if (c < 0) {
                break;
            }
            read_any = true;
            /* Kein s_last_pc_frame_ms hier — nur vollständige #…$ Frames zählen als PC-Aktivität.
             * Sonst halten CDC-Rauschen/Enumeration die „PC online“-Stille künstlich frisch. */

            const uint8_t b = (uint8_t)c;
            if (!in_frame) {
                if (b == '#') {
                    in_frame = true;
                    frame_len = 0;
                    frame[frame_len++] = b;
                }
                continue;
            }

            if (frame_len < sizeof(frame)) {
                frame[frame_len++] = b;
            } else {
                in_frame = false;
                frame_len = 0;
                continue;
            }

            if (b == '$' && frame_len >= 5) {
                flush_usb_rx_frame(frame, &frame_len);
                in_frame = false;
            } else if (b == '#') {
                frame_len = 1;
                frame[0] = '#';
            }
        }
        vTaskDelay(read_any ? 0 : pdMS_TO_TICKS(1));
    }
}

void hw_send(const uint8_t *data, size_t len)
{
    /* Mitläufer (USB-Proxy oder Fremd-Master am RS485): keine eigenen GET/TEST — SETPOSCC bleibt hw_send_priority.
     * Remote USB: eigene GET/TEST sind der Normalfall und müssen zum PC. */
    if (!s_remote_usb && rotor_rs485_is_foreign_pc_listen_mode() && is_controller_poll_frame(data, len)) {
        /* Ausnahme: einmalige Versatz-/Dipol-/Winkel-Boot-Reads müssen auch als Mitläufer auf den Bus,
         * sonst kennt der Controller die Antennenversätze nie (stehen nur im Rotor-RAM). */
        if (!(rotor_rs485_boot_read_in_progress() && is_antenna_boot_read_frame(data, len))) {
            return;
        }
    }
    (void)enqueue_tx_frame(data, len, 0u);
}

void hw_send_priority(const uint8_t *data, size_t len)
{
    (void)enqueue_tx_frame(data, len, kTxFlagPriority);
}

void begin()
{
    rotor_rs485_pre_begin();

    s_remote_usb = (pwm_config_get_remote_usb() != 0u);
    s_remote_usb_had_pc = false;
    s_remote_usb_link_up = false;
    s_last_pc_frame_ms = 0;

    if (!s_uart_mutex) {
        s_uart_mutex = xSemaphoreCreateMutex();
    }
    if (!s_tx_q) {
        s_tx_q = xQueueCreate(kTxQueueDepth, sizeof(TxFrame));
    }
    if (!s_tx_prio_q) {
        s_tx_prio_q = xQueueCreate(kTxPriorityQueueDepth, sizeof(TxFrame));
    }
    if (!s_usb_tx_q) {
        s_usb_tx_q = xQueueCreate(kUsbTxQueueDepth, sizeof(IoChunk));
    }
    if (!s_sniff_q) {
        s_sniff_q = xQueueCreate(kSniffQueueDepth, sizeof(IoChunk));
    }

    s_hw->setRxBufferSize(4096);
    s_hw->setTxBufferSize(2048);
    if (s_remote_usb) {
        force_rs485_tx_hi_z();
        s_hw->begin(s_baud, SERIAL_8N1, kPinUartRx, -1);
        force_rs485_tx_hi_z();
    } else {
        s_hw->begin(s_baud, SERIAL_8N1, kPinUartRx, kPinUartTx);
    }

    if (!s_task_usb_rx) {
        xTaskCreatePinnedToCore(task_usb_rx, "usb_rx", 4096, nullptr, 4, &s_task_usb_rx, 1);
    }
    if (!s_task_rs485_rx) {
        xTaskCreatePinnedToCore(task_rs485_rx, "rs485_rx", 4096, nullptr, 5, &s_task_rs485_rx, 1);
    }
    if (!s_task_rs485_tx) {
        xTaskCreatePinnedToCore(task_rs485_tx, "rs485_tx", 4096, nullptr, 4, &s_task_rs485_tx, 1);
    }
    if (!s_task_usb_tx) {
        xTaskCreatePinnedToCore(task_usb_tx, "usb_tx", 4096, nullptr, 3, &s_task_usb_tx, 1);
    }
    if (!s_task_sniffer) {
        /* Gleiches Niveau wie USB-RX/TX: Parser darf nicht hinter den RX-Bursts zurückfallen. */
        xTaskCreatePinnedToCore(task_sniffer, "rs485_sniff", 4096, nullptr, 4, &s_task_sniffer, 1);
    }
}

void poll()
{
    if (s_remote_usb) {
        /* TX-Pin periodisch hochohmig halten (begin kann den Pin wieder umschalten). */
        static uint32_t s_last_hiz_ms = 0;
        const uint32_t now_hiz = millis();
        if ((uint32_t)(now_hiz - s_last_hiz_ms) >= 500u) {
            s_last_hiz_ms = now_hiz;
            force_rs485_tx_hi_z();
        }
        if (s_remote_usb_link_up && s_last_pc_frame_ms != 0 &&
            (uint32_t)(millis() - s_last_pc_frame_ms) > kRemoteUsbLinkDownMs) {
            s_remote_usb_link_up = false;
            /* Meldetext „Warten auf PC“; kein Relink hier — erst beim Wiederkommen. */
        }
        return;
    }
    if (s_mode == BridgeMode::PcProxyMaster) {
        if ((uint32_t)(millis() - s_last_pc_frame_ms) > kPcProxyAutoSilenceMs) {
            s_mode = BridgeMode::LocalMaster;
        }
    }
}

} // namespace serial_bridge
