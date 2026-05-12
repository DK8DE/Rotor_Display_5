/**
 * Transparente Bruecke: USB Serial <-> HW-UART (RS485, Halbduplex).
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
/** DE/RE bzw. Sende-/Empfangsumschaltung (typ. MAX485: HIGH = Senden aktiv) */
static constexpr int kPinRs485Dir = 39;
/** true: Dir-Pin HIGH = Senden; false: invertierte Ansteuerung */
static constexpr bool kDirTransmitLevel = true;

/** Max. RS485-Telegramm inkl. '$' */
static constexpr size_t kFrameMax = 320;
/** UART-RX Lese-Chunk; Frames bleiben im Stream erhalten. */
static constexpr size_t kIoChunkMax = 256;

/** Queue-Tiefen: genug Puffer fuer kurze PC-Bursts, ohne RX/USB dauerhaft zu blockieren. */
static constexpr size_t kTxQueueDepth = 64;
static constexpr size_t kTxPriorityQueueDepth = 12;
static constexpr size_t kUsbTxQueueDepth = 96;
static constexpr size_t kSniffQueueDepth = 96;

/** Wartezeit nach letztem gesendeten Byte, bevor wieder Empfang (Bus-Freigabe). */
static constexpr uint32_t kTurnaroundMicros = 250;
/** Mindest-Stille vor PC- oder Controller-Frames. */
static constexpr uint32_t kBusIdleUsPc = 1400;
static constexpr uint32_t kBusIdleUsCtrl = 12000;
static constexpr uint32_t kBusIdleUsPriority = 1800;
static constexpr uint32_t kBusIdleWaitCapPcMs = 25;
static constexpr uint32_t kBusIdleWaitCapCtrlMs = 200;
static constexpr uint32_t kBusIdleWaitCapPriorityMs = 35;
static constexpr uint32_t kPcProxyAutoSilenceMs = 12000;

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

void set_mode(BridgeMode mode)
{
    s_mode = mode;
}

BridgeMode get_mode()
{
    return s_mode;
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

    if (priority) {
        wait_bus_idle(kBusIdleUsPriority, kBusIdleWaitCapPriorityMs);
    } else if (from_pc) {
        wait_bus_idle(kBusIdleUsPc, kBusIdleWaitCapPcMs);
    } else {
        wait_bus_idle(kBusIdleUsCtrl, kBusIdleWaitCapCtrlMs);
    }

    uart_lock();
    dir_transmit();
    s_hw->write(f.data, f.len);
    s_hw->flush();
    if (kTurnaroundMicros != 0) {
        delayMicroseconds(kTurnaroundMicros);
    }
    dir_receive();
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

static void flush_usb_rx_frame(uint8_t *buf, size_t *len)
{
    if (!buf || !len || *len == 0) {
        return;
    }
    s_last_pc_frame_ms = millis();
    s_mode = BridgeMode::PcProxyMaster;
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
            s_last_pc_frame_ms = millis();
            s_mode = BridgeMode::PcProxyMaster;

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
    /* Mitläufer (USB-Proxy oder Fremd-Master am RS485): keine eigenen GET/TEST — SETPOSCC bleibt hw_send_priority. */
    if (rotor_rs485_is_foreign_pc_listen_mode() && is_controller_poll_frame(data, len)) {
        return;
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

    pinMode(kPinRs485Dir, OUTPUT);
    dir_receive();

    s_hw->setRxBufferSize(4096);
    s_hw->setTxBufferSize(2048);
    s_hw->begin(s_baud, SERIAL_8N1, kPinUartRx, kPinUartTx);

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
        xTaskCreatePinnedToCore(task_sniffer, "rs485_sniff", 4096, nullptr, 2, &s_task_sniffer, 1);
    }
}

void poll()
{
    if (s_mode == BridgeMode::PcProxyMaster) {
        if ((uint32_t)(millis() - s_last_pc_frame_ms) > kPcProxyAutoSilenceMs) {
            s_mode = BridgeMode::LocalMaster;
        }
    }
}

} // namespace serial_bridge
