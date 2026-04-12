/**
 * Transparente Brücke: USB Serial <-> HW-UART (RS485, Halbduplex).
 *
 * RS485 = ein Sender zu jeder Zeit. Kollisionen entstehen, wenn der Controller
 * PC-Daten auf den Bus schickt, während der Rotor noch eine Antwort sendet —
 * oder wenn der Controller (rotor_rs485 / hw_send) zu früh nach einer Slave-
 * Antwort wieder sendet, bevor Halbduplex/MAX485 sauber umgeschaltet ist.
 *
 * Schutz: Zeitbasierte Bus-Idle — nach jeder Aktivität (TX oder RX) mindestens
 * kBusIdleUs Stille, bevor erneut gesendet wird. Gilt jetzt auch fuer hw_send(),
 * nicht nur fuer USB->Bus (pump_usb_to_hw).
 */

#include "serial_bridge.h"
#include "rotor_rs485.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace serial_bridge {

static SemaphoreHandle_t s_uart_mutex;

// --- Konfiguration (Board / Verdrahtung) ---------------------------------

/** TX-Leitung zum RS485-Wandler */
static constexpr int kPinUartTx = 43;
/** RX vom RS485-Wandler */
static constexpr int kPinUartRx = 44;
/** DE/RE bzw. Sende-/Empfangsumschaltung (typ. MAX485: HIGH = Senden aktiv) */
static constexpr int kPinRs485Dir = 39;

/** true: Dir-Pin HIGH = Senden; false: invertierte Ansteuerung */
static constexpr bool kDirTransmitLevel = true;

/** Puffer für USB -> HW (weniger Umschalten als Byte für Byte) */
static constexpr size_t kUsbToHwBuf = 256;
/** HW -> USB + Parser: Blockgröße (Schleife leert UART vollständig pro poll()) */
static constexpr size_t kHwToUsbBuf = 512;

/** Wartezeit nach letztem gesendeten Byte, bevor wieder Empfang (Bus-Freigabe) */
static constexpr uint32_t kTurnaroundMicros = 250;

/**
 * Mindest-Stille vor erneutem Senden (Controller + USB). 12 ms ≈ 138 Byte-Zeiten
 * @ 115200 — groessere Marge fuer MAX485/Slave-Umschaltung (sonst lange GETREF-Timeouts).
 */
static constexpr uint32_t kBusIdleUs = 12000;

/** Max. Warten auf Bus-Idle (ms); muss deutlich > kBusIdleUs sein, sonst Senden ohne Pause. */
static constexpr uint32_t kBusIdleWaitCapMs = 200;

static HardwareSerial *s_hw = &Serial2;
static uint32_t s_baud = 115200;

/** Zeitstempel der letzten Bus-Aktivität (TX oder RX), micros(). */
static uint32_t s_last_bus_activity_us = 0;

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

/** USB-Monitor: nur bis Puffer frei — nie blockieren (ohne PC-Leser: sonst Parser/Anzeige verzögert). */
static void mirror_usb_nonblocking(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return;
    }
    size_t off = 0;
    while (off < len) {
        const int space = Serial.availableForWrite();
        if (space <= 0) {
            break;
        }
        const size_t chunk =
            (size_t)space < (len - off) ? (size_t)space : (len - off);
        const size_t w = Serial.write(data + off, chunk);
        if (w == 0) {
            break;
        }
        off += w;
    }
}

/** Rest-RX leeren (zwischen poll() und hw_send eingetroffen) — Parser wie in pump_hw. */
static void drain_hw_rx_quick()
{
    for (;;) {
        uint8_t buf[128];
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
            return;
        }
        s_last_bus_activity_us = micros();
        rotor_rs485_rx_bytes(buf, n);
        mirror_usb_nonblocking(buf, n);
    }
}

/**
 * Vor Controller-Telegramm (GETREF, GETPOS, …): gleiche Idle-Regel wie bei USB->Bus,
 * damit nicht im selben loop()-Takt direkt nach Slave-TX wieder gesendet wird.
 */
static void wait_bus_idle_for_controller_tx()
{
    const uint32_t deadline_ms = millis() + kBusIdleWaitCapMs;
    for (;;) {
        drain_hw_rx_quick();

        uart_lock();
        const bool has_rx = (s_hw->available() > 0);
        uart_unlock();
        if (has_rx) {
            delayMicroseconds(200);
            yield();
            continue;
        }

        if ((uint32_t)(micros() - s_last_bus_activity_us) >= kBusIdleUs) {
            return;
        }

        if ((int32_t)(millis() - deadline_ms) >= 0) {
            /* Kein volles Idle erreicht (z. B. Dauer-RX): trotzdem Mindestluecke erzwingen */
            delayMicroseconds(kBusIdleUs);
            return;
        }

        delayMicroseconds(200);
        yield();
    }
}

void hw_send(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return;
    }
    wait_bus_idle_for_controller_tx();

    uart_lock();
    dir_transmit();
    s_hw->write(data, len);
    s_hw->flush();
    if (kTurnaroundMicros) {
        delayMicroseconds(kTurnaroundMicros);
    }
    dir_receive();
    uart_unlock();
    s_last_bus_activity_us = micros();
    mirror_usb_nonblocking(data, len);
}

void begin()
{
    if (!s_uart_mutex) {
        s_uart_mutex = xSemaphoreCreateMutex();
    }
    pinMode(kPinRs485Dir, OUTPUT);
    dir_receive();

    /* Muss vor begin() — sonst [E] HardwareSerial „can't be resized when already running" */
    /* Großer RX: bei vielen GETPOSDG-ACKs + LVGL darf der Puffer nicht überlaufen (sonst Nachlauf-Anzeige). */
    s_hw->setRxBufferSize(4096);
    s_hw->setTxBufferSize(2048);
    s_hw->begin(s_baud, SERIAL_8N1, kPinUartRx, kPinUartTx);
}

/** true wenn der Bus seit mindestens kBusIdleUs still ist UND keine RX-Bytes anliegen. */
static bool bus_clear_for_tx()
{
    uart_lock();
    const bool has_rx = (s_hw->available() > 0);
    uart_unlock();
    if (has_rx) {
        return false;
    }
    return (uint32_t)(micros() - s_last_bus_activity_us) >= kBusIdleUs;
}

/**
 * USB → RS485: PC-Daten auf den Bus schicken.
 * Nur wenn der Bus nachweislich idle ist (kein RX, kBusIdleUs Stille).
 * Sonst bleiben die PC-Daten im USB-CDC-Puffer → nächstes poll().
 */
static void pump_usb_to_hw()
{
    if (!bus_clear_for_tx()) {
        return;
    }

    uint8_t buf[kUsbToHwBuf];
    size_t n = 0;
    while (Serial.available() && n < sizeof(buf)) {
        int c = Serial.read();
        if (c < 0) {
            break;
        }
        buf[n++] = static_cast<uint8_t>(c);
    }
    if (n == 0) {
        return;
    }

    uart_lock();
    dir_transmit();
    s_hw->write(buf, n);
    s_hw->flush();
    if (kTurnaroundMicros) {
        delayMicroseconds(kTurnaroundMicros);
    }
    dir_receive();
    uart_unlock();
    s_last_bus_activity_us = micros();
    rotor_rs485_rx_bytes(buf, n);
}

/** UART → USB + Parser: komplett leeren (mehrere Blöcke). */
static void pump_hw_to_usb_and_parser()
{
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
            return;
        }
        s_last_bus_activity_us = micros();
        /* Zuerst Parser: SETPOSDG/taget_dg sofort — USB-Spiegel darf nicht vor RX-Parser blockieren */
        rotor_rs485_rx_bytes(buf, n);
        mirror_usb_nonblocking(buf, n);
    }
}

void poll()
{
    /* ERST Bus leeren (Rotor-Antworten), DANN (nur wenn Bus idle) PC-Daten senden. */
    pump_hw_to_usb_and_parser();
    pump_usb_to_hw();
    yield();
}

} // namespace serial_bridge
