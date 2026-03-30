/**
 * Transparente Brücke: alles von USB Serial -> HW-UART, HW-UART -> USB.
 * RS485: vor Senden DE/RE umschalten, nach flush wieder Empfang.
 *
 * UART: Serial2 (Serial1 bleibt z. B. für andere Module wie Signals frei).
 */

#include "serial_bridge.h"
#include "rotor_rs485.h"

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
/** HW -> USB + Parser (eine gemeinsame Lesepfad, kein „Verschlucken“ der Antworten) */
static constexpr size_t kHwToUsbBuf = 512;

/** Wartezeit nach letztem gesendeten Byte, bevor wieder Empfang (Bus-Freigabe) */
static constexpr uint32_t kTurnaroundMicros = 80;

static HardwareSerial *s_hw = &Serial2;
static uint32_t s_baud = 115200;

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

void hw_send(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return;
    }
    uart_lock();
    dir_transmit();
    s_hw->write(data, len);
    s_hw->flush();
    if (kTurnaroundMicros) {
        delayMicroseconds(kTurnaroundMicros);
    }
    dir_receive();
    uart_unlock();
    /* Lokale Master-Telegramme (SETPOSDG, GETREF, …) erscheinen sonst nicht am USB-Monitor */
    Serial.write(data, len);
}

void begin()
{
    if (!s_uart_mutex) {
        s_uart_mutex = xSemaphoreCreateMutex();
    }
    pinMode(kPinRs485Dir, OUTPUT);
    dir_receive();

    /* Muss vor begin() — sonst [E] HardwareSerial „can't be resized when already running“ */
    s_hw->setRxBufferSize(1024);
    s_hw->setTxBufferSize(1024);
    s_hw->begin(s_baud, SERIAL_8N1, kPinUartRx, kPinUartTx);
}

static void pump_usb_to_hw()
{
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
    /*
     * Lokaler „Loopback“ für den Protokoll-Parser: Was vom PC über USB auf den Bus geht,
     * steht am UART meist nicht wieder auf RX an (kein Echo). Ohne diese Zeile sieht
     * rotor_rs485 SETPOSDG vom PC nicht — taget_dg würde nicht aktualisiert.
     */
    rotor_rs485_rx_bytes(buf, n);
}

static void pump_hw_to_usb_and_parser()
{
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
    Serial.write(buf, n);
    rotor_rs485_rx_bytes(buf, n);
}

void poll()
{
    /* USB -> Bus, dann alles vom RS485 lesen: an PC spiegeln und an rotor_rs485 füttern */
    pump_usb_to_hw();
    pump_hw_to_usb_and_parser();
    yield();
}

} // namespace serial_bridge
