/**
 * Transparente Brücke: alles von USB Serial -> HW-UART, HW-UART -> USB.
 * RS485: vor Senden DE/RE umschalten, nach flush wieder Empfang.
 *
 * UART: Serial2 (Serial1 bleibt z. B. für andere Module wie Signals frei).
 */

#include "serial_bridge.h"

namespace serial_bridge {

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

void begin()
{
    pinMode(kPinRs485Dir, OUTPUT);
    dir_receive();

    s_hw->begin(s_baud, SERIAL_8N1, kPinUartRx, kPinUartTx);
    s_hw->setRxBufferSize(1024);
    s_hw->setTxBufferSize(1024);
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

    dir_transmit();
    s_hw->write(buf, n);
    s_hw->flush();
    if (kTurnaroundMicros) {
        delayMicroseconds(kTurnaroundMicros);
    }
    dir_receive();
}

static void pump_hw_to_usb()
{
    while (s_hw->available()) {
        int c = s_hw->read();
        if (c < 0) {
            break;
        }
        Serial.write(static_cast<uint8_t>(c));
    }
}

void poll()
{
    pump_usb_to_hw();
    pump_hw_to_usb();
    yield();
}

} // namespace serial_bridge
