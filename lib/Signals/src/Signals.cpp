#include "Signals.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/** Ein ATtiny-Protokoll: eine Zeile pro Befehl — gleichzeitige Sender (LVGL-Touch vs. NeoPixel-Ring) ohne Mutex
 * vermischen Bytes → kaputte T-/P-Zeilen, wegfallende Pieps, falscher Klang. */
static SemaphoreHandle_t s_signals_tx_mutex = nullptr;

Signals::Signals(HardwareSerial& serial)
: _serial(serial)
{
}

static void send_one_line_unlocked(HardwareSerial &ser, const char *line)
{
    ser.print(line);
    ser.print('\n');
}

bool Signals::begin(int8_t txPin, int8_t rxPin, uint32_t baud, bool invert)
{
  // ESP32 Arduino: HardwareSerial::begin(baud, config, rxPin, txPin, invert)
  // RX kann -1 sein, wenn nicht genutzt.
  _serial.begin(baud, SERIAL_8N1, rxPin, txPin, invert);
  _started = true;
  if (s_signals_tx_mutex == nullptr) {
      s_signals_tx_mutex = xSemaphoreCreateMutex();
  }

  // Wir koennen ohne RX nicht wirklich pruefen, ob es "wirklich" klappt.
  // Daher immer true.
  return true;
}

void Signals::sendLine(const char* line)
{
  if (!_started || !line) {
      return;
  }
  if (s_signals_tx_mutex == nullptr) {
      send_one_line_unlocked(_serial, line);
      return;
  }
  xSemaphoreTake(s_signals_tx_mutex, portMAX_DELAY);
  send_one_line_unlocked(_serial, line);
  xSemaphoreGive(s_signals_tx_mutex);
}

uint8_t Signals::clampU8(uint16_t v, uint8_t maxVal)
{
  if (v > maxVal) return maxVal;
  return (uint8_t)v;
}

/* ------------------------------- NeoPixel -------------------------------- */

void Signals::setAutoShow(bool enable)
{
  // U 0|1
  char buf[8];
  snprintf(buf, sizeof(buf), "U %u", enable ? 1u : 0u);
  sendLine(buf);
}

void Signals::show()
{
  // W
  sendLine("W");
}

void Signals::clear()
{
  // C
  sendLine("C");
}

void Signals::setAll(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
  // A r g b br
  char buf[32];
  snprintf(buf, sizeof(buf), "A %u %u %u %u", (unsigned)r, (unsigned)g, (unsigned)b, (unsigned)brightness);
  sendLine(buf);
}

void Signals::setPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
  // P i r g b br
  char buf[40];
  snprintf(buf, sizeof(buf), "P %u %u %u %u %u", (unsigned)index, (unsigned)r, (unsigned)g, (unsigned)b, (unsigned)brightness);
  sendLine(buf);
}

/* -------------------------------- Speaker -------------------------------- */

void Signals::tone(uint16_t freq, uint8_t vol, uint16_t durationMs)
{
  // Lautstaerke 0..50 wie im Empfaenger-Protokoll
  vol = clampU8(vol, 50);

  char buf[32];
  if (durationMs > 0) {
    snprintf(buf, sizeof(buf), "T %u %u %u", (unsigned)freq, (unsigned)vol, (unsigned)durationMs);
  } else {
    snprintf(buf, sizeof(buf), "T %u %u", (unsigned)freq, (unsigned)vol);
  }
  /* Zeile + flush in einem Mutex-Block — sonst kann der Ring die UART zwischen Zeile und flush stören. */
  if (s_signals_tx_mutex == nullptr) {
      send_one_line_unlocked(_serial, buf);
      _serial.flush();
      return;
  }
  xSemaphoreTake(s_signals_tx_mutex, portMAX_DELAY);
  send_one_line_unlocked(_serial, buf);
  _serial.flush();
  xSemaphoreGive(s_signals_tx_mutex);
}

void Signals::stopTone()
{
  if (s_signals_tx_mutex == nullptr) {
      send_one_line_unlocked(_serial, "X");
      _serial.flush();
      return;
  }
  xSemaphoreTake(s_signals_tx_mutex, portMAX_DELAY);
  send_one_line_unlocked(_serial, "X");
  _serial.flush();
  xSemaphoreGive(s_signals_tx_mutex);
}

void Signals::restartTone(uint16_t freq, uint8_t vol, uint16_t durationMs)
{
  vol = clampU8(vol, 50);

  char tbuf[32];
  if (durationMs > 0) {
    snprintf(tbuf, sizeof(tbuf), "T %u %u %u", (unsigned)freq, (unsigned)vol, (unsigned)durationMs);
  } else {
    snprintf(tbuf, sizeof(tbuf), "T %u %u", (unsigned)freq, (unsigned)vol);
  }

  // X + T als eine atomare Sequenz senden, damit Ring-Updates nicht dazwischen landen.
  if (s_signals_tx_mutex == nullptr) {
      send_one_line_unlocked(_serial, "X");
      send_one_line_unlocked(_serial, tbuf);
      _serial.flush();
      return;
  }
  xSemaphoreTake(s_signals_tx_mutex, portMAX_DELAY);
  send_one_line_unlocked(_serial, "X");
  send_one_line_unlocked(_serial, tbuf);
  _serial.flush();
  xSemaphoreGive(s_signals_tx_mutex);
}
