#include "Signals.h"

Signals::Signals(HardwareSerial& serial)
: _serial(serial)
{
}

bool Signals::begin(int8_t txPin, int8_t rxPin, uint32_t baud, bool invert)
{
  // ESP32 Arduino: HardwareSerial::begin(baud, config, rxPin, txPin, invert)
  // RX kann -1 sein, wenn nicht genutzt.
  _serial.begin(baud, SERIAL_8N1, rxPin, txPin, invert);
  _started = true;

  // Wir koennen ohne RX nicht wirklich pruefen, ob es "wirklich" klappt.
  // Daher immer true.
  return true;
}

void Signals::sendLine(const char* line)
{
  if (!_started || !line) return;
  _serial.print(line);
  _serial.print('\n');
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

  if (durationMs > 0)
  {
    // T freq vol dur
    char buf[32];
    snprintf(buf, sizeof(buf), "T %u %u %u", (unsigned)freq, (unsigned)vol, (unsigned)durationMs);
    sendLine(buf);
  }
  else
  {
    // T freq vol
    char buf[24];
    snprintf(buf, sizeof(buf), "T %u %u", (unsigned)freq, (unsigned)vol);
    sendLine(buf);
  }
  /* Zeile vollständig zum ATtiny schicken — sonst können schnelle Folgebefehle zerstückeln (Dauerton). */
  _serial.flush();
}

void Signals::stopTone()
{
  // X
  sendLine("X");
  _serial.flush();
}
