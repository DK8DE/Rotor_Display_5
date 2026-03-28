#ifndef SIGNALS_H
#define SIGNALS_H

#include <Arduino.h>

#if !defined(ARDUINO_ARCH_ESP32)
  #error "Signals ist fuer ESP32 (ARDUINO_ARCH_ESP32) vorgesehen."
#endif

/*
  Signals - Arduino Bibliothek (ESP32)
  -----------------------------------
  Diese Bibliothek sendet Steuerbefehle (ASCII-Protokoll) an einen ATtiny412,
  der als Empfaenger arbeitet und NeoPixel + Speaker ansteuert.

  Protokoll (eine Zeile pro Befehl, Abschluss mit '\n'):
    P i r g b br        -> Pixel setzen
    A r g b br          -> Alle LEDs setzen
    U 0|1               -> Auto-Show aus/an
    W                   -> show()
    C                   -> clear
    T freq vol [dur_ms] -> Ton (vol 0..50)
    X                   -> Ton aus

  Hinweis:
  - Der ESP32 sendet nur (TX). RX wird nicht benoetigt.
*/

class Signals
{
public:
  /*
    Konstruktor
    serial: HardwareSerial Instanz (z.B. Serial1 oder Serial2)
  */
  explicit Signals(HardwareSerial& serial);

  /*
    Initialisiert die serielle Schnittstelle.
    txPin: GPIO-Pin, auf dem der ESP32 die Daten zum ATtiny sendet
    rxPin: optional (Standard -1). Wird nicht benoetigt, kann aber gesetzt werden.
    baud:  Standard 115200 (muss zum ATtiny passen)
    invert: optional (Standard false). Fuer invertierte Pegel.
  */
  bool begin(int8_t txPin, int8_t rxPin = -1, uint32_t baud = 115200, bool invert = false);

  /*
    NeoPixel
  */
  void setAutoShow(bool enable);
  void show();
  void clear();

  void setAll(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
  void setPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

  /*
    Speaker
    freq: 200..2000 Hz (wird im Sender nicht hart begrenzt; Empfaenger clamp't ggf.)
    vol:  0..50 (0 = aus)
    durationMs: optional, 0 = dauerhaft bis stopTone()
  */
  void tone(uint16_t freq, uint8_t vol, uint16_t durationMs = 0);
  void stopTone();

private:
  HardwareSerial& _serial;
  bool _started = false;

  // Sendet eine komplette Zeile (inkl. \n)
  void sendLine(const char* line);

  // Hilfsfunktion zum sicheren Begrenzen
  static uint8_t clampU8(uint16_t v, uint8_t maxVal);
};

#endif // SIGNALS_H
