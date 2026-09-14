#ifndef SIGNALS_H
#define SIGNALS_H

#include <Arduino.h>

#if !defined(ARDUINO_ARCH_ESP32)
  #error "Signals ist fuer ESP32 (ARDUINO_ARCH_ESP32) vorgesehen."
#endif

/*
  Signals — direkte Hardware-Ansteuerung (kein ATtiny mehr)
  ---------------------------------------------------------
  - NeoPixel-Ring: WS2812 an GPIO (Standard 39)
  - Lautsprecher: PWM/LEDC an GPIO (Standard 40)

  API bleibt kompatibel zur frueheren UART→ATtiny-Variante:
  - setPixel / setAll / clear / show / setAutoShow
  - tone / stopTone / restartTone (vol 0…50 wie SETLSL)
*/

class Adafruit_NeoPixel;
struct esp_timer;

class Signals
{
public:
  Signals();

  /**
   * NeoPixel + Speaker initialisieren.
   * numLeds: Anzahl WS2812 (Standard 16)
   * neoPixelPin: Datenleitung Ring (Standard 39)
   * speakerPin: PWM-Lautsprecher (Standard 40)
   */
  bool begin(uint8_t numLeds = 16, int8_t neoPixelPin = 39, int8_t speakerPin = 40);

  /** Fallback fuer Ton-Auto-Stop (Hauptpfad: esp_timer, unabhaengig vom Loop). */
  void service();

  /* NeoPixel */
  void setAutoShow(bool enable);
  void show();
  void clear();
  void setAll(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
  void setPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

  /*
    Speaker
    freq: Hz (typ. 200…4000 wie SETCONFRQ)
    vol:  0…50 (0 = aus; gleiche Skala wie SETLSL)
    durationMs: 0 = dauerhaft bis stopTone()
  */
  void tone(uint16_t freq, uint8_t vol, uint16_t durationMs = 0);
  void stopTone();
  void restartTone(uint16_t freq, uint8_t vol, uint16_t durationMs = 0);

private:
  Adafruit_NeoPixel *_strip = nullptr;
  bool _started = false;
  bool _autoShow = true;
  int8_t _speakerPin = -1;
  uint8_t _numLeds = 0;

  bool _toneActive = false;
  uint32_t _toneStopMs = 0; /* Fallback; 0 = kein Auto-Stop */
  struct esp_timer *_toneTimer = nullptr;

  void applyPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
  void speakerStart(uint16_t freq, uint8_t vol);
  void speakerStopHw();
  void cancelToneTimer();
  void armToneTimer(uint16_t durationMs);
  void stopToneLocked();

  static void toneTimerCallback(void *arg);
  static uint8_t clampU8(uint16_t v, uint8_t maxVal);
};

#endif // SIGNALS_H
