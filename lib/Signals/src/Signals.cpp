#include "Signals.h"

#include <Adafruit_NeoPixel.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define SIGNALS_LEDC_V3 1
#else
#define SIGNALS_LEDC_V3 0
#endif

#if !SIGNALS_LEDC_V3
static constexpr int kSpeakerLecChannel = 2;
#endif

static constexpr uint8_t kSpeakerPwmBits = 10;
static constexpr uint32_t kSpeakerPwmMax = (1u << kSpeakerPwmBits) - 1u;
/** vol 0…50 → Duty; 50 % bei vol=50 (Rechteck, typ. lauteste hörbare Stufe). */
static constexpr uint32_t kSpeakerDutyAtVol50 = kSpeakerPwmMax / 2u;

static SemaphoreHandle_t s_signals_mutex = nullptr;

Signals::Signals() = default;

bool Signals::begin(uint8_t numLeds, int8_t neoPixelPin, int8_t speakerPin)
{
  if (numLeds == 0 || neoPixelPin < 0 || speakerPin < 0) {
    return false;
  }

  if (s_signals_mutex == nullptr) {
    s_signals_mutex = xSemaphoreCreateMutex();
  }

  if (_strip != nullptr) {
    delete _strip;
    _strip = nullptr;
  }

  _numLeds = numLeds;
  _speakerPin = speakerPin;
  _strip = new Adafruit_NeoPixel(numLeds, (int16_t)neoPixelPin, NEO_GRB + NEO_KHZ800);
  if (_strip == nullptr) {
    return false;
  }
  _strip->begin();
  _strip->clear();
  _strip->show();
  _autoShow = true;

#if SIGNALS_LEDC_V3
  /* Pin einmal anbinden; Frequenz/Duty werden in speakerStart gesetzt. */
  ledcAttach((uint8_t)speakerPin, 1000, kSpeakerPwmBits);
  ledcWrite((uint8_t)speakerPin, 0);
#else
  ledcSetup(kSpeakerLecChannel, 1000, kSpeakerPwmBits);
  ledcAttachPin((uint8_t)speakerPin, kSpeakerLecChannel);
  ledcWrite(kSpeakerLecChannel, 0);
#endif

  if (_toneTimer == nullptr) {
    const esp_timer_create_args_t args = {
        .callback = &Signals::toneTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "sig_tone",
        .skip_unhandled_events = true,
    };
    if (esp_timer_create(&args, &_toneTimer) != ESP_OK) {
      _toneTimer = nullptr;
    }
  }

  _toneActive = false;
  _toneStopMs = 0;
  _started = true;
  return true;
}

void Signals::toneTimerCallback(void *arg)
{
  Signals *self = static_cast<Signals *>(arg);
  if (self) {
    self->stopTone();
  }
}

void Signals::cancelToneTimer()
{
  if (_toneTimer) {
    esp_timer_stop(_toneTimer);
  }
}

void Signals::armToneTimer(uint16_t durationMs)
{
  cancelToneTimer();
  if (_toneTimer == nullptr || durationMs == 0) {
    return;
  }
  /* exakt wie früher ATtiny: Dauer unabhängig von LVGL/main-loop */
  (void)esp_timer_start_once(_toneTimer, (uint64_t)durationMs * 1000ull);
}

void Signals::service()
{
  /* Fallback, falls Timer fehlt — Hauptpfad ist esp_timer. */
  if (!_started || !_toneActive || _toneStopMs == 0) {
    return;
  }
  if ((int32_t)(millis() - _toneStopMs) >= 0) {
    stopTone();
  }
}

uint8_t Signals::clampU8(uint16_t v, uint8_t maxVal)
{
  if (v > maxVal) {
    return maxVal;
  }
  return (uint8_t)v;
}

void Signals::applyPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
  if (_strip == nullptr || index >= _numLeds) {
    return;
  }
  /* brightness 0…100 wie bisheriges ATtiny-Protokoll / Ring-App */
  brightness = clampU8(brightness, 100);
  const uint16_t br = brightness;
  const uint8_t rr = (uint8_t)((uint16_t)r * br / 100u);
  const uint8_t gg = (uint8_t)((uint16_t)g * br / 100u);
  const uint8_t bb = (uint8_t)((uint16_t)b * br / 100u);
  _strip->setPixelColor(index, _strip->Color(rr, gg, bb));
}

/* ------------------------------- NeoPixel -------------------------------- */

void Signals::setAutoShow(bool enable)
{
  _autoShow = enable;
}

void Signals::show()
{
  if (!_started || _strip == nullptr) {
    return;
  }
  if (s_signals_mutex) {
    xSemaphoreTake(s_signals_mutex, portMAX_DELAY);
  }
  _strip->show();
  if (s_signals_mutex) {
    xSemaphoreGive(s_signals_mutex);
  }
}

void Signals::clear()
{
  if (!_started || _strip == nullptr) {
    return;
  }
  if (s_signals_mutex) {
    xSemaphoreTake(s_signals_mutex, portMAX_DELAY);
  }
  _strip->clear();
  if (_autoShow) {
    _strip->show();
  }
  if (s_signals_mutex) {
    xSemaphoreGive(s_signals_mutex);
  }
}

void Signals::setAll(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
  if (!_started || _strip == nullptr) {
    return;
  }
  if (s_signals_mutex) {
    xSemaphoreTake(s_signals_mutex, portMAX_DELAY);
  }
  for (uint16_t i = 0; i < _numLeds; ++i) {
    applyPixel(i, r, g, b, brightness);
  }
  if (_autoShow) {
    _strip->show();
  }
  if (s_signals_mutex) {
    xSemaphoreGive(s_signals_mutex);
  }
}

void Signals::setPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
  if (!_started || _strip == nullptr) {
    return;
  }
  if (s_signals_mutex) {
    xSemaphoreTake(s_signals_mutex, portMAX_DELAY);
  }
  applyPixel(index, r, g, b, brightness);
  if (_autoShow) {
    _strip->show();
  }
  if (s_signals_mutex) {
    xSemaphoreGive(s_signals_mutex);
  }
}

/* -------------------------------- Speaker -------------------------------- */

void Signals::speakerStopHw()
{
  if (_speakerPin < 0) {
    return;
  }
#if SIGNALS_LEDC_V3
  ledcWrite((uint8_t)_speakerPin, 0);
#else
  ledcWrite(kSpeakerLecChannel, 0);
#endif
}

void Signals::speakerStart(uint16_t freq, uint8_t vol)
{
  if (_speakerPin < 0) {
    return;
  }
  vol = clampU8(vol, 50);
  if (vol == 0 || freq == 0) {
    speakerStopHw();
    return;
  }
  if (freq < 20) {
    freq = 20;
  }
  if (freq > 20000) {
    freq = 20000;
  }

  const uint32_t duty = (uint32_t)vol * kSpeakerDutyAtVol50 / 50u;

#if SIGNALS_LEDC_V3
  ledcChangeFrequency((uint8_t)_speakerPin, freq, kSpeakerPwmBits);
  ledcWrite((uint8_t)_speakerPin, duty);
#else
  ledcSetup(kSpeakerLecChannel, freq, kSpeakerPwmBits);
  ledcWrite(kSpeakerLecChannel, duty);
#endif
}

void Signals::stopToneLocked()
{
  cancelToneTimer();
  speakerStopHw();
  _toneActive = false;
  _toneStopMs = 0;
}

void Signals::tone(uint16_t freq, uint8_t vol, uint16_t durationMs)
{
  if (!_started) {
    return;
  }
  vol = clampU8(vol, 50);
  if (s_signals_mutex) {
    xSemaphoreTake(s_signals_mutex, portMAX_DELAY);
  }
  cancelToneTimer();
  speakerStart(freq, vol);
  _toneActive = (vol > 0 && freq > 0);
  if (_toneActive && durationMs > 0) {
    _toneStopMs = millis() + durationMs;
    armToneTimer(durationMs);
  } else {
    _toneStopMs = 0;
  }
  if (s_signals_mutex) {
    xSemaphoreGive(s_signals_mutex);
  }
}

void Signals::stopTone()
{
  if (!_started) {
    return;
  }
  if (s_signals_mutex) {
    xSemaphoreTake(s_signals_mutex, portMAX_DELAY);
  }
  stopToneLocked();
  if (s_signals_mutex) {
    xSemaphoreGive(s_signals_mutex);
  }
}

void Signals::restartTone(uint16_t freq, uint8_t vol, uint16_t durationMs)
{
  /* Ein Aufruf mit Mutex: sonst kann zwischen stop und tone der Ring-show den Ton verzögern. */
  if (!_started) {
    return;
  }
  vol = clampU8(vol, 50);
  if (s_signals_mutex) {
    xSemaphoreTake(s_signals_mutex, portMAX_DELAY);
  }
  cancelToneTimer();
  speakerStopHw();
  speakerStart(freq, vol);
  _toneActive = (vol > 0 && freq > 0);
  if (_toneActive && durationMs > 0) {
    _toneStopMs = millis() + durationMs;
    armToneTimer(durationMs);
  } else {
    _toneStopMs = 0;
  }
  if (s_signals_mutex) {
    xSemaphoreGive(s_signals_mutex);
  }
}
