/**
 * Button-Pieps über Signals (ATtiny): Aufruf aus den konkreten UI-Aktionshandlern (CLICKED).
 * Vor jedem Ton: stopTone (Signals), damit der Empfänger keine halben „T“-Zeilen parsiert.
 */

#include "touch_feedback.h"

#include <Arduino.h>

#include "pwm_config.h"
#include "signals_ring_app.h"
#include "Signals.h"

static Signals *s_sig = nullptr;
static uint32_t s_last_arc_beep_ms = 0;

static void play_safe_beep(uint16_t freq_hz, uint8_t vol, uint16_t dur_ms)
{
    if (s_sig == nullptr) {
        return;
    }
    // Ring-Updates kurz pausieren, damit der Beep-Befehl auf der Shared-UART nicht untergeht.
    signals_ring_app_pause_updates_for((uint16_t)(dur_ms + 45u));
    s_sig->restartTone(freq_hz, vol, dur_ms);
    // Redundanz gegen sporadisch verlorenes Einzel-Telegramm bei kurzen Taps.
    delayMicroseconds(1200);
    s_sig->tone(freq_hz, vol, dur_ms);
}

void touch_feedback_set_signals(Signals *sig)
{
    s_sig = sig;
    s_last_arc_beep_ms = 0;
}

void touch_feedback_button_click(void)
{
    if (s_sig == nullptr) {
        return;
    }
    play_safe_beep(pwm_config_get_touch_beep_freq_hz(), pwm_config_get_touch_beep_vol(), 32);
}

void touch_feedback_arc_release(void)
{
    if (s_sig == nullptr) {
        return;
    }
    const uint32_t now = millis();
    if ((uint32_t)(now - s_last_arc_beep_ms) < 40u) {
        return;
    }
    s_last_arc_beep_ms = now;
    /* Gleicher Klang wie Button-Pieps (früher: niedrigere Frequenz + leisere Stufe — wirkte „dunkel“/fremd). */
    play_safe_beep(pwm_config_get_touch_beep_freq_hz(), pwm_config_get_touch_beep_vol(), 32);
}
