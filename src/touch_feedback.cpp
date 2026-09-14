/**
 * Button-Pieps über Signals (PWM GPIO40): Aufruf aus den konkreten UI-Aktionshandlern (CLICKED).
 * Lautstärke/Frequenz kommen aus pwm_config (SETLSL / SETCONFRQ am Bus).
 */

#include "touch_feedback.h"

#include <Arduino.h>

#include "pwm_config.h"
#include "Signals.h"

static Signals *s_sig = nullptr;
static uint32_t s_last_arc_beep_ms = 0;

static void play_safe_beep(uint16_t freq_hz, uint8_t vol, uint16_t dur_ms)
{
    if (s_sig == nullptr) {
        return;
    }
    s_sig->restartTone(freq_hz, vol, dur_ms);
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
    /* Gleicher Klang wie Button-Pieps; vol/freq aus pwm_config (Bus SETLSL/SETCONFRQ). */
    play_safe_beep(pwm_config_get_touch_beep_freq_hz(), pwm_config_get_touch_beep_vol(), 32);
}
