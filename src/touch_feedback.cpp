/**
 * Kurzer Klick-Pieps über Signals (ATtiny), nur für lv_btn-Hits (Label auf Button zählt).
 * Vor jedem Ton: stopTone + flush (Signals), damit der Empfänger keine halben „T“-Zeilen parsiert
 * (sonst: Dauerton). Kein zweiter Ton beim Loslassen — vermeidet überlappende Befehle.
 * Entprellung nur bei kurzem Abstand *und* fast gleicher Position; schnelle Folge auf andere Buttons bleibt hörbar.
 */

#include "touch_feedback.h"

#include <Arduino.h>

#include <lvgl.h>

#include "pwm_config.h"
#include "Signals.h"

static Signals *s_sig = nullptr;
static uint32_t s_last_beep_ms = 0;
/** Letzte Piep-Position — schnelle Folgetaps auf andere Buttons sollen nicht blockiert werden */
static int16_t s_last_beep_x = 0;
static int16_t s_last_beep_y = 0;
static uint32_t s_last_arc_beep_ms = 0;

static void play_safe_beep(uint16_t freq_hz, uint8_t vol, uint16_t dur_ms)
{
    if (s_sig == nullptr) {
        return;
    }
    s_sig->stopTone();
    s_sig->tone(freq_hz, vol, dur_ms);
}

void touch_feedback_set_signals(Signals *sig)
{
    s_sig = sig;
    s_last_beep_ms = 0;
    s_last_beep_x = 0;
    s_last_beep_y = 0;
    s_last_arc_beep_ms = 0;
}

static bool hit_is_button(lv_obj_t *hit)
{
    for (lv_obj_t *o = hit; o != nullptr; o = lv_obj_get_parent(o)) {
        if (lv_obj_check_type(o, &lv_btn_class)) {
            return true;
        }
    }
    return false;
}

static lv_obj_t *search_obj_at(lv_coord_t x, lv_coord_t y)
{
    lv_point_t pt;
    pt.x = x;
    pt.y = y;
    lv_disp_t *disp = lv_disp_get_default();
    if (disp == nullptr) {
        return nullptr;
    }
    lv_obj_t *hit = lv_indev_search_obj(lv_disp_get_layer_sys(disp), &pt);
    if (hit == nullptr) {
        hit = lv_indev_search_obj(lv_disp_get_layer_top(disp), &pt);
    }
    if (hit == nullptr) {
        hit = lv_indev_search_obj(lv_disp_get_scr_act(disp), &pt);
    }
    return hit;
}

void touch_feedback_touchpad_edge(bool pressed, int16_t x, int16_t y)
{
    if (s_sig == nullptr) {
        return;
    }
    if (!pressed) {
        return;
    }
    const uint32_t now = millis();
    const uint32_t dt = (uint32_t)(now - s_last_beep_ms);
    /*
     * Früher: fester Mindestabstand 45 ms → bei schnellem Wechsel zwischen Buttons oft kein Piep,
     * obwohl LVGL den Klick verarbeitet. Nur unterdrücken: sehr kurzer Abstand *und* fast gleiche
     * Koordinate (Touch-Prellen / doppelte Flanke), nicht bei anderem Ziel.
     */
#ifndef TOUCH_FEEDBACK_DEBOUNCE_MS
#define TOUCH_FEEDBACK_DEBOUNCE_MS 22u
#endif
#ifndef TOUCH_FEEDBACK_SAME_SPOT_PX2
#define TOUCH_FEEDBACK_SAME_SPOT_PX2 (28 * 28)
#endif
    if (s_last_beep_ms != 0u && dt < TOUCH_FEEDBACK_DEBOUNCE_MS) {
        const int dx = (int)x - (int)s_last_beep_x;
        const int dy = (int)y - (int)s_last_beep_y;
        const int d2 = dx * dx + dy * dy;
        if (d2 < (int)TOUCH_FEEDBACK_SAME_SPOT_PX2) {
            return;
        }
    }
    lv_obj_t *hit = search_obj_at(static_cast<lv_coord_t>(x), static_cast<lv_coord_t>(y));
    if (!hit_is_button(hit)) {
        return;
    }
    s_last_beep_ms = now;
    s_last_beep_x = x;
    s_last_beep_y = y;
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
    s_last_beep_ms = now;
    {
        const uint16_t f = pwm_config_get_touch_beep_freq_hz();
        const uint16_t f_arc = (f > 220u) ? static_cast<uint16_t>(f - 180u) : 200u;
        const uint8_t v = pwm_config_get_touch_beep_vol();
        const uint8_t v_arc = (v > 2u) ? static_cast<uint8_t>(v - 2u) : v;
        play_safe_beep(f_arc, v_arc, 30);
    }
}
