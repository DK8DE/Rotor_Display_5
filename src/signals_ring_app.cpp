/**
 * Ring: Homing = oranges Lauflicht; referenziert = grün, Ist-Richtung rot (LED 0 = Nord, Uhrzeigersinn).
 * Nicht referenziert, kein Homing = gedimmtes Orange (Warnung).
 */

#include "signals_ring_app.h"

#include <Arduino.h>
#include <math.h>

#include "rotor_error_app.h"
#include "rotor_rs485.h"
#include <Signals.h>

#ifndef SIGNALS_RING_MIN_INTERVAL_MS
#define SIGNALS_RING_MIN_INTERVAL_MS 45u
#endif

static Signals *s_sig = nullptr;
static uint8_t s_n = 16;
static uint32_t s_last_draw_ms = 0;

static float normalize_deg_for_led(float deg)
{
    float x = fmodf(deg, 360.0f);
    if (x < 0.0f) {
        x += 360.0f;
    }
    if (x >= 359.5f) {
        x = 0.0f;
    }
    return x;
}

/** Richtung als kontinuierliche LED-Position 0..n (für weichen Übergang zwischen benachbarten LEDs) */
static float deg_to_led_position(float deg_ui)
{
    if (s_n == 0) {
        return 0.0f;
    }
    const float x = normalize_deg_for_led(deg_ui);
    return (x / 360.0f) * (float)s_n;
}

/** Zwei benachbarte LEDs mit Anteil (1-frac) bzw. frac — weniger Ruckeln als harter Sprung */
static void set_direction_red_on_green(float led_pos)
{
    const int n = (int)s_n;
    if (n <= 0) {
        return;
    }
    float w = fmodf(led_pos, (float)n);
    if (w < 0.0f) {
        w += (float)n;
    }
    const int i0 = (int)floorf(w);
    const float frac = w - floorf(w);
    const int i1 = (i0 + 1) % n;

    for (uint8_t j = 0; j < s_n; j++) {
        float rw = 0.0f;
        if ((int)j == i0) {
            rw += 1.0f - frac;
        }
        if ((int)j == i1) {
            rw += frac;
        }
        if (rw > 1.0f) {
            rw = 1.0f;
        }
        const float gw = 1.0f - rw;
        const uint8_t r = (uint8_t)lroundf(255.0f * rw);
        const uint8_t g = (uint8_t)lroundf(200.0f * gw);
        const uint8_t b = 0;
        const uint8_t br = (uint8_t)lroundf(90.0f + 15.0f * rw);
        s_sig->setPixel(j, r, g, b, br);
    }
}

void signals_ring_app_init(Signals *signals, uint8_t num_leds)
{
    s_sig = signals;
    s_n = num_leds ? num_leds : 16;
    s_last_draw_ms = 0;
    if (s_sig) {
        s_sig->setAutoShow(false);
    }
}

void signals_ring_app_loop(uint32_t now_ms)
{
    if (!s_sig || s_n == 0) {
        return;
    }
    if ((uint32_t)(now_ms - s_last_draw_ms) < SIGNALS_RING_MIN_INTERVAL_MS) {
        return;
    }
    s_last_draw_ms = now_ms;

    if (rotor_error_app_is_fault_ring_red()) {
        for (uint8_t i = 0; i < s_n; i++) {
            s_sig->setPixel(i, 255, 0, 0, 100);
        }
        s_sig->show();
        return;
    }

    const bool ref = rotor_rs485_is_referenced();
    const bool homing = rotor_rs485_is_homing();
    const float pos_deg = rotor_rs485_get_last_position_deg();
    const float dir_led_pos = deg_to_led_position(pos_deg);

    if (homing) {
        const uint32_t phase = (now_ms / 70u) % (uint32_t)s_n;
        for (uint8_t i = 0; i < s_n; i++) {
            const bool on = (i == (uint8_t)phase) || (i == (uint8_t)((phase + 1u) % s_n));
            if (on) {
                s_sig->setPixel(i, 255, 80, 0, 100);
            } else {
                s_sig->setPixel(i, 20, 5, 0, 25);
            }
        }
    } else if (ref) {
        set_direction_red_on_green(dir_led_pos);
    } else {
        for (uint8_t i = 0; i < s_n; i++) {
            s_sig->setPixel(i, 80, 25, 0, 40);
        }
    }

    s_sig->show();
}
