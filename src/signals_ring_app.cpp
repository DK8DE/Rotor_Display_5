/**
 * Ring: Homing = oranges Lauflicht; referenziert = grün, Ist-Richtung rot (LED 0 Mitte = 0°, Uhrzeigersinn).
 * Punkt (<20° Öffnung): Gauß; breiter Sektor: cos bis half+1,5·LED-Breite, Rand gedimmt.
 * Nicht referenziert, kein Homing = gedimmtes Orange (Warnung).
 */

#include "signals_ring_app.h"

#include <Arduino.h>
#include <math.h>

#include "pwm_config.h"
#include "rotor_app.h"
#include "rotor_error_app.h"
#include "rotor_rs485.h"
#include <Signals.h>

#ifndef SIGNALS_RING_MIN_INTERVAL_MS
#define SIGNALS_RING_MIN_INTERVAL_MS 45u
#endif

#ifndef SIGNALS_RING_POINT_SIGMA_LED
/** Gauß-ähnlicher Punkt: σ in LED-Schritten — Nachbar-LEDs sichtbar gedimmt */
#define SIGNALS_RING_POINT_SIGMA_LED 0.62f
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

/** Kleinster Winkelabstand 0…180° (Wrap bei 0/360) */
static float angle_diff_abs_wrap(float a_deg, float b_deg)
{
    float d = fabsf(a_deg - b_deg);
    if (d > 180.0f) {
        d = 360.0f - d;
    }
    return d;
}

/**
 * Kürzeste Distanz auf dem Ring in LED-Schritten (led_pos 0…n).
 * LED j hat ihre Mittel bei j·(360/n)° bzw. led_pos = j — damit 0° (Nord) mit LED 0 und Arc übereinstimmt;
 * (j+0.5)/n·360° würde alles um eine halbe LED gegenüber dem Arc verschieben.
 */
static float circular_led_dist_abs(float led_pos, int j, int n)
{
    float d = led_pos - (float)j;
    const float nf = (float)n;
    if (d > nf * 0.5f) {
        d -= nf;
    } else if (d < -nf * 0.5f) {
        d += nf;
    }
    return fabsf(d);
}

static void set_pixel_rg(float rw, uint8_t j)
{
    if (rw < 0.0f) {
        rw = 0.0f;
    } else if (rw > 1.0f) {
        rw = 1.0f;
    }
    const float gw = 1.0f - rw;
    const uint8_t r = (uint8_t)lroundf(255.0f * rw);
    const uint8_t g = (uint8_t)lroundf(200.0f * gw);
    const uint8_t b = 0;
    const uint8_t br = (uint8_t)lroundf(90.0f + 15.0f * rw);
    s_sig->setPixel(j, r, g, b, br);
}

/** Kleiner Öffnungswinkel: Gauß um Richtung — eine Haupt-LED, Nachbarn gedimmt, Drehen bleibt weich */
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
    const float sigma = SIGNALS_RING_POINT_SIGMA_LED;
    const float denom = 2.0f * sigma * sigma;
    for (uint8_t j = 0; j < s_n; j++) {
        const float dist = circular_led_dist_abs(w, (int)j, n);
        float rw = expf(-(dist * dist) / denom);
        set_pixel_rg(rw, j);
    }
}

/**
 * Großer Öffnungswinkel: Helligkeit fällt vom Zentrum zum Rand (cos), äußere LEDs gedimmt.
 * Geometrischer Rand ±half; limit = half + 1,5·LED-Breite (½ + 1 LED weicher Auslauf ≈ eine LED mehr sichtbar).
 * Kein „inner = half - feather“ mehr — das schnitt z. B. bei 150° (half=75°) bei 66° ab und ließ nur
 * 3 LED-Mitten (0…45°) hart rot, obwohl 150° ≈ 6,7 LED-Schritte (16 LEDs) sein sollen.
 */
static void set_direction_sector_red_on_green(float center_deg_ui, float opening_deg)
{
    const int n = (int)s_n;
    if (n <= 0) {
        return;
    }
    const float center = normalize_deg_for_led(center_deg_ui);
    const float half = opening_deg * 0.5f;
    const float half_safe = (half > 1e-4f) ? half : 1e-4f;
    const float deg_per_led = 360.0f / (float)n;
    const float limit = half_safe + 1.5f * deg_per_led;
    const float limit_safe = (limit > 1e-4f) ? limit : 1e-4f;
    for (uint8_t j = 0; j < s_n; j++) {
        const float led_center_deg = (float)j * deg_per_led;
        const float d = angle_diff_abs_wrap(center, led_center_deg);
        float rw = 0.0f;
        if (d <= limit_safe) {
            rw = cosf(1.57079633f * (d / limit_safe));
        }
        set_pixel_rg(rw, j);
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
    /* Wie LVGL-Arc: Bus-Ist + Antennenversatz der gewählten Antenne */
    const float pos_deg = rotor_app_get_display_direction_deg();
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
        const uint8_t ant = pwm_config_get_last_antenna();
        const float opening = pwm_config_get_opening_deg((int)ant);
        if (opening >= 20.0f) {
            set_direction_sector_red_on_green(pos_deg, opening);
        } else {
            set_direction_red_on_green(dir_led_pos);
        }
    } else {
        for (uint8_t i = 0; i < s_n; i++) {
            s_sig->setPixel(i, 80, 25, 0, 40);
        }
    }

    s_sig->show();
}
