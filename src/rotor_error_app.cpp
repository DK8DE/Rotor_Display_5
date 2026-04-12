/**
 * Meldungen zu GETERR / async ERR; UI: meldetext, homing_led, Ring-Override in signals_ring_app.
 * Fehler bleiben bis Neustart, Ausnahme: Verbindungstimeout (10) — wird bei ACK_ERR:0 (GETERR)
 * zurückgesetzt, nicht bei beliebiger Slave-Zeile (ref=1 kann trotz Slave-Fehler sein).
 */

#include "rotor_error_app.h"

#include <Arduino.h>

#include "lvgl_v8_port.h"
#include "rotor_rs485.h"
#include "ui/screens.h"

#include <lvgl.h>

#ifndef ROTOR_ERR_LED_BLINK_MS
#define ROTOR_ERR_LED_BLINK_MS 400u
#endif
/* Sehr kurze Verbindungstimeout-Glitches (10) nicht sofort als roten Ring anzeigen. */
#ifndef ROTOR_ERR10_RING_DELAY_MS
#define ROTOR_ERR10_RING_DELAY_MS 900u
#endif

static int s_err_code = 0;
static uint32_t s_err_set_ms = 0;
static uint32_t s_led_blink_last_ms = 0;
static bool s_led_blink_bright = true;
/** Wechsel Fehlertext / „Bitte Neustart“ pro Sekunde */
static uint32_t s_meldetext_last_alternate_sec = UINT32_MAX;

static const char *message_for_code(int code)
{
    switch (code) {
    case 0:
        return "Betriebsbereit";
    case 10:
        return "Verbindungstimeout";
    case 11:
        return "Endschalter Fehler";
    case 12:
        return "Not-Aus";
    case 15:
        return "\xC3\x9C" "berstrom";
    case 16:
        return "Stall keine Bewegung";
    case 17:
        return "Homing Timeout";
    case 18:
        return "Positio Timeout";
    default:
        return nullptr;
    }
}

static void apply_fault_meldetext_alternate(uint32_t now_ms)
{
    if (!objects.meldetext || s_err_code == 0) {
        return;
    }
    /* Timeout: kein Wechsel mit „Bitte Neustart“ — Verbindung kann ohne Neustart wiederkehren */
    if (s_err_code == 10) {
        lv_textarea_set_text(objects.meldetext, "Verbindungstimeout");
        return;
    }
    const uint32_t sec = now_ms / 1000u;
    if (sec == s_meldetext_last_alternate_sec) {
        return;
    }
    s_meldetext_last_alternate_sec = sec;
    const bool show_err = (sec % 2u) == 0u;
    if (show_err) {
        const char *msg = message_for_code(s_err_code);
        if (!msg) {
            static char buf[40];
            snprintf(buf, sizeof(buf), "Fehler %d", s_err_code);
            lv_textarea_set_text(objects.meldetext, buf);
        } else {
            lv_textarea_set_text(objects.meldetext, msg);
        }
    } else {
        lv_textarea_set_text(objects.meldetext, "Bitte Neustart");
    }
}

static void apply_meldetext(void)
{
    if (!objects.meldetext) {
        return;
    }
    if (s_err_code != 0) {
        s_meldetext_last_alternate_sec = UINT32_MAX;
        apply_fault_meldetext_alternate(millis());
        return;
    }
    if (!rotor_rs485_is_startup_error_checked()) {
        lv_textarea_set_text(objects.meldetext, "Initialisiere");
        return;
    }
    const char *msg = nullptr;
    if (rotor_rs485_is_homing()) {
        msg = "Referenziere";
    } else if (!rotor_rs485_is_referenced()) {
        msg = "Nicht referenziert";
    } else {
        msg = "Betriebsbereit";
    }
    lv_textarea_set_text(objects.meldetext, msg);
}

#ifndef ROTOR_HOMING_LED_GREEN
#define ROTOR_HOMING_LED_GREEN 0x43b302
#endif
#ifndef ROTOR_HOMING_LED_RED
#define ROTOR_HOMING_LED_RED 0xff0000
#endif

static void apply_homing_led_fault(uint32_t now_ms)
{
    if (!objects.homing_led) {
        return;
    }
    /* Fehler: immer rot blinken (Homing/Referenz nicht möglich) */
    if (s_err_code != 0) {
        if ((uint32_t)(now_ms - s_led_blink_last_ms) < ROTOR_ERR_LED_BLINK_MS) {
            return;
        }
        s_led_blink_last_ms = now_ms;
        s_led_blink_bright = !s_led_blink_bright;
        lv_led_set_color(objects.homing_led, lv_color_hex(0xff0000));
        lv_led_set_brightness(objects.homing_led, s_led_blink_bright ? 255 : 80);
        return;
    }
    if (!rotor_rs485_is_startup_error_checked()) {
        lv_led_set_color(objects.homing_led, lv_color_hex(ROTOR_HOMING_LED_RED));
        lv_led_set_brightness(objects.homing_led, 255);
        return;
    }
    if (rotor_rs485_is_homing()) {
        return;
    }
    const bool ref = rotor_rs485_is_referenced();
    lv_led_set_color(objects.homing_led,
                     ref ? lv_color_hex(ROTOR_HOMING_LED_GREEN) : lv_color_hex(ROTOR_HOMING_LED_RED));
    lv_led_set_brightness(objects.homing_led, 255);
}

void rotor_error_app_init(void)
{
    s_err_code = 0;
    s_err_set_ms = 0;
    s_led_blink_last_ms = 0;
    s_led_blink_bright = true;
    s_meldetext_last_alternate_sec = UINT32_MAX;
    lvgl_port_lock(-1);
    apply_meldetext();
    lvgl_port_unlock();
}

void rotor_error_app_set_error_code(int code)
{
    if (code < 0) {
        code = 0;
    }
    /* Latch: Fehler bleibt bis Neustart — Ausnahme Verbindungstimeout (10) per Bus quittierbar */
    if (code == 0 && s_err_code != 0 && s_err_code != 10) {
        return;
    }
    if (s_err_code != code) {
        s_err_set_ms = millis();
    }
    s_err_code = code;
    if (code != 0) {
        s_meldetext_last_alternate_sec = UINT32_MAX;
    }
    lvgl_port_lock(-1);
    apply_meldetext();
    lvgl_port_unlock();
}

int rotor_error_app_get_error_code(void)
{
    return s_err_code;
}

bool rotor_error_app_is_fault_ring_red(void)
{
    if (s_err_code == 0) {
        return false;
    }
    if (s_err_code == 10) {
        return (uint32_t)(millis() - s_err_set_ms) >= ROTOR_ERR10_RING_DELAY_MS;
    }
    return true;
}

bool rotor_error_app_is_fault_locked(void)
{
    return s_err_code != 0 && s_err_code != 10;
}

void rotor_error_app_loop(uint32_t now_ms)
{
    static bool last_homing = false;
    /* Initial true: erzwingt Abgleich, falls Slave nach Boot referenziert meldet */
    static bool last_referenced = true;
    static bool last_startup_checked = false;
    const bool homing = rotor_rs485_is_homing();
    const bool referenced = rotor_rs485_is_referenced();
    const bool startup_checked = rotor_rs485_is_startup_error_checked();

    lvgl_port_lock(-1);
    if (s_err_code == 0 &&
        (homing != last_homing || referenced != last_referenced || startup_checked != last_startup_checked)) {
        last_homing = homing;
        last_referenced = referenced;
        last_startup_checked = startup_checked;
        apply_meldetext();
    }
    if (s_err_code != 0) {
        apply_fault_meldetext_alternate(now_ms);
    }
    apply_homing_led_fault(now_ms);
    lvgl_port_unlock();
}
