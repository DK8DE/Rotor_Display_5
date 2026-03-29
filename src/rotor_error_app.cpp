/**
 * Meldungen zu GETERR / async ERR; UI: meldetext, homing_led, Ring-Override in signals_ring_app.
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

static int s_err_code = 0;
static uint32_t s_led_blink_last_ms = 0;
static bool s_led_blink_bright = true;

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

static void apply_meldetext(void)
{
    if (!objects.meldetext) {
        return;
    }
    const char *msg = nullptr;
    if (rotor_rs485_is_homing()) {
        msg = "Referenziere";
    } else if (s_err_code != 0) {
        msg = message_for_code(s_err_code);
        if (!msg) {
            static char buf[40];
            snprintf(buf, sizeof(buf), "Fehler %d", s_err_code);
            lv_textarea_set_text(objects.meldetext, buf);
            return;
        }
    } else {
        msg = "Betriebsbereit";
    }
    lv_textarea_set_text(objects.meldetext, msg);
}

static void apply_homing_led_fault(uint32_t now_ms)
{
    if (!objects.homing_led) {
        return;
    }
    /* Referenzfahrt: normale Homing-Farbe (rot = nicht ref.) übernimmt on_ref_status */
    if (rotor_rs485_is_homing()) {
        return;
    }
    if (s_err_code == 0) {
        return;
    }
    if ((uint32_t)(now_ms - s_led_blink_last_ms) < ROTOR_ERR_LED_BLINK_MS) {
        return;
    }
    s_led_blink_last_ms = now_ms;
    s_led_blink_bright = !s_led_blink_bright;
    lv_led_set_color(objects.homing_led, lv_color_hex(0xff0000));
    /* lv_led: typisch 80 = dunkel, 255 = hell */
    lv_led_set_brightness(objects.homing_led, s_led_blink_bright ? 255 : 80);
}

void rotor_error_app_init(void)
{
    s_err_code = 0;
    s_led_blink_last_ms = 0;
    s_led_blink_bright = true;
    lvgl_port_lock(-1);
    apply_meldetext();
    lvgl_port_unlock();
}

void rotor_error_app_set_error_code(int code)
{
    if (code < 0) {
        code = 0;
    }
    s_err_code = code;
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
    if (rotor_rs485_is_homing()) {
        return false;
    }
    return true;
}

bool rotor_error_app_encoder_click_triggers_homing_only(void)
{
    /* Solange ein Fehlercode anliegt: Taster soll SETREF auslösen (auch wenn noch „referenziert“ angezeigt wird). */
    return s_err_code != 0;
}

void rotor_error_app_loop(uint32_t now_ms)
{
    static bool last_homing = false;
    const bool homing = rotor_rs485_is_homing();

    lvgl_port_lock(-1);
    if (homing != last_homing) {
        last_homing = homing;
        apply_meldetext();
    }
    apply_homing_led_fault(now_ms);
    lvgl_port_unlock();
}
