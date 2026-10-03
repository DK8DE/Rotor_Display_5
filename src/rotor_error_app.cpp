/**
 * Meldungen zu GETERR / async ERR; UI: meldetext, homing_led, Ring-Override in signals_ring_app.
 * Fehler bleiben bis Neustart, Ausnahme: Fehler 10 (Deadman / lokaler Verbindungstimeout) —
 * per Homing-Taste (SETREF) oder wiederkehrendem Slave-Verkehr / ACK_ERR:0 quittierbar.
 */

#include "rotor_error_app.h"

#include <Arduino.h>

#include "lvgl_v8_port.h"
#include "pwm_config.h"
#include "rotor_rs485.h"
#include "serial_bridge.h"
#include "ui/screens.h"

#include <lvgl.h>

#ifndef ROTOR_ERR_LED_BLINK_MS
#define ROTOR_ERR_LED_BLINK_MS 400u
#endif
/* Sehr kurze lokale Verbindungstimeout-Glitches (10) nicht sofort als roten Ring anzeigen. */
#ifndef ROTOR_ERR10_RING_DELAY_MS
#define ROTOR_ERR10_RING_DELAY_MS 900u
#endif

static int s_err_code = 0;
static uint32_t s_err_set_ms = 0;
static uint32_t s_led_blink_last_ms = 0;
static bool s_led_blink_bright = true;
/** Abwechselnd Fehlertext und Hinweis (UI-String) pro Sekunde */
static uint32_t s_meldetext_last_alternate_sec = UINT32_MAX;
/** true = Code kam vom Rotor (ERR/ACK_ERR), nicht vom lokalen Verbindungs-Watchdog */
static bool s_err_from_rotor = false;

static const char *message_for_code(int code)
{
    switch (code) {
    case 0:
        return "Betriebsbereit";
    case 10:
        /* Rotor-Deadman vs. lokaler Link-Watchdog — gleiche Quittung (SETREF / Homing-Taste). */
        return s_err_from_rotor ? "Deadman Timeout" : "Verbindungstimeout";
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
    /* Fehler 10: kein „Bitte Neustart“ — Quittierung per Homing-Taste (SETREF). */
    if (s_err_code == 10) {
        const uint32_t sec = now_ms / 1000u;
        if (sec == s_meldetext_last_alternate_sec) {
            return;
        }
        s_meldetext_last_alternate_sec = sec;
        if ((sec % 2u) == 0u) {
            lv_textarea_set_text(objects.meldetext, message_for_code(10));
        } else {
            lv_textarea_set_text(objects.meldetext, "Homing-Taste");
        }
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

/** Remote-USB laut Config oder Bridge — Config gewinnt bei Kaltstart. */
static bool remote_usb_mode(void)
{
    if (pwm_config_get_remote_usb() != 0u) {
        if (!serial_bridge::remote_usb_active()) {
            serial_bridge::set_remote_usb(true);
        }
        return true;
    }
    return serial_bridge::remote_usb_active();
}

static void apply_meldetext(void)
{
    if (!objects.meldetext) {
        return;
    }
    /* Remote-USB: nur PC-Präsenz zählt — kein Soft-10/Boot-Text ohne Software. */
    if (remote_usb_mode()) {
        if (!serial_bridge::remote_usb_pc_seen()) {
            if (rotor_rs485_is_homing() && serial_bridge::remote_usb_link_session()) {
                lv_textarea_set_text(objects.meldetext, "Referenziere");
            } else {
                lv_textarea_set_text(objects.meldetext, "Warten auf PC");
            }
            return;
        }
        /* Soft-10 ist im Remote-USB unsichtbar — lokal irrelevant. */
        if (s_err_code == 10 && !s_err_from_rotor) {
            s_err_code = 0;
        }
        if (s_err_code != 0) {
            s_meldetext_last_alternate_sec = UINT32_MAX;
            apply_fault_meldetext_alternate(millis());
            return;
        }
        if (!rotor_rs485_is_boot_done() || !rotor_rs485_is_startup_error_checked()) {
            lv_textarea_set_text(objects.meldetext, "Initialisiere");
            return;
        }
        if (rotor_rs485_is_homing()) {
            lv_textarea_set_text(objects.meldetext, "Referenziere");
            return;
        }
        if (!rotor_rs485_is_referenced()) {
            lv_textarea_set_text(objects.meldetext, "Nicht referenziert");
            return;
        }
        lv_textarea_set_text(objects.meldetext, "Remote USB");
        return;
    }
    if (s_err_code != 0) {
        s_meldetext_last_alternate_sec = UINT32_MAX;
        apply_fault_meldetext_alternate(millis());
        return;
    }
    /* Beim parallelen Start (Display sofort sichtbar) ist der GETREF-Boot ggf. noch nicht durch.
     * Bis Boot-Abschluss keine "Nicht referenziert"-Meldung anzeigen. */
    if (!rotor_rs485_is_boot_done() || !rotor_rs485_is_startup_error_checked()) {
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
#ifndef ROTOR_HOMING_LED_YELLOW
#define ROTOR_HOMING_LED_YELLOW 0xffcc00
#endif

static void apply_homing_led_fault(uint32_t now_ms)
{
    if (!objects.homing_led) {
        return;
    }
    /* Remote-USB ohne Software: immer gelb — kein Soft-10-Blinken. */
    if (remote_usb_mode() && !serial_bridge::remote_usb_pc_seen()) {
        if (!(rotor_rs485_is_homing() && serial_bridge::remote_usb_link_session())) {
            lv_led_set_color(objects.homing_led, lv_color_hex(ROTOR_HOMING_LED_YELLOW));
            lv_led_set_brightness(objects.homing_led, 255);
            return;
        }
    }
    /* Harte Fehler: rot blinken. Fehler 10 (quittierbar): ebenfalls blinken bis SETREF.
     * Soft-10 im Remote-USB: nie. */
    if (s_err_code != 0 &&
        !(remote_usb_mode() && s_err_code == 10 && !s_err_from_rotor)) {
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
        if (remote_usb_mode() && !serial_bridge::remote_usb_pc_seen()) {
            lv_led_set_color(objects.homing_led, lv_color_hex(ROTOR_HOMING_LED_YELLOW));
            lv_led_set_brightness(objects.homing_led, 255);
            return;
        }
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
    s_err_from_rotor = false;
    s_led_blink_last_ms = 0;
    s_led_blink_bright = true;
    s_meldetext_last_alternate_sec = UINT32_MAX;
    /* Config → Bridge nachziehen (Kaltstart Remote-USB). */
    (void)remote_usb_mode();
    lvgl_port_lock(-1);
    apply_meldetext();
    lvgl_port_unlock();
}

void rotor_error_app_set_error_code(int code)
{
    if (code < 0) {
        code = 0;
    }
    /* Soft-10 (lokaler Link-Watchdog) im Remote-USB nie latchen — sonst
     * „Verbindungstimeout / Homing-Taste“ statt „Warten auf PC“ beim Start ohne Software. */
    if (code == 10 && (serial_bridge::remote_usb_active() || pwm_config_get_remote_usb() != 0u)) {
        return;
    }
    /* Latch: Fehler bleibt bis Neustart — Ausnahme Fehler 10 (Deadman / Link) per SETREF quittierbar. */
    if (code == 0) {
        if (s_err_code != 0 && s_err_code != 10) {
            return;
        }
        s_err_from_rotor = false;
    } else if (code == 10) {
        /* Aufruf vom Watchdog / Boot-TEST: lokaler Soft-Timeout (Rotor-10 kommt über report_rotor). */
        s_err_from_rotor = false;
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

void rotor_error_app_report_rotor_err(int code)
{
    if (code < 0) {
        code = 0;
    }
    if (code == 0) {
        /* ACK_ERR:0 / SETREF-Quittung: Fehler 10 löschen; andere Codes bleiben bis Neustart. */
        if (s_err_code != 0 && s_err_code != 10) {
            return;
        }
        s_err_from_rotor = false;
        if (s_err_code != 0) {
            s_err_set_ms = millis();
        }
        s_err_code = 0;
        lvgl_port_lock(-1);
        apply_meldetext();
        lvgl_port_unlock();
        return;
    }
    s_err_from_rotor = true;
    if (s_err_code != code) {
        s_err_set_ms = millis();
    }
    s_err_code = code;
    s_meldetext_last_alternate_sec = UINT32_MAX;
    lvgl_port_lock(-1);
    apply_meldetext();
    lvgl_port_unlock();
}

int rotor_error_app_get_error_code(void)
{
    return s_err_code;
}

bool rotor_error_app_is_rotor_reported(void)
{
    return s_err_code != 0 && s_err_from_rotor;
}

bool rotor_error_app_is_fault_ring_red(void)
{
    /* Remote-USB ohne Software: nie roter Soft-10-Ring. */
    if (remote_usb_mode() && !serial_bridge::remote_usb_pc_seen()) {
        return false;
    }
    if (remote_usb_mode() && s_err_code == 10 && !s_err_from_rotor) {
        return false;
    }
    if (s_err_code == 0) {
        return false;
    }
    if (s_err_code == 10 && !s_err_from_rotor) {
        return (uint32_t)(millis() - s_err_set_ms) >= ROTOR_ERR10_RING_DELAY_MS;
    }
    return true;
}

bool rotor_error_app_is_waiting_for_pc(void)
{
    /* Gelber Wartezustand: Remote-USB und keine aktuellen PC-Protokoll-Frames. */
    return remote_usb_mode() && !serial_bridge::remote_usb_pc_seen() &&
           !(rotor_rs485_is_homing() && serial_bridge::remote_usb_link_session());
}

bool rotor_error_app_is_fault_locked(void)
{
    if (s_err_code == 0) {
        return false;
    }
    /* Fehler 10 (Deadman / Verbindungstimeout): Homing-Taste / SETREF quittiert — nicht sperren. */
    if (s_err_code == 10) {
        return false;
    }
    return true;
}

void rotor_error_app_loop(uint32_t now_ms)
{
    static bool last_homing = false;
    /* Initial true: erzwingt Abgleich, falls Slave nach Boot referenziert meldet */
    static bool last_referenced = true;
    static bool last_startup_checked = false;
    static bool last_wait_pc = false;
    static bool last_pc_seen = false;
    const bool remote = remote_usb_mode();
    const bool homing = rotor_rs485_is_homing();
    const bool referenced = rotor_rs485_is_referenced();
    const bool startup_checked = rotor_rs485_is_startup_error_checked();
    const bool pc_seen = remote && serial_bridge::remote_usb_pc_seen();
    const bool wait_pc = rotor_error_app_is_waiting_for_pc();

    /* Soft-10 im Remote-USB: immer löschen — nur RS485-Modus braucht den Watchdog. */
    if (remote && s_err_code == 10 && !s_err_from_rotor) {
        s_err_code = 0;
        s_err_set_ms = now_ms;
    }

    lvgl_port_lock(-1);
    if (wait_pc != last_wait_pc || pc_seen != last_pc_seen ||
        (s_err_code == 0 &&
         (homing != last_homing || referenced != last_referenced ||
          startup_checked != last_startup_checked))) {
        last_homing = homing;
        last_referenced = referenced;
        last_startup_checked = startup_checked;
        last_wait_pc = wait_pc;
        last_pc_seen = pc_seen;
        apply_meldetext();
    }
    if (s_err_code != 0 && !remote) {
        apply_fault_meldetext_alternate(now_ms);
    } else if (remote && s_err_code != 0 && s_err_from_rotor) {
        apply_fault_meldetext_alternate(now_ms);
    } else if (wait_pc || (remote && !pc_seen)) {
        apply_meldetext();
    }
    apply_homing_led_fault(now_ms);
    lvgl_port_unlock();
}
