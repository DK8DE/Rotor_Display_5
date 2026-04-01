/**
 * Verbindet EEZ-UI (ref, homing_led, grad_acc, taget_dg, actual_dg) mit rotor_rs485 — ohne Änderungen in src/ui.
 * Arc 0..360° (1°); Encoder-Soll in Zehntelgrad (Skalierung siehe main ENCODER_DELTA_TENTHS_PER_STEP).
 */

#include "rotor_app.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <lvgl.h>

#include <Arduino.h>

#include "lvgl_v8_port.h"
#include "pwm_config.h"
#include "rotor_error_app.h"
#include "rotor_rs485.h"
#include "ui/screens.h"

extern "C" void rotor_app_antenna_offset_changed(void);

/** EEZ-Export: lv_img_set_angle am Pfeil; nach Init auf 0, Drehung nur per Style-Transform (siehe rotor_app_init). */
static int16_t s_pfeil_wind_eez_base_angle01 = 0;

static bool s_arc_dragging = false;
/** Zwischen PRESSED und RELEASED: mindestens ein VALUE_CHANGED (echter Dreh am Arc)? */
static bool s_arc_moved_this_press = false;
static bool s_arc_updating = false;
/** Encoder: Soll einstellen, Ist-Nachführung am Arc aus */
static bool s_encoder_adjusting = false;
/** true: letzter encoder_apply_goto ist fehlgeschlagen — rotor_app_loop soll erneut senden */
static bool s_encoder_goto_retry_pending = false;
/** Nächster Retry-Zeitpunkt (millis), sobald s_encoder_goto_retry_pending */
static uint32_t s_encoder_retry_deadline_ms = 0;
/** Nach letztem Encoder-Tick: erst wenn millis() >= dieser Zeit → SETPOSDG (kein Senden während Drehens) */
static uint32_t s_encoder_idle_deadline_ms = 0;
static float s_encoder_target_deg = 0.0f;
/** Encoder: Zehntelgrad 0..3599 (±1 pro Klick = ±0,1°); Arc 0..360 = 1°-Schritte */
static int s_encoder_tenths = 0;
static constexpr uint32_t ENCODER_BUS_RETRY_MS = 50;
static constexpr uint32_t ENCODER_SEND_IDLE_MS = 500;
/** false (Test): Encoder ändert nur Soll-Text + Bus, Arc bleibt; true: Arc folgt dem Encoder wie bisher */
static constexpr bool ENCODER_MOVES_ARC = false;

/** Slow/Fast: Bus belegt → PWM erneut in rotor_pwm_ui_loop */
static uint8_t s_pwm_deferred = 255;
/** Nach Start: einmal Fast-PWM aus config (SETPWM) */
static bool s_pwm_boot_send_pending = true;

static void pwm_style_slow_fast(bool fast_active)
{
    const lv_color_t c_on = lv_color_hex(0x087321);
    const lv_color_t c_off = lv_color_hex(0x2196f3);
    if (objects.slow) {
        lv_obj_set_style_bg_color(objects.slow, fast_active ? c_off : c_on, LV_PART_MAIN);
    }
    if (objects.fast) {
        lv_obj_set_style_bg_color(objects.fast, fast_active ? c_on : c_off, LV_PART_MAIN);
    }
}

static void on_slow_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    pwm_style_slow_fast(false);
    const uint8_t p = pwm_config_get_slow();
    if (!rotor_rs485_send_set_pwm_limit(p)) {
        s_pwm_deferred = p;
    }
}

static void on_fast_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    pwm_style_slow_fast(true);
    const uint8_t p = pwm_config_get_fast();
    if (!rotor_rs485_send_set_pwm_limit(p)) {
        s_pwm_deferred = p;
    }
}

static void antenna_apply_style(uint8_t active_1_to_3)
{
    const lv_color_t c_on = lv_color_hex(0x087321);
    const lv_color_t c_off = lv_color_hex(0x2196f3);
    lv_obj_t *btns[3] = { objects.antenna_1, objects.antenna_2, objects.antenna_3 };
    for (uint8_t i = 0; i < 3; i++) {
        if (!btns[i]) {
            continue;
        }
        const bool on = (active_1_to_3 == (uint8_t)(i + 1));
        lv_obj_set_style_bg_color(btns[i], on ? c_on : c_off, LV_PART_MAIN);
    }
}

extern "C" void rotor_app_apply_remote_antenna_selection(uint8_t n_1_to_3)
{
    if (n_1_to_3 < 1u || n_1_to_3 > 3u) {
        return;
    }
    pwm_config_set_last_antenna(n_1_to_3);
    pwm_config_save();
    lvgl_port_lock(-1);
    antenna_apply_style(n_1_to_3);
    rotor_app_antenna_offset_changed();
    lvgl_port_unlock();
}

static void on_antenna_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    lv_obj_t *btn = lv_event_get_target(e);
    uint8_t n = 1;
    if (btn == objects.antenna_2) {
        n = 2;
    } else if (btn == objects.antenna_3) {
        n = 3;
    }
    pwm_config_set_last_antenna(n);
    pwm_config_save();
    antenna_apply_style(n);
    rotor_app_antenna_offset_changed();
    rotor_rs485_send_setaselect(n);
}

static void antenna_apply_labels_from_config(void)
{
    if (objects.antenna_1_label) {
        lv_label_set_text(objects.antenna_1_label, pwm_config_get_antenna_label(1));
    }
    if (objects.antenna_2_label) {
        lv_label_set_text(objects.antenna_2_label, pwm_config_get_antenna_label(2));
    }
    if (objects.antenna_3_label) {
        lv_label_set_text(objects.antenna_3_label, pwm_config_get_antenna_label(3));
    }
}

static int wrap_tenths_deg(int t)
{
    t %= 3600;
    if (t < 0) {
        t += 3600;
    }
    return t;
}

/** Soll aus taget_dg (z. B. „273,7°“) — gleiche Trunc-Logik wie fmt_de */
static bool parse_taget_text_to_tenths(int *out_tenths)
{
    if (!objects.taget_dg || !out_tenths) {
        return false;
    }
    const char *s = lv_textarea_get_text(objects.taget_dg);
    if (!s || !*s) {
        return false;
    }
    char buf[32];
    size_t n = 0;
    while (s[n] && n < sizeof(buf) - 1U) {
        buf[n] = (s[n] == ',') ? '.' : s[n];
        ++n;
    }
    buf[n] = '\0';
    char *end = nullptr;
    const float deg = std::strtof(buf, &end);
    if (end == buf) {
        return false;
    }
    /* Epsilon: float (z. B. 255,4°) liegt oft knapp unter n*0,1 → ohne eps: trunc → 255,3 */
    const int t = static_cast<int>(
        std::floor(static_cast<double>(deg) * 10.0 + 1e-4));
    *out_tenths = wrap_tenths_deg(t);
    return true;
}

/** Arc-Wert 0..360: ganze Grade; ≥359,5° (Homing 360°) → 360 */
static int deg_to_arc_value(float deg)
{
    if (deg >= 359.5f) {
        return 360;
    }
    int v = static_cast<int>(deg + 0.5f);
    if (v >= 360) {
        v = 0;
    }
    if (v < 0) {
        v = 0;
    }
    return v;
}

/** Zuletzt Ist vom Bus (mechanisch, vor Antennenversatz) — für Umrechnung bei Antennenwechsel */
static float s_last_bus_ist_deg = 0.0f;

static float norm360_add(float a)
{
    float x = fmodf(a, 360.0f);
    if (x < 0.0f) {
        x += 360.0f;
    }
    return x;
}

static float active_antenna_offset_deg(void)
{
    return pwm_config_get_antoff_deg(static_cast<int>(pwm_config_get_last_antenna()));
}

/** Anzeige (Kompass / Antennenrichtung) = Buslage + gewählter Antennenversatz */
static float bus_to_display(float bus_deg_ui)
{
    const float off = active_antenna_offset_deg();
    if (bus_deg_ui >= 359.5f) {
        if (std::fabs(static_cast<double>(off)) < 1e-6) {
            return 360.0f;
        }
        return norm360_add(360.0f + off);
    }
    return norm360_add(bus_deg_ui + off);
}

/** Soll am Bus für SETPOSDG aus Anzeige-Winkel */
static float display_to_bus(float display_deg)
{
    const float off = active_antenna_offset_deg();
    if (display_deg >= 359.5f) {
        if (std::fabs(static_cast<double>(off)) < 1e-6f) {
            return 360.0f;
        }
        return norm360_add(360.0f - off);
    }
    return norm360_add(display_deg - off);
}

/**
 * Eine Nachkommastelle: abschneiden auf 0,1° (wie Bus 273,77 → 273,7).
 * Kleines Epsilon vor floor: Binärfloat (255,4) ist oft 255,3999… — ohne eps wird fälschlich 255,3 angezeigt.
 */
static void fmt_de(char *buf, size_t n, float deg)
{
    const double t =
        std::floor(static_cast<double>(deg) * 10.0 + 1e-4) / 10.0;
    snprintf(buf, n, "%.1f", t);
    for (char *p = buf; *p; ++p) {
        if (*p == '.') {
            *p = ',';
        }
    }
}

/** RS485-Pfad darf nicht direkt lvgl_port_lock + Label setzen (WDT/Deadlock mit LVGL-Task). */
static void on_ref_status(bool referenced)
{
    lvgl_port_lock(-1);
    if (objects.homing_led) {
        lv_led_set_color(objects.homing_led,
                         referenced ? lv_color_hex(0x43b302) : lv_color_hex(0xff0000));
    }
    /* Ohne Referenz kein gültiger Ist-Winkel vom Slave — alte Anzeige verwirrt nach Rotor-Neustart */
    if (!referenced && objects.actual_dg) {
        lv_textarea_set_text(objects.actual_dg, "—");
    }
    lvgl_port_unlock();
}

/** Soll aus Bus (Echo, Ankunft, PC-Loopback) — taget_dg in Anzeige-Koordinaten (mit Versatz) */
static void on_target_deg(float bus_deg)
{
    /* Während Encoder aktiv ist: Bus darf taget_dg NICHT überschreiben.
     * Sonst überschreibt z. B. „Ankunft am alten SETPOS“ (notify_target(s_goto_commanded_deg))
     * einen bereits weiter gedrehten Soll — wirkt wie „1. Klick fehlt“ / falsches Ziel. */
    if (s_encoder_adjusting) {
        return;
    }
    lvgl_port_lock(-1);
    char buf[16];
    const float disp = bus_to_display(bus_deg);
    fmt_de(buf, sizeof(buf), disp);
    if (objects.taget_dg) {
        lv_textarea_set_text(objects.taget_dg, buf);
    }
    lvgl_port_unlock();
}

static void on_position_deg(float bus_deg_ui)
{
    s_last_bus_ist_deg = bus_deg_ui;
    lvgl_port_lock(-1);
    char buf[16];
    const float disp = bus_to_display(bus_deg_ui);
    fmt_de(buf, sizeof(buf), disp);
    if (objects.actual_dg) {
        lv_textarea_set_text(objects.actual_dg, buf);
    }
    if (!s_arc_dragging && !s_encoder_adjusting && objects.grad_acc) {
        s_arc_updating = true;
        lv_arc_set_value(objects.grad_acc, deg_to_arc_value(disp));
        s_arc_updating = false;
    }
    lvgl_port_unlock();
}

static void on_ref_button(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    rotor_rs485_send_setref_homing();
}

static void on_arc(lv_event_t *e)
{
    lv_event_code_t c = lv_event_get_code(e);
    lv_obj_t *arc = lv_event_get_target(e);

    if (c == LV_EVENT_PRESSED) {
        s_arc_dragging = true;
        s_arc_moved_this_press = false;
        /* Encoder-Session hier NICHT abbrechen: kurzer Touch ohne Drehen soll keine Klicks „verschlucken“
         * und kein ausstehendes SETPOSDG verwerfen — erst bei echtem Drag (VALUE_CHANGED). */
    }
    if (c == LV_EVENT_VALUE_CHANGED) {
        if (s_arc_updating) {
            return;
        }
        /* Nur während Nutzer-Drag: Soll in taget_dg; sonst würde Ist-Nachführung das Ziel überschreiben */
        if (!s_arc_dragging) {
            return;
        }
        /* Erst echte Arc-Bewegung: Encoder-Modus beenden (sonst Konflikt Soll / Arc / Bus). */
        if (s_encoder_adjusting) {
            s_encoder_adjusting = false;
            s_encoder_goto_retry_pending = false;
            s_encoder_retry_deadline_ms = 0;
            s_encoder_idle_deadline_ms = 0;
        }
        s_arc_moved_this_press = true;
        int v = lv_arc_get_value(arc);
        float deg = static_cast<float>(v);
        char buf[16];
        fmt_de(buf, sizeof(buf), deg);
        if (objects.taget_dg) {
            lv_textarea_set_text(objects.taget_dg, buf);
        }
    }
    if (c == LV_EVENT_RELEASED) {
        s_arc_dragging = false;
        if (s_arc_updating) {
            return;
        }
        /* Nur nach echtem Drehen: GOTO — sonst (Finger kurz auf Arc) Encoder/Bus nicht mit Arc-Wert überschreiben. */
        if (!s_arc_moved_this_press) {
            return;
        }
        int target_v = lv_arc_get_value(arc);
        float target_deg = static_cast<float>(target_v);
        char buf[16];
        fmt_de(buf, sizeof(buf), target_deg);
        if (objects.taget_dg) {
            lv_textarea_set_text(objects.taget_dg, buf);
        }
        /* Arc zeigt Ist (mitfahren), nicht auf dem Ziel stehen bleiben */
        if (objects.grad_acc) {
            s_arc_updating = true;
            lv_arc_set_value(objects.grad_acc,
                             deg_to_arc_value(bus_to_display(s_last_bus_ist_deg)));
            s_arc_updating = false;
        }
        (void)rotor_rs485_goto_degrees(display_to_bus(target_deg));
    }
}

extern "C" void rotor_app_init(void)
{
    rotor_rs485_set_master_id(pwm_config_get_master_id());
    rotor_rs485_set_slave_id(pwm_config_get_rotor_id());
    rotor_rs485_set_ref_callback(on_ref_status);
    rotor_rs485_set_position_callback(on_position_deg);
    rotor_rs485_set_target_callback(on_target_deg);
    rotor_rs485_init();
    rotor_error_app_init();
    if (objects.homing_led) {
        lv_led_set_color(objects.homing_led, lv_color_hex(0xff0000));
    }
    if (objects.ref) {
        lv_obj_add_event_cb(objects.ref, on_ref_button, LV_EVENT_CLICKED, nullptr);
    }
    if (objects.grad_acc) {
        lv_obj_add_event_cb(objects.grad_acc, on_arc, LV_EVENT_ALL, nullptr);
    }
    pwm_style_slow_fast(true);
    if (objects.slow) {
        lv_obj_add_event_cb(objects.slow, on_slow_btn, LV_EVENT_CLICKED, nullptr);
    }
    if (objects.fast) {
        lv_obj_add_event_cb(objects.fast, on_fast_btn, LV_EVENT_CLICKED, nullptr);
    }
    antenna_apply_labels_from_config();
    antenna_apply_style(pwm_config_get_last_antenna());
    if (objects.antenna_1) {
        lv_obj_add_event_cb(objects.antenna_1, on_antenna_btn, LV_EVENT_CLICKED, nullptr);
    }
    if (objects.antenna_2) {
        lv_obj_add_event_cb(objects.antenna_2, on_antenna_btn, LV_EVENT_CLICKED, nullptr);
    }
    if (objects.antenna_3) {
        lv_obj_add_event_cb(objects.antenna_3, on_antenna_btn, LV_EVENT_CLICKED, nullptr);
    }
    /* Windpfeil: EEZ (screens.c) unverändert lassen — REAL+lv_img_set_angle clippt falsch (1px-Strich).
     * Laufzeit: VIRTUAL erzwingen, EEZ-Winkel lesen, lv_img-Winkel aus, Drehung per Objekt-Style (Pivot Mitte). */
    if (objects.pfeil_wind) {
        s_pfeil_wind_eez_base_angle01 = static_cast<int16_t>(lv_img_get_angle(objects.pfeil_wind));
        lv_img_set_size_mode(objects.pfeil_wind, LV_IMG_SIZE_MODE_VIRTUAL);
        lv_img_set_angle(objects.pfeil_wind, 0);
        lv_obj_update_layout(objects.pfeil_wind);
        lv_obj_set_style_transform_pivot_x(objects.pfeil_wind, lv_pct(50), 0);
        lv_obj_set_style_transform_pivot_y(objects.pfeil_wind, lv_pct(50), 0);
        lv_obj_set_style_transform_angle(objects.pfeil_wind, s_pfeil_wind_eez_base_angle01, 0);
    }
    /* GETREF/GETPOSDG nach 2 s Bus-Bereitschaft — siehe rotor_rs485_init / rotor_rs485_loop */
}

/** @return true wenn SETPOSDG gestartet */
static bool encoder_apply_goto(float target_deg)
{
    lvgl_port_lock(-1);
    if (ENCODER_MOVES_ARC && objects.grad_acc) {
        s_arc_updating = true;
        lv_arc_set_value(objects.grad_acc,
                         deg_to_arc_value(bus_to_display(s_last_bus_ist_deg)));
        s_arc_updating = false;
    }
    lvgl_port_unlock();
    return rotor_rs485_goto_degrees(display_to_bus(target_deg));
}

extern "C" void rotor_app_encoder_step(int delta_tenths)
{
    if (!objects.grad_acc || delta_tenths == 0) {
        return;
    }

    lvgl_port_lock(-1);

    /* Session-Start: Zehntel aus taget_dg (Soll), sonst Arc (nur ganze Grade) */
    if (!s_encoder_adjusting) {
        int parsed = 0;
        if (parse_taget_text_to_tenths(&parsed)) {
            s_encoder_tenths = parsed;
        } else {
            int av = lv_arc_get_value(objects.grad_acc);
            if (av >= 360) {
                av = 0;
            }
            s_encoder_tenths = wrap_tenths_deg(av * 10);
        }
    }
    s_encoder_adjusting = true;

    s_encoder_tenths = wrap_tenths_deg(s_encoder_tenths + delta_tenths);

    const float deg = s_encoder_tenths / 10.0f;
    if (ENCODER_MOVES_ARC) {
        int v_arc = (s_encoder_tenths + 5) / 10;
        if (v_arc > 360) {
            v_arc = 360;
        }
        s_arc_updating = true;
        lv_arc_set_value(objects.grad_acc, v_arc);
        s_arc_updating = false;
    }

    char buf[16];
    fmt_de(buf, sizeof(buf), deg);
    if (objects.taget_dg) {
        lv_textarea_set_text(objects.taget_dg, buf);
    }

    lvgl_port_unlock();

    s_encoder_target_deg = deg;

    /* Kein Telegramm während Drehen: Pause neu starten; ausstehenden Bus-Retry verwirft neuer Takt */
    s_encoder_goto_retry_pending = false;
    s_encoder_retry_deadline_ms = 0;
    s_encoder_idle_deadline_ms = millis() + ENCODER_SEND_IDLE_MS;
}

extern "C" void rotor_app_antenna_offset_changed(void)
{
    on_position_deg(s_last_bus_ist_deg);
}

/**
 * Wind/Temperatur/Windrichtung: Werte im Parser; UI hier in loop() (nicht im Parser / kein lv_async).
 * Windrichtung: Drehung per lv_obj_set_style_transform_angle (Pivot 50%), nicht lv_img_set_angle;
 * Basis aus EEZ-Snapshot s_pfeil_wind_eez_base_angle01 + Meteor (0,1°).
 */
extern "C" void rotor_app_weather_ui_poll(void)
{
    const uint8_t m = rotor_rs485_weather_ui_take_mask();
    if (m == 0) {
        return;
    }
    const float w = rotor_rs485_get_last_wind_kmh();
    const float t = rotor_rs485_get_last_tempa_c();
    const float dir = rotor_rs485_get_last_wind_dir_deg();
    lvgl_port_lock(-1);
    char buf[24];
    /* EEZ: wind_speed / temperature sind lv_textarea (Wind max. 5 Zeichen, z. B. „123,4“), keine lv_label */
    if (m & 1u) {
        float wd = w;
        if (wd > 999.9f) {
            wd = 999.9f;
        }
        fmt_de(buf, sizeof(buf), wd);
        if (objects.wind_speed) {
            lv_textarea_set_text(objects.wind_speed, buf);
        }
    }
    if (m & 2u) {
        fmt_de(buf, sizeof(buf), t);
        if (objects.temperature) {
            lv_textarea_set_text(objects.temperature, buf);
        }
    }
    if (m & 4u) {
        if (objects.pfeil_wind) {
            int32_t ang = static_cast<int32_t>(s_pfeil_wind_eez_base_angle01)
                + static_cast<int32_t>(dir * 10.0f + 0.5f);
            while (ang >= 3600) {
                ang -= 3600;
            }
            while (ang < 0) {
                ang += 3600;
            }
            lv_obj_set_style_transform_angle(objects.pfeil_wind, static_cast<lv_coord_t>(ang), 0);
        }
    }
    lvgl_port_unlock();
}

extern "C" void rotor_pwm_ui_loop(void)
{
    /* Erst nach Boot-GETREF/GETPOS: sonst blockiert SETPWM den ersten GETREF. */
    if (!rotor_rs485_is_boot_done()) {
        return;
    }
    if (s_pwm_boot_send_pending) {
        if (rotor_rs485_send_set_pwm_limit(pwm_config_get_fast())) {
            s_pwm_boot_send_pending = false;
        }
        return;
    }
    if (s_pwm_deferred <= 100u) {
        if (rotor_rs485_send_set_pwm_limit(s_pwm_deferred)) {
            s_pwm_deferred = 255;
        }
    }
}

extern "C" void rotor_app_loop(void)
{
    if (!s_encoder_adjusting) {
        return;
    }

    /* Nach Ruhepause fehlgeschlagen → erneut versuchen */
    if (s_encoder_goto_retry_pending) {
        if ((int32_t)(millis() - s_encoder_retry_deadline_ms) < 0) {
            return;
        }
        if (!encoder_apply_goto(s_encoder_target_deg)) {
            s_encoder_retry_deadline_ms = millis() + ENCODER_BUS_RETRY_MS;
            return;
        }
        s_encoder_goto_retry_pending = false;
        s_encoder_adjusting = false;
        s_encoder_retry_deadline_ms = 0;
        s_encoder_idle_deadline_ms = 0;
        return;
    }

    if (s_encoder_idle_deadline_ms == 0) {
        return;
    }
    if ((int32_t)(millis() - s_encoder_idle_deadline_ms) < 0) {
        return;
    }

    s_encoder_idle_deadline_ms = 0;
    if (!encoder_apply_goto(s_encoder_target_deg)) {
        s_encoder_goto_retry_pending = true;
        s_encoder_retry_deadline_ms = millis();
        return;
    }
    s_encoder_adjusting = false;
    s_encoder_retry_deadline_ms = 0;
}

extern "C" void rotor_app_snap_target_to_deg(float bus_deg)
{
    s_encoder_adjusting = false;
    s_encoder_goto_retry_pending = false;
    s_encoder_retry_deadline_ms = 0;
    s_encoder_idle_deadline_ms = 0;
    const float disp = bus_to_display(bus_deg);
    s_encoder_target_deg = disp;
    s_encoder_tenths = wrap_tenths_deg(
        static_cast<int>(std::trunc(static_cast<double>(disp) * 10.0)));

    lvgl_port_lock(-1);
    char buf[16];
    fmt_de(buf, sizeof(buf), disp);
    if (objects.taget_dg) {
        lv_textarea_set_text(objects.taget_dg, buf);
    }
    if (ENCODER_MOVES_ARC && objects.grad_acc) {
        s_arc_updating = true;
        lv_arc_set_value(objects.grad_acc, deg_to_arc_value(disp));
        s_arc_updating = false;
    }
    lvgl_port_unlock();
}
