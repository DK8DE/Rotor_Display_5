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
#include "rotor_rs485.h"
#include "ui/screens.h"

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
    const int t = static_cast<int>(std::trunc(static_cast<double>(deg) * 10.0));
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

/** Zuletzt von GETPOSDG (Ist), für Arc-Sprung beim Loslassen */
static float s_last_ist_deg = 0.0f;

/** Eine Nachkommastelle für Anzeige: zweite Stelle abschneiden (trunc), nicht runden — wie Bus 273,77 → 273,7 */
static void fmt_de(char *buf, size_t n, float deg)
{
    const double t = std::trunc(static_cast<double>(deg) * 10.0) / 10.0;
    snprintf(buf, n, "%.1f", t);
    for (char *p = buf; *p; ++p) {
        if (*p == '.') {
            *p = ',';
        }
    }
}

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

/** Soll aus Bus (Echo, Ankunft, PC-Loopback) — nur taget_dg, Arc folgt Ist über on_position_deg */
static void on_target_deg(float deg)
{
    /* Während Encoder aktiv ist: Bus darf taget_dg NICHT überschreiben.
     * Sonst überschreibt z. B. „Ankunft am alten SETPOS“ (notify_target(s_goto_commanded_deg))
     * einen bereits weiter gedrehten Soll — wirkt wie „1. Klick fehlt“ / falsches Ziel. */
    if (s_encoder_adjusting) {
        return;
    }
    lvgl_port_lock(-1);
    char buf[16];
    fmt_de(buf, sizeof(buf), deg);
    if (objects.taget_dg) {
        lv_textarea_set_text(objects.taget_dg, buf);
    }
    lvgl_port_unlock();
}

static void on_position_deg(float deg)
{
    s_last_ist_deg = deg;
    lvgl_port_lock(-1);
    char buf[16];
    fmt_de(buf, sizeof(buf), deg);
    if (objects.actual_dg) {
        lv_textarea_set_text(objects.actual_dg, buf);
    }
    if (!s_arc_dragging && !s_encoder_adjusting && objects.grad_acc) {
        s_arc_updating = true;
        lv_arc_set_value(objects.grad_acc, deg_to_arc_value(deg));
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
            lv_arc_set_value(objects.grad_acc, deg_to_arc_value(s_last_ist_deg));
            s_arc_updating = false;
        }
        (void)rotor_rs485_goto_degrees(target_deg);
    }
}

extern "C" void rotor_app_init(void)
{
    rotor_rs485_set_master_id(2);
    rotor_rs485_set_slave_id(20);
    rotor_rs485_set_ref_callback(on_ref_status);
    rotor_rs485_set_position_callback(on_position_deg);
    rotor_rs485_set_target_callback(on_target_deg);
    rotor_rs485_init();
    if (objects.homing_led) {
        lv_led_set_color(objects.homing_led, lv_color_hex(0xff0000));
    }
    if (objects.ref) {
        lv_obj_add_event_cb(objects.ref, on_ref_button, LV_EVENT_CLICKED, nullptr);
    }
    if (objects.grad_acc) {
        lv_obj_add_event_cb(objects.grad_acc, on_arc, LV_EVENT_ALL, nullptr);
    }
    /* GETREF/GETPOSDG nach 2 s Bus-Bereitschaft — siehe rotor_rs485_init / rotor_rs485_loop */
}

/** @return true wenn SETPOSDG gestartet */
static bool encoder_apply_goto(float target_deg)
{
    lvgl_port_lock(-1);
    if (ENCODER_MOVES_ARC && objects.grad_acc) {
        s_arc_updating = true;
        lv_arc_set_value(objects.grad_acc, deg_to_arc_value(s_last_ist_deg));
        s_arc_updating = false;
    }
    lvgl_port_unlock();
    return rotor_rs485_goto_degrees(target_deg);
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

extern "C" void rotor_app_snap_target_to_deg(float deg)
{
    s_encoder_adjusting = false;
    s_encoder_goto_retry_pending = false;
    s_encoder_retry_deadline_ms = 0;
    s_encoder_idle_deadline_ms = 0;
    s_encoder_target_deg = deg;
    s_encoder_tenths = wrap_tenths_deg(
        static_cast<int>(std::trunc(static_cast<double>(deg) * 10.0)));

    lvgl_port_lock(-1);
    char buf[16];
    fmt_de(buf, sizeof(buf), deg);
    if (objects.taget_dg) {
        lv_textarea_set_text(objects.taget_dg, buf);
    }
    if (ENCODER_MOVES_ARC && objects.grad_acc) {
        s_arc_updating = true;
        lv_arc_set_value(objects.grad_acc, deg_to_arc_value(deg));
        s_arc_updating = false;
    }
    lvgl_port_unlock();
}
