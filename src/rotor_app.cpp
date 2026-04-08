/**
 * Verbindet EEZ-UI (ref, homing_led, grad_acc, taget_dg, actual_dg) mit rotor_rs485 — ohne Änderungen in src/ui.
 * Arc: Wert = mechanische Buslage (0..360°); Rotation = EEZ-Basis (270°) + Antennenversatz — Skala dreht beim Antennenwechsel, Zeiger bleibt zur Mechanik.
 * Encoder-Soll in Zehntelgrad (Skalierung siehe main ENCODER_DELTA_TENTHS_PER_STEP).
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
#include "touch_feedback.h"
#include "ui/screens.h"

extern "C" void rotor_app_antenna_offset_changed(void);
static void rotor_app_antenna_switch_from_ui(uint8_t prev_antenna_1_to_3, bool send_bus_goto);

/** EEZ-Export: lv_img_set_angle am Pfeil; nach Init auf 0, Drehung nur per Style-Transform (siehe rotor_app_init). */
static int16_t s_pfeil_wind_eez_base_angle01 = 0;

static bool s_arc_dragging = false;
/** Zwischen PRESSED und RELEASED: mindestens ein VALUE_CHANGED (echter Dreh am Arc)? */
static bool s_arc_moved_this_press = false;
/** lv_arc_get_value beim Drücken — falls VALUE_CHANGED ausbleibt, Loslassen trotzdem als Dreh erkennen */
static int s_arc_value_at_press = 0;
static bool s_arc_updating = false;
/** Encoder: Soll einstellen, Ist-Nachführung am Arc aus */
static bool s_encoder_adjusting = false;
/** true: letzter encoder_apply_goto ist fehlgeschlagen — rotor_app_loop soll erneut senden */
static bool s_encoder_goto_retry_pending = false;
/** Nächster Retry-Zeitpunkt (millis), sobald s_encoder_goto_retry_pending */
static uint32_t s_encoder_retry_deadline_ms = 0;
/** Nach letztem Encoder-Tick: erst wenn millis() >= dieser Zeit → SETPOSDG (kein Senden während Drehens) */
static uint32_t s_encoder_idle_deadline_ms = 0;
/** millis() beim letzten rotor_app_encoder_step — Bus-Soll darf UI nicht um 0,1° zurücksetzen */
static uint32_t s_last_encoder_step_ms = 0;
static float s_encoder_target_deg = 0.0f;
/** Encoder: Zehntelgrad 0..3599 (±1 pro Klick = ±0,1°); Arc 0..360 = 1°-Schritte */
static int s_encoder_tenths = 0;
/** Nur Arc: letzter lv_arc_set_value-Integer — gleicher Wert → kein erneutes grad_acc_sync (Hot-Path). */
static int s_encoder_arc_int_cached = -32768;
/** Nach Antennenwechsel: on_target_deg filtert Bus-Soll-Echos, die noch zum alten Geometrie-/Slave-Soll passen. */
static uint32_t s_taget_ignore_bus_target_until_ms = 0;
static constexpr uint32_t ENCODER_BUS_RETRY_MS = 50;
/** Pause ohne neuen Tick bis SETPOS. War 1400 ms (langsam drehen); Ziel: ~1/3 für schnellere Busreaktion,
 *  Fallback Session-Start nutzt s_encoder_target_deg (kein Arc-1°-Verlust) bleibt aktiv. */
static constexpr uint32_t ENCODER_SEND_IDLE_MS = 670;
/** true: wie Touch am Arc — Drehen zeigt Soll am Arc; vor SETPOSDG Arc auf Ist (Startlage), dann Fahrt */
static constexpr bool ENCODER_MOVES_ARC = true;

/** Tab Rotor_Info: Encoder/Tippen nur Vorschau; Flash nur per HW-Taster und nur bei geänderter Zahl */
enum class IdFieldFocus : uint8_t { None = 0, RotorId, ControllerId };
static IdFieldFocus s_id_field_focus = IdFieldFocus::None;
/** Verhindert Rekursion bei lv_textarea_set_text → VALUE_CHANGED */
static bool s_id_field_programmatic_text = false;

/** Slow/Fast: Bus belegt → PWM erneut in rotor_pwm_ui_loop */
static uint8_t s_pwm_deferred = 255;
/** Nach Start: einmal Fast-PWM aus config (SETPWM) */
static bool s_pwm_boot_send_pending = true;
/** UI: Fast-Taste aktiv (sonst Slow) — für Config-Sync vom Bus */
/** Standard: Fast (wie Boot-SETPWM); Slow nur nach Tipp auf „Slow“. */
static bool s_pwm_ui_is_fast = true;

static void pwm_style_slow_fast(bool fast_active)
{
    const lv_color_t c_on = lv_color_hex(0x087321);
    const lv_color_t c_off = lv_color_hex(0x2196f3);
    const lv_color_t c_txt_on_green = lv_color_hex(0xFFFFFF);
    const lv_color_t c_txt_on_blue = lv_color_hex(0x000000);
    if (objects.slow) {
        lv_obj_set_style_bg_color(objects.slow, fast_active ? c_off : c_on, LV_PART_MAIN);
    }
    if (objects.fast) {
        lv_obj_set_style_bg_color(objects.fast, fast_active ? c_on : c_off, LV_PART_MAIN);
    }
    /* Grün aktiv → weiße Schrift; blau inaktiv → schwarz */
    if (objects.label__slow) {
        lv_obj_set_style_text_color(objects.label__slow, fast_active ? c_txt_on_blue : c_txt_on_green,
                                    LV_PART_MAIN);
    }
    if (objects.label_fast) {
        lv_obj_set_style_text_color(objects.label_fast, fast_active ? c_txt_on_green : c_txt_on_blue,
                                    LV_PART_MAIN);
    }
}

/** Pieps bei Finger-Druck (PRESSED) — zuverlässiger als CLICKED (Letzteres fehlt z. B. bei Tab-Scroll/Noise). */
static void on_ui_button_press_beep(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_PRESSED) {
        return;
    }
    if (rotor_error_app_is_fault_locked()) {
        return;
    }
    touch_feedback_button_click();
}

static void on_slow_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    if (rotor_error_app_is_fault_locked()) {
        return;
    }
    s_pwm_ui_is_fast = false;
    pwm_config_set_pwm_ui_fast(0);
    pwm_config_save();
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
    if (rotor_error_app_is_fault_locked()) {
        return;
    }
    s_pwm_ui_is_fast = true;
    pwm_config_set_pwm_ui_fast(1);
    pwm_config_save();
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
    const lv_color_t c_txt_on_green = lv_color_hex(0xFFFFFF);
    const lv_color_t c_txt_on_blue = lv_color_hex(0x000000);
    lv_obj_t *btns[3] = { objects.antenna_1, objects.antenna_2, objects.antenna_3 };
    lv_obj_t *labels[3] = { objects.antenna_1_label, objects.antenna_2_label, objects.antenna_3_label };
    for (uint8_t i = 0; i < 3; i++) {
        if (!btns[i]) {
            continue;
        }
        const bool on = (active_1_to_3 == (uint8_t)(i + 1));
        lv_obj_set_style_bg_color(btns[i], on ? c_on : c_off, LV_PART_MAIN);
        if (labels[i]) {
            lv_obj_set_style_text_color(labels[i], on ? c_txt_on_green : c_txt_on_blue, LV_PART_MAIN);
        }
    }
}

extern "C" void rotor_app_apply_remote_antenna_selection(uint8_t n_1_to_3)
{
    if (n_1_to_3 < 1u || n_1_to_3 > 3u) {
        return;
    }
    const uint8_t prev = pwm_config_get_last_antenna();
    if (prev == n_1_to_3) {
        return;
    }
    pwm_config_set_last_antenna(n_1_to_3);
    pwm_config_save();
    rotor_rs485_send_setaselect(n_1_to_3);
    lvgl_port_lock(-1);
    antenna_apply_style(n_1_to_3);
    /* Mitläufer (fremder Master): kein eigenes SETPOSDG — PC-Software sendet Soll; nur Anzeige/Arc anpassen. */
    rotor_app_antenna_switch_from_ui(prev, !rotor_rs485_is_foreign_pc_listen_mode());
    lvgl_port_unlock();
}

static void on_antenna_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    if (rotor_error_app_is_fault_locked()) {
        return;
    }
    lv_obj_t *btn = lv_event_get_target(e);
    uint8_t n = 1;
    if (btn == objects.antenna_2) {
        n = 2;
    } else if (btn == objects.antenna_3) {
        n = 3;
    }
    const uint8_t prev = pwm_config_get_last_antenna();
    if (prev == n) {
        return;
    }
    pwm_config_set_last_antenna(n);
    pwm_config_save();
    antenna_apply_style(n);
    rotor_rs485_send_setaselect(n);
    rotor_app_antenna_switch_from_ui(prev, true);
}

/** hauptanzeige: Tab 0…4 = Position, Fast, Antennen, Temperaturen_Wind, Rotor_Info */
static constexpr uint32_t k_weather_tab_idx = 3u;

static void apply_anemometer_weather_tab_visibility(void)
{
    if (!objects.hauptanzeige || !objects.temperaturen_wind) {
        return;
    }
    const bool show = pwm_config_get_anemometer() != 0;
    lv_obj_t *const tv = objects.hauptanzeige;
    lv_obj_t *const btns = lv_tabview_get_tab_btns(tv);
    if (show) {
        lv_obj_clear_flag(objects.temperaturen_wind, LV_OBJ_FLAG_HIDDEN);
        if (btns) {
            lv_btnmatrix_clear_btn_ctrl(btns, static_cast<uint16_t>(k_weather_tab_idx),
                                        LV_BTNMATRIX_CTRL_HIDDEN);
        }
    } else {
        if (static_cast<uint32_t>(lv_tabview_get_tab_act(tv)) == k_weather_tab_idx) {
            lv_tabview_set_act(tv, 0, LV_ANIM_OFF);
        }
        lv_obj_add_flag(objects.temperaturen_wind, LV_OBJ_FLAG_HIDDEN);
        if (btns) {
            lv_btnmatrix_set_btn_ctrl(btns, static_cast<uint16_t>(k_weather_tab_idx),
                                      LV_BTNMATRIX_CTRL_HIDDEN);
        }
    }
    lv_obj_invalidate(tv);
}

/** Beschriftung auf encoder_delta_bu (Label-Kind): aktive Schrittweite */
static void encoder_delta_apply_button_label(void)
{
    if (!objects.encoder_delta_lable) {
        return;
    }
    const uint8_t d = pwm_config_get_encoder_delta_tenths();
    const char *txt = (d >= 10u) ? "1 Grad" : "0,1 Grad";
    lv_label_set_text(objects.encoder_delta_lable, txt);
}

static void on_encoder_delta_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    if (rotor_error_app_is_fault_locked()) {
        return;
    }
    const uint8_t cur = pwm_config_get_encoder_delta_tenths();
    const uint8_t next = (cur >= 10u) ? 1u : 10u;
    pwm_config_set_encoder_delta_tenths(next);
    pwm_config_save();
    encoder_delta_apply_button_label();
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

static void id_fields_set_text(lv_obj_t *ta, unsigned id_1_to_254)
{
    if (!ta) {
        return;
    }
    char buf[8];
    snprintf(buf, sizeof(buf), "%u", id_1_to_254);
    s_id_field_programmatic_text = true;
    lv_textarea_set_text(ta, buf);
    s_id_field_programmatic_text = false;
}

/** Nach load / RS485-SET: Anzeige = config (ohne Fokus auf ID-Felder zu ändern) */
static void id_fields_sync_textareas_from_config(void)
{
    id_fields_set_text(objects.rotor_id, pwm_config_get_rotor_id());
    id_fields_set_text(objects.controller_id, pwm_config_get_master_id());
}

/** Nur parsen (1…254), kein Schreiben */
static bool id_field_parse_ta_id(lv_obj_t *ta, uint8_t *out)
{
    if (!ta || !out) {
        return false;
    }
    const char *p = lv_textarea_get_text(ta);
    if (!p) {
        return false;
    }
    while (*p == ' ' || *p == '\t') {
        ++p;
    }
    if (*p == '\0') {
        return false;
    }
    char *end = nullptr;
    const long v = strtol(p, &end, 10);
    if (end == p) {
        return false;
    }
    while (*end == ' ' || *end == '\t') {
        ++end;
    }
    if (*end != '\0') {
        return false;
    }
    if (v < 1L || v > 254L) {
        return false;
    }
    *out = static_cast<uint8_t>(v);
    return true;
}

/**
 * Nur beim HW-Taster: gültige Zahl → bei Abweichung von der Config RS485 + pwm_config_save();
 * gleicher Wert → nur Text normalisieren, kein Flash-Schreiben.
 * @return false bei ungültigem Inhalt (kein save).
 */
static bool id_field_try_commit_text(lv_obj_t *ta, bool is_rotor_id)
{
    if (!ta) {
        return false;
    }
    uint8_t nv = 0;
    if (!id_field_parse_ta_id(ta, &nv)) {
        return false;
    }
    const uint8_t cur =
        is_rotor_id ? pwm_config_get_rotor_id() : pwm_config_get_master_id();
    if (nv != cur) {
        if (is_rotor_id) {
            pwm_config_set_rotor_id(nv);
            rotor_rs485_set_slave_id(nv);
        } else {
            pwm_config_set_master_id(nv);
            rotor_rs485_set_master_id(nv);
        }
        pwm_config_save();
    }
    id_fields_set_text(ta, nv);
    return true;
}

static uint8_t id_field_display_or_saved_config(lv_obj_t *ta, bool is_rotor_id)
{
    uint8_t v = 0;
    if (id_field_parse_ta_id(ta, &v)) {
        return v;
    }
    return is_rotor_id ? pwm_config_get_rotor_id() : pwm_config_get_master_id();
}

static void id_field_blur(lv_obj_t *ta)
{
    if (!ta) {
        return;
    }
    lv_group_t *const g = static_cast<lv_group_t *>(lv_obj_get_group(ta));
    if (g) {
        lv_group_focus_next(g);
    } else {
        lv_obj_clear_state(ta, LV_STATE_FOCUSED | LV_STATE_FOCUS_KEY);
    }
}

extern "C" bool rotor_app_commit_id_field_on_hw_click(void)
{
    if (s_id_field_focus == IdFieldFocus::None) {
        return false;
    }
    lvgl_port_lock(-1);
    const bool is_rotor = (s_id_field_focus == IdFieldFocus::RotorId);
    lv_obj_t *const ta = is_rotor ? objects.rotor_id : objects.controller_id;
    if (!ta) {
        s_id_field_focus = IdFieldFocus::None;
        lvgl_port_unlock();
        return false;
    }
    if (!id_field_try_commit_text(ta, is_rotor)) {
        const uint8_t id = is_rotor ? pwm_config_get_rotor_id() : pwm_config_get_master_id();
        id_fields_set_text(ta, id);
    }
    id_field_blur(ta);
    s_id_field_focus = IdFieldFocus::None;
    lvgl_port_unlock();
    return true;
}

static void on_id_field_event(lv_event_t *e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_FOCUSED && code != LV_EVENT_DEFOCUSED && code != LV_EVENT_VALUE_CHANGED) {
        return;
    }
    lv_obj_t *ta = lv_event_get_target(e);
    const bool is_rotor = (ta == objects.rotor_id);

    if (code == LV_EVENT_FOCUSED) {
        s_id_field_focus = is_rotor ? IdFieldFocus::RotorId : IdFieldFocus::ControllerId;
        const uint8_t id = is_rotor ? pwm_config_get_rotor_id() : pwm_config_get_master_id();
        id_fields_set_text(ta, id);
        /* Encoder soll Winkel nicht weiter bedienen; Bus darf taget_dg wieder nachführen */
        s_encoder_adjusting = false;
        s_encoder_goto_retry_pending = false;
        s_encoder_retry_deadline_ms = 0;
        s_encoder_idle_deadline_ms = 0;
        return;
    }
    if (code == LV_EVENT_DEFOCUSED) {
        /* Kein Flash-Schreiben beim Verlassen — nur gespeicherten Wert anzeigen */
        const uint8_t id = is_rotor ? pwm_config_get_rotor_id() : pwm_config_get_master_id();
        id_fields_set_text(ta, id);
        s_id_field_focus = IdFieldFocus::None;
        return;
    }
    if (code == LV_EVENT_VALUE_CHANGED) {
        if (s_id_field_programmatic_text) {
            return;
        }
        /* Tippen: nur Vorschau; Speichern nur per HW-Taster wenn Zahl sich geändert hat */
        return;
    }
}

extern "C" void rotor_app_config_changed_from_bus(void)
{
    lvgl_port_lock(-1);
    antenna_apply_labels_from_config();
    antenna_apply_style(pwm_config_get_last_antenna());
    s_pwm_ui_is_fast = pwm_config_get_pwm_ui_fast() != 0;
    pwm_style_slow_fast(s_pwm_ui_is_fast);
    rotor_rs485_set_master_id(pwm_config_get_master_id());
    rotor_rs485_set_slave_id(pwm_config_get_rotor_id());
    id_fields_sync_textareas_from_config();
    apply_anemometer_weather_tab_visibility();
    encoder_delta_apply_button_label();
    const uint8_t p = s_pwm_ui_is_fast ? pwm_config_get_fast() : pwm_config_get_slow();
    if (!rotor_rs485_send_set_pwm_limit(p)) {
        s_pwm_deferred = p;
    }
    lvgl_port_unlock();
}

static int wrap_tenths_deg(int t)
{
    t %= 3600;
    if (t < 0) {
        t += 3600;
    }
    return t;
}

/**
 * Soll aus taget_dg — ohne strtof: 273,7 bzw. 273.7 exakt als Zehntel 2737 (kein Float-Rundungsfehler
 * bei .2/.7, der sonst einen Zehntelschritt beim Session-Start frisst).
 */
static bool parse_taget_text_to_tenths(int *out_tenths)
{
    if (!objects.taget_dg || !out_tenths) {
        return false;
    }
    const char *p = lv_textarea_get_text(objects.taget_dg);
    if (!p || !*p) {
        return false;
    }
    while (*p == ' ' || *p == '\t') {
        ++p;
    }
    int hi = 0;
    bool any_digit = false;
    while (*p >= '0' && *p <= '9') {
        any_digit = true;
        hi = hi * 10 + (*p - '0');
        if (hi > 4000) {
            return false;
        }
        ++p;
    }
    int lo = 0;
    if (*p == ',' || *p == '.') {
        ++p;
        if (*p >= '0' && *p <= '9') {
            lo = *p - '0';
            ++p;
        }
    }
    if (!any_digit) {
        return false;
    }
    const int t = hi * 10 + lo;
    *out_tenths = wrap_tenths_deg(t);
    return true;
}

/** EEZ screens.c: lv_arc_set_rotation(grad_acc, 270) */
static constexpr int GRAD_ACC_BASE_ROTATION = 270;

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

/** Kompass = Buslage + Versatz der angegebenen Antenne (1…3) — beim Wechsel: Strahl mit alter Antenne. */
static float bus_to_display_for_idx(float bus_deg_ui, int ant_1_to_3)
{
    const float off = pwm_config_get_antoff_deg(ant_1_to_3);
    if (bus_deg_ui >= 359.5f) {
        if (std::fabs(static_cast<double>(off)) < 1e-6) {
            return 360.0f;
        }
        return norm360_add(360.0f + off);
    }
    return norm360_add(bus_deg_ui + off);
}

/** Anzeige (Kompass / Antennenrichtung) = Buslage + Versatz der aktuell gewählten Antenne */
static float bus_to_display(float bus_deg_ui)
{
    return bus_to_display_for_idx(bus_deg_ui, static_cast<int>(pwm_config_get_last_antenna()));
}

/** Soll am Bus für SETPOSDG aus Anzeige-Winkel und Antenne ant_1_to_3 */
static float display_to_bus_for_idx(float display_deg, int ant_1_to_3)
{
    const float off = pwm_config_get_antoff_deg(ant_1_to_3);
    if (display_deg >= 359.5f) {
        if (std::fabs(static_cast<double>(off)) < 1e-6f) {
            return 360.0f;
        }
        return norm360_add(360.0f - off);
    }
    return norm360_add(display_deg - off);
}

/** Soll am Bus für SETPOSDG aus Anzeige-Winkel (aktuelle last_antenna) */
static float display_to_bus(float display_deg)
{
    return display_to_bus_for_idx(display_deg, static_cast<int>(pwm_config_get_last_antenna()));
}

float rotor_app_get_display_direction_deg(void)
{
    return bus_to_display(s_last_bus_ist_deg);
}

/** lv_arc_get_value → Busgrad (Arc zeigt Mechanik, nicht Anzeige-Kompass) */
static float arc_int_value_to_bus_deg(int v)
{
    if (v >= 360) {
        return 360.0f;
    }
    return static_cast<float>(v);
}

static int grad_acc_rotation_from_antoff(void)
{
    const float off = active_antenna_offset_deg();
    long r = static_cast<long>(GRAD_ACC_BASE_ROTATION)
        + std::lround(static_cast<double>(off));
    r %= 360;
    if (r < 0) {
        r += 360;
    }
    return static_cast<int>(r);
}

/** Arc: Ist = Buslage; Rotation = Basis + Antennenversatz (Kompass-Skala kippt mit Antenne) */
static void grad_acc_sync_mechanical(float bus_deg_ui)
{
    if (!objects.grad_acc) {
        return;
    }
    s_arc_updating = true;
    lv_arc_set_value(objects.grad_acc, deg_to_arc_value(bus_deg_ui));
    lv_arc_set_rotation(objects.grad_acc, grad_acc_rotation_from_antoff());
    s_arc_updating = false;
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

/** Soll-Anzeige taget: immer aus ganzen Zehnteln — kein tenths/10.0f + %.1f (sonst fehlende ,8/,3). */
static void fmt_taget_from_wrapped_tenths(char *buf, size_t n, int tenths)
{
    const int t = wrap_tenths_deg(tenths);
    snprintf(buf, n, "%d,%d", t / 10, t % 10);
}

static void fmt_taget_from_display_deg(char *buf, size_t n, float deg)
{
    if (deg >= 359.5f) {
        snprintf(buf, n, "360,0");
        return;
    }
    const int t = wrap_tenths_deg(static_cast<int>(
        std::floor(static_cast<double>(deg) * 10.0 + 1e-4)));
    fmt_taget_from_wrapped_tenths(buf, n, t);
}

/**
 * EEZ: taget_dg hat max_length + accepted_chars — lv_textarea_set_text() ist für 0,1°-Schritte
 * unzuverlässig. Internes Label setzen.
 * Kein lv_textarea_set_cursor_pos(LAST): bei gleicher Cursor-Länge early-exit in LVGL v8 ohne
 * refr_cursor_area → fehlender Redraw bei nur geänderter Nachkommastelle (Komma-Text).
 * lv_refr_now(NULL): Refresh sofort, nicht erst beim nächsten lv_timer_handler-Slot.
 * Encoder: sync_full_refr_now=false — kein synchrones Voll-Rendering; LVGL-Task zeichnet asynchron
 * (sonst blockiert loop() bei schnellem Drehen trotz Bündelung).
 */
static void taget_dg_set_display_text(const char *buf, bool sync_full_refr_now = true)
{
    if (!objects.taget_dg || !buf) {
        return;
    }
    lv_obj_t *const lab = lv_textarea_get_label(objects.taget_dg);
    if (!lab) {
        return;
    }
    lv_label_set_text(lab, buf);
    lv_obj_mark_layout_as_dirty(objects.taget_dg);
    lv_obj_t *const par = lv_obj_get_parent(objects.taget_dg);
    if (par) {
        lv_obj_mark_layout_as_dirty(par);
    }
    lv_obj_invalidate(lab);
    lv_obj_invalidate(objects.taget_dg);
    if (sync_full_refr_now) {
        lv_refr_now(nullptr);
    }
}

/**
 * Ist-Feld: max_length → lv_textarea_set_text baut den Text offiziell neu; nur Label zu setzen kann kurz
 * richtig anzeigen, bis die Textarea intern wieder mit altem Puffer synchronisiert (Sprung zurück).
 * Zusätzlich Label + invalidate wie bei taget (0,1°-Redraw).
 */
static void actual_dg_set_display_text(const char *buf, bool sync_full_refr_now = false)
{
    if (!objects.actual_dg || !buf) {
        return;
    }
    lv_textarea_set_text(objects.actual_dg, buf);
    lv_obj_t *const lab = lv_textarea_get_label(objects.actual_dg);
    if (lab) {
        lv_label_set_text(lab, buf);
        lv_obj_invalidate(lab);
    }
    lv_obj_mark_layout_as_dirty(objects.actual_dg);
    lv_obj_t *const par = lv_obj_get_parent(objects.actual_dg);
    if (par) {
        lv_obj_mark_layout_as_dirty(par);
    }
    lv_obj_invalidate(objects.actual_dg);
    if (sync_full_refr_now) {
        lv_refr_now(nullptr);
    }
}

/** RS485-Pfad darf nicht direkt lvgl_port_lock + Label setzen (WDT/Deadlock mit LVGL-Task). */
static void on_ref_status(bool referenced)
{
    lvgl_port_lock(-1);
    if (objects.homing_led) {
        /* Fehler / Start-GETERR ausstehend: nicht grün nur wegen GETREF=1 */
        if (rotor_error_app_get_error_code() != 0 || !rotor_rs485_is_startup_error_checked()) {
            lv_led_set_color(objects.homing_led, lv_color_hex(0xff0000));
            lv_led_set_brightness(objects.homing_led, 255);
        } else {
            lv_led_set_color(objects.homing_led,
                             referenced ? lv_color_hex(0x43b302) : lv_color_hex(0xff0000));
            lv_led_set_brightness(objects.homing_led, 255);
        }
    }
    /* Ohne Referenz kein gültiger Ist-Winkel vom Slave — alte Anzeige verwirrt nach Rotor-Neustart.
     * Kein Unicode U+2014 (—): eingebettete Font hat kein Glyph → lv_draw_sw_letter Warnung. */
    if (!referenced && objects.actual_dg) {
        actual_dg_set_display_text("-", false);
    }
    lvgl_port_unlock();
}

/**
 * True, wenn bus_to_display(Soll) in Zehntelgrad nur um typisches Float-Rauschen von der Encoder-Session
 * abweicht (z. B. 353,7499° → floor → 353,7 angezeigt obwohl Soll 353,8). Dann taget_dg nicht überschreiben.
 * Fremd-Master-Fahrt: nie wegfiltern — PC-Soll kann bewusst um 0,1° anders sein.
 */
static bool bus_target_matches_session_tenth_noise(float disp_deg, int enc_tenths)
{
    double u = static_cast<double>(disp_deg) * 10.0;
    double e = static_cast<double>(enc_tenths);
    double diff = u - e;
    if (diff > 1800.0) {
        diff -= 3600.0;
    } else if (diff < -1800.0) {
        diff += 3600.0;
    }
    return std::fabs(diff) < 0.5;
}

/** Soll aus Bus (Echo, Ankunft, PC-Loopback) — taget_dg in Anzeige-Koordinaten (mit Versatz) */
static void on_target_deg(float bus_deg)
{
    /* Während Encoder aktiv ist: Bus darf taget_dg NICHT überschreiben.
     * Sonst überschreibt z. B. „Ankunft am alten SETPOS“ (notify_target(s_goto_commanded_deg))
     * einen bereits weiter gedrehten Soll — wirkt wie „1. Klick fehlt“ / falsches Ziel.
     * Ausnahme: SETPOSDG von anderem Master (Mithören) — explizite PC-Fahrt, Session beenden. */
    if (s_encoder_adjusting) {
        if (rotor_rs485_is_remote_setpos_motion()) {
            s_encoder_adjusting = false;
            s_encoder_goto_retry_pending = false;
            s_encoder_retry_deadline_ms = 0;
            s_encoder_idle_deadline_ms = 0;
        } else {
            return;
        }
    }
    /* loop(): serial_bridge/RS485 vor encoder_process_pending — alter Bus-Soll sonst vor dem Encoder-Tick. */
    if (rotor_encoder_pending_detents() != 0) {
        return;
    }
    /* Kurz nach Antennenwechsel: Slave meldet noch den alten Bus-Soll — mit neuem Versatz falsch in Anzeige;
     * erst Echo zum neuen SETPOSDG (gleicher Anzeige-Soll wie s_encoder_tenths) anwenden. */
    if (millis() < s_taget_ignore_bus_target_until_ms) {
        const float disp_probe = bus_to_display(bus_deg);
        const int incoming_t = wrap_tenths_deg(static_cast<int>(
            std::floor(static_cast<double>(disp_probe) * 10.0 + 1e-4)));
        int d = incoming_t - s_encoder_tenths;
        if (d > 1800) {
            d -= 3600;
        }
        if (d < -1800) {
            d += 3600;
        }
        if (d > 2 || d < -2) {
            return;
        }
    }
    const float disp = bus_to_display(bus_deg);
    /* Nicht um 0,1° zurückspringen: gleicher physikalischer Soll, nur floor/Float unterhalb der Session-Zehntel. */
    if (!rotor_rs485_is_remote_setpos_motion() &&
        bus_target_matches_session_tenth_noise(disp, s_encoder_tenths)) {
        return;
    }
    /* 0,1°/Klick: Bus-Soll (notify_target / Echo) oft exakt eine Zehntelstufe hinter der Encoder-Session
     * (Rundung/Slave noch nicht nachgezogen). Früher nur 900 ms Schutz → danach sprang taget zurück,
     * SETPOSDG lief mit veraltetem s_encoder_target_deg / gar kein sinnvoller Versand. Mehrere Zehntel
     * auf einmal: d != -1, kein Konflikt. */
    const int bus_t = wrap_tenths_deg(
        static_cast<int>(std::floor(static_cast<double>(disp) * 10.0 + 1e-4)));
    {
        int d = bus_t - s_encoder_tenths;
        if (d > 1800) {
            d -= 3600;
        }
        if (d < -1800) {
            d += 3600;
        }
        constexpr uint32_t kBusTargetOneTenthLagProtectMs = 4500u;
        if (!rotor_rs485_is_remote_setpos_motion() &&
            (uint32_t)(millis() - s_last_encoder_step_ms) < kBusTargetOneTenthLagProtectMs && d == -1) {
            return;
        }
    }
    /* Gleiche Zehntel wie fmt_de/Encoder-Session — sonst zeigt taget_dg X, intern bleibt Y,
     * nächster Encoder-Tick parst X und „verschluckt“ einen Schritt / doppelter Sprung. */
    s_encoder_target_deg = disp;
    s_encoder_tenths = wrap_tenths_deg(
        static_cast<int>(std::floor(static_cast<double>(disp) * 10.0 + 1e-4)));

    lvgl_port_lock(-1);
    char buf[16];
    fmt_taget_from_display_deg(buf, sizeof(buf), disp);
    /* Kein lv_refr_now: bei Bus-Flut blockiert synchrones Rendering die RS485-Zeilenverarbeitung. */
    taget_dg_set_display_text(buf, false);
    lvgl_port_unlock();
}

static void on_position_deg(float bus_deg_ui)
{
    /* Kein gültiger Ist-Winkel ohne Referenz (trotz GETPOSDG vom PC während Homing). */
    if (!rotor_rs485_is_referenced()) {
        lvgl_port_lock(-1);
        if (objects.actual_dg) {
            actual_dg_set_display_text("-", false);
        }
        lvgl_port_unlock();
        return;
    }
    s_last_bus_ist_deg = bus_deg_ui;
    lvgl_port_lock(-1);
    char buf[16];
    const float disp = bus_to_display(bus_deg_ui);
    fmt_de(buf, sizeof(buf), disp);
    if (objects.actual_dg) {
        actual_dg_set_display_text(buf, false);
    }
    /* Während Positionsfahrt: Arc immer aus Ist (GETPOSDG), auch wenn Encoder-Flag noch gesetzt ist
     * (Beschleunigungs-Detents / Überlappung) — sonst „steht“ der Zeiger, actual läuft weiter. */
    if (!s_arc_dragging &&
        (!s_encoder_adjusting || rotor_rs485_is_position_polling()) &&
        objects.grad_acc) {
        grad_acc_sync_mechanical(bus_deg_ui);
    }
    lvgl_port_unlock();
}

static void on_arc(lv_event_t *e)
{
    lv_event_code_t c = lv_event_get_code(e);
    lv_obj_t *arc = lv_event_get_target(e);

    if (c == LV_EVENT_PRESSED) {
        s_arc_dragging = true;
        s_arc_moved_this_press = false;
        s_arc_value_at_press = lv_arc_get_value(arc);
        /* Encoder-Session hier NICHT abbrechen: kurzer Touch ohne Drehen soll keine Klicks „verschlucken“
         * und kein ausstehendes SETPOSDG verwerfen — erst bei echtem Drag (VALUE_CHANGED). */
    }
    if (c == LV_EVENT_VALUE_CHANGED) {
        if (rotor_error_app_is_fault_locked()) {
            return;
        }
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
        const int v = lv_arc_get_value(arc);
        const float bus_deg = arc_int_value_to_bus_deg(v);
        const float disp = bus_to_display(bus_deg);
        char buf[16];
        fmt_taget_from_display_deg(buf, sizeof(buf), disp);
        taget_dg_set_display_text(buf);
    }
    if (c == LV_EVENT_RELEASED) {
        s_arc_dragging = false;
        /* Pieps immer beim Loslassen (Arc-Callback nur bei Touch auf dem Arc) — nicht hinter
         * s_arc_updating verstecken: sonst kein Ton und kein GOTO, wenn zufällig Flag noch stand. */
        touch_feedback_arc_release();
        if (rotor_error_app_is_fault_locked()) {
            return;
        }
        const int target_v = lv_arc_get_value(arc);
        /* VALUE_CHANGED fehlt manchmal (kurzer Zug, gleicher ganzzahliger Grad, Coalescing) — Abgleich Wert Press/Release. */
        const bool moved = s_arc_moved_this_press || (target_v != s_arc_value_at_press);
        /* Nur nach echtem Drehen: GOTO — sonst (Finger kurz auf Arc) Encoder/Bus nicht mit Arc-Wert überschreiben. */
        if (!moved) {
            return;
        }
        const float bus_tgt = arc_int_value_to_bus_deg(target_v);
        char buf[16];
        fmt_taget_from_display_deg(buf, sizeof(buf), bus_to_display(bus_tgt));
        taget_dg_set_display_text(buf);
        /* Arc zeigt Ist (mitfahren), nicht auf dem Ziel stehen bleiben */
        if (objects.grad_acc) {
            grad_acc_sync_mechanical(s_last_bus_ist_deg);
        }
        (void)rotor_rs485_goto_degrees(bus_tgt);
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
    if (objects.grad_acc) {
        lv_obj_add_event_cb(objects.grad_acc, on_arc, LV_EVENT_ALL, nullptr);
        /* Arc-Knauf (Zeiger-Punkt): rot — nur Laufzeit, keine Änderung in ui/screens.c */
        lv_obj_set_style_bg_color(objects.grad_acc, lv_color_hex(0xff0000), LV_PART_KNOB);
        lv_obj_set_style_bg_opa(objects.grad_acc, LV_OPA_COVER, LV_PART_KNOB);
    }
    s_pwm_ui_is_fast = pwm_config_get_pwm_ui_fast() != 0;
    pwm_style_slow_fast(s_pwm_ui_is_fast);
    if (objects.slow) {
        lv_obj_add_event_cb(objects.slow, on_ui_button_press_beep, LV_EVENT_PRESSED, nullptr);
        lv_obj_add_event_cb(objects.slow, on_slow_btn, LV_EVENT_CLICKED, nullptr);
    }
    if (objects.fast) {
        lv_obj_add_event_cb(objects.fast, on_ui_button_press_beep, LV_EVENT_PRESSED, nullptr);
        lv_obj_add_event_cb(objects.fast, on_fast_btn, LV_EVENT_CLICKED, nullptr);
    }
    if (objects.encoder_delta_bu) {
        lv_obj_add_event_cb(objects.encoder_delta_bu, on_ui_button_press_beep, LV_EVENT_PRESSED, nullptr);
        lv_obj_add_event_cb(objects.encoder_delta_bu, on_encoder_delta_btn, LV_EVENT_CLICKED, nullptr);
    }
    encoder_delta_apply_button_label();
    antenna_apply_labels_from_config();
    antenna_apply_style(pwm_config_get_last_antenna());
    if (objects.antenna_1) {
        lv_obj_add_event_cb(objects.antenna_1, on_ui_button_press_beep, LV_EVENT_PRESSED, nullptr);
        lv_obj_add_event_cb(objects.antenna_1, on_antenna_btn, LV_EVENT_CLICKED, nullptr);
    }
    if (objects.antenna_2) {
        lv_obj_add_event_cb(objects.antenna_2, on_ui_button_press_beep, LV_EVENT_PRESSED, nullptr);
        lv_obj_add_event_cb(objects.antenna_2, on_antenna_btn, LV_EVENT_CLICKED, nullptr);
    }
    if (objects.antenna_3) {
        lv_obj_add_event_cb(objects.antenna_3, on_ui_button_press_beep, LV_EVENT_PRESSED, nullptr);
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
    if (objects.rotor_id) {
        lv_obj_add_event_cb(objects.rotor_id, on_id_field_event, LV_EVENT_ALL, nullptr);
    }
    if (objects.controller_id) {
        lv_obj_add_event_cb(objects.controller_id, on_id_field_event, LV_EVENT_ALL, nullptr);
    }
    id_fields_sync_textareas_from_config();
    apply_anemometer_weather_tab_visibility();
    /* GETREF/GETPOSDG nach 2 s Bus-Bereitschaft — siehe rotor_rs485_init / rotor_rs485_loop */
}

/** @return true wenn SETPOSDG gestartet */
static bool encoder_apply_goto(float target_deg)
{
    lvgl_port_lock(-1);
    if (ENCODER_MOVES_ARC && objects.grad_acc) {
        grad_acc_sync_mechanical(s_last_bus_ist_deg);
    }
    lvgl_port_unlock();
    return rotor_rs485_goto_degrees(display_to_bus(target_deg));
}

extern "C" void rotor_app_encoder_step(int delta_tenths)
{
    if (delta_tenths == 0) {
        return;
    }
    if (rotor_error_app_is_fault_locked()) {
        return;
    }

    /* Rotor_Info: Encoder nur Vorschau — Speichern per HW-Taster oder Tastatur (VALUE_CHANGED) */
    if (s_id_field_focus != IdFieldFocus::None) {
        const int sign = (delta_tenths > 0) ? 1 : -1;
        lvgl_port_lock(-1);
        lv_obj_t *const ta =
            (s_id_field_focus == IdFieldFocus::RotorId) ? objects.rotor_id : objects.controller_id;
        if (ta) {
            const bool is_rotor = (s_id_field_focus == IdFieldFocus::RotorId);
            const uint8_t v = id_field_display_or_saved_config(ta, is_rotor);
            int nv = static_cast<int>(v) + sign;
            if (nv < 1) {
                nv = 1;
            }
            if (nv > 254) {
                nv = 254;
            }
            const uint8_t newv = static_cast<uint8_t>(nv);
            id_fields_set_text(ta, newv);
        }
        lvgl_port_unlock();
        return;
    }

    /* Soll-Feld zählt — Arc (grad_acc) ist optional (ENCODER_MOVES_ARC). */
    if (!objects.taget_dg) {
        return;
    }

    lvgl_port_lock(-1);

    /* Session-Start: Zehntel aus taget_dg; sonst letzter Soll (s_encoder_target_deg), nicht Arc — Arc ist nur 1°. */
    if (!s_encoder_adjusting) {
        s_encoder_arc_int_cached = -32768;
        int parsed = 0;
        if (parse_taget_text_to_tenths(&parsed)) {
            s_encoder_tenths = parsed;
        } else {
            const int t = static_cast<int>(
                std::floor(static_cast<double>(s_encoder_target_deg) * 10.0 + 1e-4));
            s_encoder_tenths = wrap_tenths_deg(t);
        }
    }
    s_encoder_adjusting = true;

    s_encoder_tenths = wrap_tenths_deg(s_encoder_tenths + delta_tenths);

    const float deg = static_cast<float>(s_encoder_tenths) / 10.0f;
    if (ENCODER_MOVES_ARC && objects.grad_acc) {
        const float bus_deg = display_to_bus(deg);
        const int arc_int = deg_to_arc_value(bus_deg);
        if (arc_int != s_encoder_arc_int_cached) {
            grad_acc_sync_mechanical(bus_deg);
            s_encoder_arc_int_cached = arc_int;
        }
    }

    char buf[16];
    fmt_taget_from_wrapped_tenths(buf, sizeof(buf), s_encoder_tenths);
    taget_dg_set_display_text(buf, false);

    lvgl_port_unlock();

    s_encoder_target_deg = deg;

    /* PC/Bus: bei jedem neuen taget vor dem verzögerten SETPOSDG (gleiche Buslage wie späteres SETPOSDG) */
    rotor_rs485_send_setposcc_degrees(display_to_bus(deg));

    /* Kein Telegramm während Drehen: Pause neu starten; ausstehenden Bus-Retry verwirft neuer Takt */
    s_encoder_goto_retry_pending = false;
    s_encoder_retry_deadline_ms = 0;
    s_encoder_idle_deadline_ms = millis() + ENCODER_SEND_IDLE_MS;
    s_last_encoder_step_ms = millis();
}

extern "C" bool rotor_app_encoder_id_field_focused(void)
{
    return s_id_field_focus != IdFieldFocus::None;
}

/**
 * Nur GETANTOFF1…3 vom Rotor geladen — keine Antennenumschaltung, kein SETPOS (sonst Bewegung beim Boot).
 */
extern "C" void rotor_app_antenna_offset_changed(void)
{
    s_encoder_adjusting = false;
    s_encoder_goto_retry_pending = false;
    s_encoder_retry_deadline_ms = 0;
    s_encoder_idle_deadline_ms = 0;
    on_position_deg(s_last_bus_ist_deg);
}

/**
 * Nach pwm_config_set_last_antenna(neu): gleiche Kompassrichtung (Strahl) wie mit prev_antenna
 * zur aktuellen Buslage — SETPOSDG mit umgerechneter Mechanik (nur wenn send_bus_goto).
 * Mitläufer-Modus: send_bus_goto false — nur taget/Arc/Offset, kein eigenes SETPOSDG (vom PC kommt SETPOSDG).
 */
static void rotor_app_antenna_switch_from_ui(uint8_t prev_antenna_1_to_3, bool send_bus_goto)
{
    s_encoder_adjusting = false;
    s_encoder_goto_retry_pending = false;
    s_encoder_retry_deadline_ms = 0;
    s_encoder_idle_deadline_ms = 0;

    const uint8_t now_ant = pwm_config_get_last_antenna();
    if (prev_antenna_1_to_3 < 1u || prev_antenna_1_to_3 > 3u || now_ant < 1u || now_ant > 3u) {
        return;
    }

    /* concha 0: Soll = Ist in Anzeige für die neue Antenne (kein Strahl beibehalten). */
    const float beam_compass =
        bus_to_display_for_idx(s_last_bus_ist_deg, static_cast<int>(prev_antenna_1_to_3));
    const float disp_ist_new_ant =
        bus_to_display_for_idx(s_last_bus_ist_deg, static_cast<int>(now_ant));
    const uint8_t concha = pwm_config_get_concha();
    const float soll_display = (concha == 0) ? disp_ist_new_ant : beam_compass;

    s_encoder_target_deg = soll_display;
    s_encoder_tenths = wrap_tenths_deg(
        static_cast<int>(std::floor(static_cast<double>(soll_display) * 10.0 + 1e-4)));

    on_position_deg(s_last_bus_ist_deg);

    if (rotor_error_app_is_fault_locked()) {
        return;
    }
    if (!rotor_rs485_is_referenced()) {
        return;
    }
    if (!objects.taget_dg) {
        return;
    }

    if (concha == 0) {
        s_taget_ignore_bus_target_until_ms = 0;
        lvgl_port_lock(-1);
        char buf[16];
        fmt_taget_from_display_deg(buf, sizeof(buf), soll_display);
        taget_dg_set_display_text(buf);
        lvgl_port_unlock();
        return;
    }

    s_taget_ignore_bus_target_until_ms = millis() + 2000;

    lvgl_port_lock(-1);
    char buf[16];
    fmt_taget_from_wrapped_tenths(buf, sizeof(buf), s_encoder_tenths);
    taget_dg_set_display_text(buf);
    lvgl_port_unlock();

    if (!send_bus_goto) {
        return;
    }
    if (!encoder_apply_goto(beam_compass)) {
        rotor_rs485_hw_snap_retarget_request(
            display_to_bus_for_idx(beam_compass, static_cast<int>(now_ant)));
    }
}

/**
 * Außentemp (ACK_GETTEMPA): immer Tab Rotor_Info (aussen_temperatur); Wetter-Tab (temperature) nur bei anemometer=1.
 * Wind nur bei anemometer=1; Motortemp (Bit 0x8) immer.
 */
extern "C" void rotor_app_weather_ui_poll(void)
{
    const uint8_t m = rotor_rs485_weather_ui_take_mask();
    if (m == 0) {
        return;
    }
    const float w = rotor_rs485_get_last_wind_kmh();
    const float t = rotor_rs485_get_last_tempa_c();
    const float tm = rotor_rs485_get_last_tempm_c();
    const float dir = rotor_rs485_get_last_wind_dir_deg();
    const bool ano = pwm_config_get_anemometer() != 0;
    lvgl_port_lock(-1);
    char buf[24];
    /* EEZ: wind_speed / temperature sind lv_textarea (Wind max. 5 Zeichen, z. B. „123,4“), keine lv_label */
    if (ano && (m & 1u)) {
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
        if (objects.aussen_temperatur) {
            lv_textarea_set_text(objects.aussen_temperatur, buf);
        }
        if (ano && objects.temperature) {
            lv_textarea_set_text(objects.temperature, buf);
        }
    }
    if (ano && (m & 4u)) {
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
            lv_obj_invalidate(objects.pfeil_wind);
        }
    }
    if (m & 8u) {
        float mtd = tm;
        if (mtd > 999.9f) {
            mtd = 999.9f;
        }
        fmt_de(buf, sizeof(buf), mtd);
        if (objects.motor_temperatur) {
            lv_textarea_set_text(objects.motor_temperatur, buf);
        }
    }
    lvgl_port_unlock();
}

extern "C" void rotor_pwm_ui_loop(void)
{
    if (rotor_error_app_is_fault_locked()) {
        return;
    }
    /* Erst nach Boot-GETREF/GETPOS: sonst blockiert SETPWM den ersten GETREF. */
    if (!rotor_rs485_is_boot_done()) {
        return;
    }
    if (s_pwm_boot_send_pending) {
        const uint8_t p = (pwm_config_get_pwm_ui_fast() != 0) ? pwm_config_get_fast() : pwm_config_get_slow();
        if (rotor_rs485_send_set_pwm_limit(p)) {
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
        static_cast<int>(std::floor(static_cast<double>(disp) * 10.0 + 1e-4)));

    lvgl_port_lock(-1);
    char buf[16];
    fmt_taget_from_display_deg(buf, sizeof(buf), disp);
    taget_dg_set_display_text(buf);
    if (ENCODER_MOVES_ARC && objects.grad_acc) {
        grad_acc_sync_mechanical(bus_deg);
    }
    lvgl_port_unlock();
}
