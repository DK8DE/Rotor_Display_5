/**
 * Verbindet EEZ-UI (ref, homing_led, grad_acc, taget_dg, actual_dg) mit rotor_rs485 — ohne Änderungen in src/ui.
 * Arc: Wert = mechanische Buslage (0..360°); Rotation = EEZ-Basis (270°) + Antennenversatz — Anfang = Mechanik-0°.
 * taget/actual: logische Kompassrichtung (Bus + Versatz, bei Dipol ggf. Rückkeule).
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
/** Arc-Drag: SETPOSCC-Vorschau an PC-Master (wie Encoder), max. alle 100 ms */
static uint32_t s_arc_drag_cc_next_ms = 0;
static float s_arc_drag_last_disp_deg = 0.0f;
static bool s_arc_drag_cc_have_deg = false;
/** Bus-Ist beim Arc-Press — shortest-path stabil während Drag. */
static float s_arc_drag_bus_ref_deg = 0.0f;
static constexpr uint32_t ARC_DRAG_SETPOSCC_MS = 100u;
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
/** Bei aktivem Fremd-PC-Verkehr: finalen Encoder-SETPOSDG früher senden, damit Soll->Fahrt enger zusammenliegt. */
static constexpr uint32_t ENCODER_SEND_IDLE_MS_FOREIGN_PC = 240;
/** true: wie Touch am Arc — Drehen zeigt Soll am Arc; vor SETPOSDG Arc auf Ist (Startlage), dann Fahrt */
static constexpr bool ENCODER_MOVES_ARC = true;

/** Tab Rotor_Info: Encoder/Tippen nur Vorschau; Flash nur per HW-Taster und nur bei geänderter Zahl */
enum class IdFieldFocus : uint8_t { None = 0, RotorAz, RotorEl, ControllerId };
static IdFieldFocus s_id_field_focus = IdFieldFocus::None;

enum class AxisMode : uint8_t { Az = 0, El = 1 };
static AxisMode s_axis = AxisMode::Az;

struct AxisCache {
    float bus_ist_deg = 0.0f;
    float target_deg = 0.0f;
    int tenths = 0;
    bool dipole_back = false;
    bool referenced = false;
    bool have = false;
};
static AxisCache s_axis_cache[2];

/** Zielmarker (Definition hier; Init/Refresh weiter unten). */
static lv_obj_t *s_arc_target_marker = nullptr;
static int s_arc_target_int_cached = -32768;
static bool s_arc_target_visible = false;

static bool axis_is_el(void)
{
    return s_axis == AxisMode::El;
}

static float axis_span_deg(void)
{
    return axis_is_el() ? pwm_config_get_el_max_deg() : pwm_config_get_axis_span_deg();
}

static uint8_t axis_slave_id(void)
{
    return axis_is_el() ? pwm_config_get_rotor_el_id() : pwm_config_get_rotor_id();
}

static float clamp_el_deg(float d)
{
    const float el_max = pwm_config_get_el_max_deg();
    if (d < 0.0f) {
        return 0.0f;
    }
    if (d > el_max) {
        return el_max;
    }
    return d;
}

/** Aktiver Ist-Arc: AZ = grad_acc, EL = grad_acc_el (EEZ-Geometrie unverändert). */
static lv_obj_t *active_grad_acc(void)
{
    if (axis_is_el() && objects.grad_acc_el) {
        return objects.grad_acc_el;
    }
    return objects.grad_acc;
}
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

static void on_slow_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    if (rotor_error_app_is_fault_locked()) {
        return;
    }
    touch_feedback_button_click();
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
    touch_feedback_button_click();
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
    const lv_color_t c_disabled = lv_color_hex(0x607d8b);
    const lv_color_t c_txt_on_green = lv_color_hex(0xFFFFFF);
    const lv_color_t c_txt_on_blue = lv_color_hex(0x000000);
    const lv_color_t c_txt_disabled = lv_color_hex(0x90a4ae);
    const bool az_ok = (pwm_config_get_rotor_id() != 0u);
    lv_obj_t *btns[3] = { objects.antenna_1, objects.antenna_2, objects.antenna_3 };
    lv_obj_t *labels[3] = { objects.antenna_1_label, objects.antenna_2_label, objects.antenna_3_label };
    for (uint8_t i = 0; i < 3; i++) {
        if (!btns[i]) {
            continue;
        }
        if (!az_ok) {
            lv_obj_clear_flag(btns[i], LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_state(btns[i], LV_STATE_DISABLED);
            lv_obj_set_style_bg_color(btns[i], c_disabled, LV_PART_MAIN);
            if (labels[i]) {
                lv_obj_set_style_text_color(labels[i], c_txt_disabled, LV_PART_MAIN);
            }
            continue;
        }
        lv_obj_clear_state(btns[i], LV_STATE_DISABLED);
        lv_obj_add_flag(btns[i], LV_OBJ_FLAG_CLICKABLE);
        const bool on = (active_1_to_3 == (uint8_t)(i + 1));
        lv_obj_set_style_bg_color(btns[i], on ? c_on : c_off, LV_PART_MAIN);
        if (labels[i]) {
            lv_obj_set_style_text_color(labels[i], on ? c_txt_on_green : c_txt_on_blue, LV_PART_MAIN);
        }
    }
}

extern "C" void rotor_app_apply_remote_antenna_selection_deferred(uint8_t prev_1_to_3, uint8_t n_1_to_3)
{
    if (n_1_to_3 < 1u || n_1_to_3 > 3u || prev_1_to_3 < 1u || prev_1_to_3 > 3u) {
        return;
    }
    if (prev_1_to_3 == n_1_to_3) {
        return;
    }
    /* Ohne AZ: Antennenwahl deaktiviert — Bus-Telegramme ignorieren. */
    if (pwm_config_get_rotor_id() == 0u) {
        return;
    }
    pwm_config_set_last_antenna(n_1_to_3);
    pwm_config_save();
    /* Kein erneutes SETASELECT — kommt vom AZ/Fremd-Master; Display nur UI/Cache. */
    lvgl_port_lock(-1);
    antenna_apply_style(n_1_to_3);
    /* Antenne nur AZ; auf EL kein SETPOSDG an die aktive EL-ID. */
    if (!axis_is_el()) {
        rotor_app_antenna_switch_from_ui(prev_1_to_3, !rotor_rs485_is_foreign_pc_listen_mode());
    }
    lvgl_port_unlock();
}

static void on_antenna_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    if (pwm_config_get_rotor_id() == 0u) {
        return;
    }
    if (rotor_error_app_is_fault_locked()) {
        return;
    }
    touch_feedback_button_click();
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
    /* Antennenwahl gilt nur für AZ — auf EL kein SETPOSDG / keine EL-Soll-Umrechnung. */
    if (axis_is_el()) {
        return;
    }
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

/** Beschriftung Encoder-Schrittweite auf dem ehemaligen Homing-/Delta-Button (ref_label). */
static void encoder_delta_apply_button_label(void)
{
    if (!objects.ref_label) {
        return;
    }
    const uint8_t t = pwm_config_get_encoder_delta_tenths();
    lv_label_set_text(objects.ref_label, (t == 1u) ? "0,1 Deg" : "1 Deg");
    lv_obj_set_style_text_color(objects.ref_label, lv_color_hex(0x000000), LV_PART_MAIN);
}

/** UI-Button: Encoder-Schritt 0,1° ↔ 1° (wirkt für die aktive Achse). */
static void on_encoder_delta_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    if (rotor_error_app_is_fault_locked()) {
        return;
    }
    touch_feedback_button_click();
    const uint8_t cur = pwm_config_get_encoder_delta_tenths();
    const uint8_t nv = (cur == 1u) ? 10u : 1u;
    pwm_config_set_encoder_delta_tenths(nv);
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
    id_fields_set_text(objects.rotor_az, pwm_config_get_rotor_id());
    id_fields_set_text(objects.rotor_el, pwm_config_get_rotor_el_id());
    id_fields_set_text(objects.controller_id, pwm_config_get_master_id());
}

/** Nur parsen; min_id…254 (EL: min_id=0 = Achse aus) */
static bool id_field_parse_ta_id(lv_obj_t *ta, uint8_t *out, uint8_t min_id)
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
    if (v < static_cast<long>(min_id) || v > 254L) {
        return false;
    }
    *out = static_cast<uint8_t>(v);
    return true;
}

static void apply_axis_background(void);
static void apply_axis_arc_geometry(void);
static void apply_axis_ui_after_switch(void);
static void axis_cache_store_current(void);
static void axis_cache_restore(AxisMode mode);

/** Nach Commit EL=0 → AZ bzw. AZ=0 → EL: UI umschalten (ohne erneutes LVGL-Lock). */
static bool s_pending_force_axis_az_ui = false;
static bool s_pending_force_axis_el_ui = false;

/**
 * Aktive Achse hat ID 0 und die andere ist ungleich 0 → auf die andere Achse wechseln.
 * Sonst Slave-ID der aktiven Achse (falls ungleich 0) übernehmen.
 * @return true wenn Achse gewechselt wurde.
 */
static bool apply_axis_fallback_for_zero_id(void)
{
    if (!axis_is_el()) {
        const uint8_t az = pwm_config_get_rotor_id();
        const uint8_t el = pwm_config_get_rotor_el_id();
        if (az == 0u && el != 0u) {
            axis_cache_store_current();
            s_axis = AxisMode::El;
            rotor_rs485_set_slave_id(el);
            axis_cache_restore(AxisMode::El);
            return true;
        }
        if (az != 0u) {
            rotor_rs485_set_slave_id(az);
        }
        return false;
    }
    const uint8_t el = pwm_config_get_rotor_el_id();
    const uint8_t az = pwm_config_get_rotor_id();
    if (el == 0u && az != 0u) {
        axis_cache_store_current();
        s_axis = AxisMode::Az;
        rotor_rs485_set_slave_id(az);
        axis_cache_restore(AxisMode::Az);
        return true;
    }
    if (el != 0u) {
        rotor_rs485_set_slave_id(el);
    }
    return false;
}

/**
 * Nur beim HW-Taster: gültige Zahl → bei Abweichung von der Config RS485 + pwm_config_save();
 * gleicher Wert → nur Text normalisieren, kein Flash-Schreiben.
 * kind: 0=AZ (0 = Achse aus), 1=EL (0 = Achse aus), 2=Master
 */
static bool id_field_try_commit_text(lv_obj_t *ta, uint8_t kind)
{
    if (!ta) {
        return false;
    }
    uint8_t nv = 0;
    const uint8_t min_id = (kind == 2) ? 1u : 0u;
    if (!id_field_parse_ta_id(ta, &nv, min_id)) {
        return false;
    }
    uint8_t cur = 0;
    if (kind == 0) {
        cur = pwm_config_get_rotor_id();
    } else if (kind == 1) {
        cur = pwm_config_get_rotor_el_id();
    } else {
        cur = pwm_config_get_master_id();
    }
    if (nv != cur) {
        if (kind == 0) {
            pwm_config_set_rotor_id(nv);
        } else if (kind == 1) {
            pwm_config_set_rotor_el_id(nv);
        } else {
            pwm_config_set_master_id(nv);
            rotor_rs485_set_master_id(nv);
        }
        if (kind != 2) {
            if (apply_axis_fallback_for_zero_id()) {
                if (axis_is_el()) {
                    s_pending_force_axis_el_ui = true;
                } else {
                    s_pending_force_axis_az_ui = true;
                }
            }
        }
        pwm_config_save();
    }
    id_fields_set_text(ta, nv);
    return true;
}

static uint8_t id_field_display_or_saved_config(lv_obj_t *ta, uint8_t kind)
{
    uint8_t v = 0;
    const uint8_t min_id = (kind == 1) ? 0u : 1u;
    if (id_field_parse_ta_id(ta, &v, min_id)) {
        return v;
    }
    if (kind == 0) {
        return pwm_config_get_rotor_id();
    }
    if (kind == 1) {
        return pwm_config_get_rotor_el_id();
    }
    return pwm_config_get_master_id();
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
    uint8_t kind = 2;
    lv_obj_t *ta = objects.controller_id;
    if (s_id_field_focus == IdFieldFocus::RotorAz) {
        kind = 0;
        ta = objects.rotor_az;
    } else if (s_id_field_focus == IdFieldFocus::RotorEl) {
        kind = 1;
        ta = objects.rotor_el;
    }
    if (!ta) {
        s_id_field_focus = IdFieldFocus::None;
        lvgl_port_unlock();
        return false;
    }
    if (!id_field_try_commit_text(ta, kind)) {
        id_fields_set_text(ta, id_field_display_or_saved_config(ta, kind));
    }
    bool need_getref = false;
    if (s_pending_force_axis_az_ui || s_pending_force_axis_el_ui) {
        s_pending_force_axis_az_ui = false;
        s_pending_force_axis_el_ui = false;
        apply_axis_background();
        apply_axis_arc_geometry();
        apply_axis_ui_after_switch();
        need_getref = true;
    }
    antenna_apply_style(pwm_config_get_last_antenna());
    id_field_blur(ta);
    s_id_field_focus = IdFieldFocus::None;
    lvgl_port_unlock();
    if (need_getref) {
        rotor_rs485_send_getref();
    }
    return true;
}

static void on_id_field_event(lv_event_t *e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_FOCUSED && code != LV_EVENT_DEFOCUSED && code != LV_EVENT_VALUE_CHANGED) {
        return;
    }
    lv_obj_t *ta = lv_event_get_target(e);
    uint8_t kind = 2;
    IdFieldFocus focus = IdFieldFocus::ControllerId;
    if (ta == objects.rotor_az) {
        kind = 0;
        focus = IdFieldFocus::RotorAz;
    } else if (ta == objects.rotor_el) {
        kind = 1;
        focus = IdFieldFocus::RotorEl;
    }

    if (code == LV_EVENT_FOCUSED) {
        s_id_field_focus = focus;
        id_fields_set_text(ta, id_field_display_or_saved_config(ta, kind));
        s_encoder_adjusting = false;
        s_encoder_goto_retry_pending = false;
        s_encoder_retry_deadline_ms = 0;
        s_encoder_idle_deadline_ms = 0;
        return;
    }
    if (code == LV_EVENT_DEFOCUSED) {
        id_fields_set_text(ta, id_field_display_or_saved_config(ta, kind));
        s_id_field_focus = IdFieldFocus::None;
        return;
    }
    if (code == LV_EVENT_VALUE_CHANGED) {
        if (s_id_field_programmatic_text) {
            return;
        }
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
    const bool axis_switched = apply_axis_fallback_for_zero_id();
    if (axis_switched) {
        apply_axis_background();
        apply_axis_arc_geometry();
        apply_axis_ui_after_switch();
    }
    id_fields_sync_textareas_from_config();
    apply_anemometer_weather_tab_visibility();
    encoder_delta_apply_button_label();
    const uint8_t p = s_pwm_ui_is_fast ? pwm_config_get_fast() : pwm_config_get_slow();
    if (!rotor_rs485_send_set_pwm_limit(p)) {
        s_pwm_deferred = p;
    }
    lvgl_port_unlock();
    if (axis_switched) {
        rotor_rs485_send_getref();
        if (axis_is_el()) {
            rotor_app_el_limits_changed();
            rotor_rs485_request_el_rotor_type();
        }
    }
}

static int wrap_tenths_deg(int t)
{
    if (axis_is_el()) {
        const int el_max_t =
            static_cast<int>(std::lround(static_cast<double>(pwm_config_get_el_max_deg()) * 10.0));
        if (t < 0) {
            return 0;
        }
        if (t > el_max_t) {
            return el_max_t;
        }
        return t;
    }
    const int span_t = static_cast<int>(std::lround(static_cast<double>(axis_span_deg()) * 10.0));
    const int mod = (span_t > 0) ? span_t : 3600;
    t %= mod;
    if (t < 0) {
        t += mod;
    }
    return t;
}

/** Grad -> Zehntelgrad robust runden (kein floor-Bias bei 0,1°-Richtungswechseln). */
static int deg_to_tenths_rounded(float deg)
{
    const int t = static_cast<int>(std::lround(static_cast<double>(deg) * 10.0));
    return wrap_tenths_deg(t);
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
    const int hi_limit = static_cast<int>(std::lround(
        static_cast<double>(axis_span_deg()) * 10.0)) + 100;
    while (*p >= '0' && *p <= '9') {
        any_digit = true;
        hi = hi * 10 + (*p - '0');
        if (hi > hi_limit) {
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

/** Arc-Wert: AZ 0…360; EL 0…el_max (Arc-Wert = Elevation). */
static int deg_to_arc_value(float deg)
{
    if (axis_is_el()) {
        const int el_max = static_cast<int>(pwm_config_get_el_max_deg() + 0.5f);
        float d = clamp_el_deg(deg);
        int v = static_cast<int>(d + 0.5f);
        if (v < 0) {
            v = 0;
        }
        if (v > el_max) {
            v = el_max;
        }
        return v;
    }
    float folded = fmodf(deg, 360.0f);
    if (folded < 0.0f) {
        folded += 360.0f;
    }
    if (folded >= 359.5f) {
        return 360;
    }
    int v = static_cast<int>(folded + 0.5f);
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
/** Dipol: aktuelle Fahrt nutzt Rückkeule (mechanisch +180° zur logischen Strahlrichtung). */
static bool s_dipole_back_lobe_active = false;

static float norm360_add(float a)
{
    float x = fmodf(a, 360.0f);
    if (x < 0.0f) {
        x += 360.0f;
    }
    return x;
}

/** Addieren im wirksamen Fahrbereich (360 oder Typ-3-Span). */
static float norm_span_add(float a)
{
    const float span = pwm_config_get_axis_span_deg();
    if (span <= 360.5f) {
        return norm360_add(a);
    }
    float x = fmodf(a, span);
    if (x < 0.0f) {
        x += span;
    }
    return x;
}

/** Kompass = Buslage + Versatz der angegebenen Antenne (1…3) — beim Wechsel: Strahl mit alter Antenne. */
static float bus_to_display_for_idx(float bus_deg_ui, int ant_1_to_3)
{
    if (axis_is_el()) {
        return clamp_el_deg(bus_deg_ui);
    }
    const float off = pwm_config_get_antoff_deg(ant_1_to_3);
    const float span = pwm_config_get_axis_span_deg();
    /* Eng um 360,0 (±0,05°) halten, NICHT ab 359,5° (das ist die 1°-Arc-Schwelle) — sonst wird jede
     * echte Buslage von 359,5…359,99° hier faelschlich als exakt 360° behandelt (Anzeige springt vor
     * der Ankunft schon auf 360, obwohl der Rotor z. B. noch bei 359,6° steht). */
    if (span <= 360.5f && bus_deg_ui >= 359.95f) {
        if (std::fabs(static_cast<double>(off)) < 1e-6) {
            return 360.0f;
        }
        return norm_span_add(360.0f + off);
    }
    return norm_span_add(bus_deg_ui + off);
}

/** Anzeige (Kompass / Antennenrichtung) = Buslage + Versatz der aktuell gewählten Antenne */
static float bus_to_display(float bus_deg_ui)
{
    return bus_to_display_for_idx(bus_deg_ui, static_cast<int>(pwm_config_get_last_antenna()));
}

/** Soll am Bus für SETPOSDG aus Anzeige-Winkel und Antenne ant_1_to_3 */
static float display_to_bus_for_idx(float display_deg, int ant_1_to_3)
{
    if (axis_is_el()) {
        return clamp_el_deg(display_deg);
    }
    const float off = pwm_config_get_antoff_deg(ant_1_to_3);
    const float span = pwm_config_get_axis_span_deg();
    /* Eng um 360,0 (±0,05°) halten, NICHT ab 359,5° (das ist die 1°-Arc-Schwelle) — sonst wird ein
     * eingegebenes Ziel von 359,5…359,9° hier faelschlich als exakt 360° umgerechnet und der Rotor
     * faehrt/zeigt 360° statt des tatsaechlich gewuenschten Winkels an. */
    if (span <= 360.5f && display_deg >= 359.95f) {
        if (std::fabs(static_cast<double>(off)) < 1e-6f) {
            return 360.0f;
        }
        return norm_span_add(360.0f - off);
    }
    return norm_span_add(display_deg - off);
}

/** Soll am Bus für SETPOSDG aus Anzeige-Winkel (aktuelle last_antenna) */
static float display_to_bus(float display_deg)
{
    return display_to_bus_for_idx(display_deg, static_cast<int>(pwm_config_get_last_antenna()));
}

static float min_angle_diff_display(float a_deg, float b_deg)
{
    if (axis_is_el()) {
        return std::fabs(a_deg - b_deg);
    }
    const float span = pwm_config_get_axis_span_deg();
    if (span > 360.5f) {
        /* Erweiterter Span: lineare Distanz (0 und 360 sind verschiedene Lagen). */
        return std::fabs(a_deg - b_deg);
    }
    float d = std::fabs(a_deg - b_deg);
    if (d > 180.0f) {
        d = 360.0f - d;
    }
    return d;
}

struct MotionBusResolve {
    float bus_cmd;
    bool use_back_lobe;
};

/** Logisches Soll → mechanischer Bus-Soll (Haupt- oder Rückkeule bei Dipol).
 * Der Rotor hat einen mechanischen Anschlag am Bus-0/360-Übergang und kann nicht darüber drehen.
 * Die echte Fahrstrecke ist daher die LINEARE Bus-Distanz |Soll−Ist| (kein Wraparound) — sonst
 * würde z. B. „16° über den Anschlag“ gewählt, real aber 344° (volle Drehung) statt der halben
 * Drehung über die Rückkeule. */
static MotionBusResolve resolve_motion_bus(float logical_deg, float current_bus, int ant_1_to_3)
{
    if (axis_is_el()) {
        return {clamp_el_deg(logical_deg), false};
    }
    const float bus_main = display_to_bus_for_idx(logical_deg, ant_1_to_3);
    if (!pwm_config_get_antdp(ant_1_to_3)) {
        return {bus_main, false};
    }
    float logical_back = logical_deg + 180.0f;
    if (logical_back >= pwm_config_get_axis_span_deg()) {
        logical_back = norm_span_add(logical_back);
    }
    const float bus_back = display_to_bus_for_idx(logical_back, ant_1_to_3);
    const float d_main = std::fabs(current_bus - bus_main);
    const float d_back = std::fabs(current_bus - bus_back);
    if (d_back < d_main) {
        return {bus_back, true};
    }
    return {bus_main, false};
}

/** Mechanisches Ist → logische Anzeige (Kompass / taget / Arc). */
static float bus_to_logical_display(float bus_mech, int ant_1_to_3)
{
    if (axis_is_el()) {
        return clamp_el_deg(bus_mech);
    }
    if (!pwm_config_get_antdp(ant_1_to_3)) {
        return bus_to_display_for_idx(bus_mech, ant_1_to_3);
    }
    if (s_dipole_back_lobe_active) {
        return bus_to_display_for_idx(norm_span_add(bus_mech + 180.0f), ant_1_to_3);
    }
    const float logical_main = bus_to_display_for_idx(bus_mech, ant_1_to_3);
    const float logical_back = bus_to_display_for_idx(norm_span_add(bus_mech + 180.0f), ant_1_to_3);
    if (s_encoder_adjusting || rotor_rs485_is_position_polling()) {
        const float target = s_encoder_target_deg;
        const float d_main = min_angle_diff_display(logical_main, target);
        const float d_back = min_angle_diff_display(logical_back, target);
        return (d_back < d_main) ? logical_back : logical_main;
    }
    return logical_main;
}

float rotor_app_get_display_direction_deg(void)
{
    if (axis_is_el()) {
        return clamp_el_deg(s_last_bus_ist_deg);
    }
    return bus_to_display_for_idx(s_last_bus_ist_deg,
                                  static_cast<int>(pwm_config_get_last_antenna()));
}

float rotor_app_get_display_target_deg(void)
{
    return s_encoder_target_deg;
}

extern "C" uint8_t rotor_app_get_axis(void)
{
    return static_cast<uint8_t>(s_axis);
}

static int grad_acc_rotation_from_antoff(int ant_1_to_3)
{
    const float off = pwm_config_get_antoff_deg(ant_1_to_3);
    long r = static_cast<long>(GRAD_ACC_BASE_ROTATION)
        + std::lround(static_cast<double>(off));
    r %= 360;
    if (r < 0) {
        r += 360;
    }
    return static_cast<int>(r);
}

/** lv_arc_get_value → Busgrad auf der gedrehten Arc-Skala (ohne Span-Zonenwahl). */
static float arc_int_value_to_bus_deg(int v)
{
    if (axis_is_el()) {
        const int el_max = static_cast<int>(pwm_config_get_el_max_deg() + 0.5f);
        if (v < 0) {
            v = 0;
        }
        if (v > el_max) {
            v = el_max;
        }
        return clamp_el_deg(static_cast<float>(v));
    }
    if (v >= 360) {
        return 360.0f;
    }
    return static_cast<float>(v);
}

/** Arc-Wert → Busgrad; bei Span > 360 die Zone mit kürzerer LINEARER Fahrstrecke. */
static float arc_value_to_bus_shortest(int v, float current_bus)
{
    if (axis_is_el()) {
        return arc_int_value_to_bus_deg(v);
    }
    float base = arc_int_value_to_bus_deg(v);
    const float span = pwm_config_get_axis_span_deg();
    if (span <= 360.5f) {
        return base;
    }
    float best = base;
    float best_d = std::fabs(current_bus - base);
    const float alt = base + 360.0f;
    if (alt <= span + 0.05f) {
        const float d = std::fabs(current_bus - alt);
        if (d < best_d) {
            best = alt;
            best_d = d;
        }
    }
    /* Arc 360: auch Span-Endlage (z. B. 420) als Kandidat, falls näher */
    if (v >= 360) {
        const float d_span = std::fabs(current_bus - span);
        if (d_span < best_d) {
            best = span;
        }
    }
    return best;
}

/** Arc: mechanische Lage; bei Dipol-Rückkeule Arc-Wert +180° (logische Strahlrichtung am Zeiger).
 * Knauf-Farbe: Typ 3 → grün in der 1. Umdrehung (≤360°), rot darüber; sonst immer rot.
 * EL: EEZ-Arc grad_acc_el (Geometrie/Mode unangetastet). */
static void grad_acc_sync_bus(float bus_mech_deg, int ant_1_to_3, bool dipole_back_lobe)
{
    lv_obj_t *const arc = active_grad_acc();
    if (!arc) {
        return;
    }
    float arc_bus = bus_mech_deg;
    if (!axis_is_el() && pwm_config_get_antdp(ant_1_to_3) && dipole_back_lobe) {
        arc_bus = norm_span_add(bus_mech_deg + 180.0f);
    }
    if (axis_is_el()) {
        arc_bus = clamp_el_deg(bus_mech_deg);
    }
    s_arc_updating = true;
    lv_arc_set_value(arc, deg_to_arc_value(arc_bus));
    if (!axis_is_el()) {
        lv_arc_set_rotation(arc, grad_acc_rotation_from_antoff(ant_1_to_3));
    }
    {
        uint32_t knob = 0xff0000u;
        if (!axis_is_el() && pwm_config_get_enc_type() == 3u) {
            knob = (bus_mech_deg > 360.0f) ? 0xff0000u : 0x43b302u;
        }
        lv_obj_set_style_bg_color(arc, lv_color_hex(knob), LV_PART_KNOB);
    }
    s_arc_updating = false;
}

/** AZ/EL-Arcs und Zielmarker: nur Sichtbarkeit; EEZ-Geometrie von grad_acc / grad_acc_el bleibt. */
static void apply_axis_arc_geometry(void)
{
    if (objects.grad_acc) {
        if (axis_is_el()) {
            lv_obj_add_flag(objects.grad_acc, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(objects.grad_acc, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (objects.grad_acc_el) {
        if (axis_is_el()) {
            lv_obj_clear_flag(objects.grad_acc_el, LV_OBJ_FLAG_HIDDEN);
            /* NORMAL: Knauf folgt 0…el_max (Arc-Wert = Elevation). */
            const int el_max = static_cast<int>(pwm_config_get_el_max_deg() + 0.5f);
            lv_arc_set_mode(objects.grad_acc_el, LV_ARC_MODE_NORMAL);
            lv_arc_set_range(objects.grad_acc_el, 0, el_max);
            lv_arc_set_bg_start_angle(objects.grad_acc_el, 0);
            lv_arc_set_bg_end_angle(objects.grad_acc_el, el_max);
        } else {
            lv_obj_add_flag(objects.grad_acc_el, LV_OBJ_FLAG_HIDDEN);
        }
    }
    /* Zielmarker an aktiven Arc anpassen (Größe/Winkel wie EEZ). */
    lv_obj_t *const arc = active_grad_acc();
    if (s_arc_target_marker && arc) {
        lv_obj_set_pos(s_arc_target_marker, lv_obj_get_x(arc) + 5, lv_obj_get_y(arc) + 5);
        lv_obj_set_size(s_arc_target_marker,
                        lv_obj_get_width(arc) - 10,
                        lv_obj_get_height(arc) - 10);
        lv_arc_set_range(s_arc_target_marker, lv_arc_get_min_value(arc), lv_arc_get_max_value(arc));
        lv_arc_set_bg_start_angle(s_arc_target_marker, lv_arc_get_bg_angle_start(arc));
        lv_arc_set_bg_end_angle(s_arc_target_marker, lv_arc_get_bg_angle_end(arc));
        lv_arc_set_mode(s_arc_target_marker, lv_arc_get_mode(arc));
        /* Rotation: EL unverändert aus EEZ; AZ = Basis + Antoff */
        if (axis_is_el()) {
            lv_arc_set_rotation(s_arc_target_marker, 180); /* wie screens.c grad_acc_el */
        } else {
            lv_arc_set_rotation(s_arc_target_marker,
                                grad_acc_rotation_from_antoff(
                                    static_cast<int>(pwm_config_get_last_antenna())));
        }
        const uint32_t idx = lv_obj_get_index(arc);
        lv_obj_move_to_index(s_arc_target_marker, idx + 1);
    }
    s_arc_target_int_cached = -32768;
    s_encoder_arc_int_cached = -32768;
}

static void apply_axis_background(void)
{
    if (objects.kompass_bg) {
        if (axis_is_el()) {
            lv_obj_add_flag(objects.kompass_bg, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(objects.kompass_bg, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (objects.kompass_el) {
        if (axis_is_el()) {
            lv_obj_clear_flag(objects.kompass_el, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(objects.kompass_el, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void axis_cache_store_current(void)
{
    const unsigned i = static_cast<unsigned>(s_axis);
    s_axis_cache[i].bus_ist_deg = s_last_bus_ist_deg;
    s_axis_cache[i].target_deg = s_encoder_target_deg;
    s_axis_cache[i].tenths = s_encoder_tenths;
    s_axis_cache[i].dipole_back = s_dipole_back_lobe_active;
    s_axis_cache[i].referenced = rotor_rs485_is_referenced();
    s_axis_cache[i].have = true;
}

static void axis_cache_restore(AxisMode mode)
{
    const unsigned i = static_cast<unsigned>(mode);
    if (!s_axis_cache[i].have) {
        s_last_bus_ist_deg = 0.0f;
        s_encoder_target_deg = 0.0f;
        s_encoder_tenths = 0;
        s_dipole_back_lobe_active = false;
        /* Achse noch nie gelesen: bis zum GETREF als nicht referenziert behandeln. */
        rotor_rs485_seed_referenced(false);
        return;
    }
    s_last_bus_ist_deg = s_axis_cache[i].bus_ist_deg;
    s_encoder_target_deg = s_axis_cache[i].target_deg;
    s_encoder_tenths = s_axis_cache[i].tenths;
    s_dipole_back_lobe_active = s_axis_cache[i].dipole_back;
    /* Referenz der neuen Achse sofort übernehmen — sonst zeigt der Meldetext bis zum
     * bestätigenden GETREF „Nicht referenziert“, obwohl die Achse referenziert ist. */
    rotor_rs485_seed_referenced(s_axis_cache[i].referenced);
}

static void fmt_de(char *buf, size_t n, float deg);
static void fmt_taget_from_display_deg(char *buf, size_t n, float deg);
static void taget_dg_set_display_text(const char *buf, bool sync_full_refr_now = true);
static void actual_dg_set_display_text(const char *buf, bool sync_full_refr_now = false);
static void arc_target_marker_refresh(void);

/** UI nach Achsenwechsel: Ist/Soll-Felder, Arc, Homing-LED aus Cache. */
static void apply_axis_ui_after_switch(void)
{
    const int ant = static_cast<int>(pwm_config_get_last_antenna());
    const unsigned i = static_cast<unsigned>(s_axis);
    const bool ref = s_axis_cache[i].have ? s_axis_cache[i].referenced
                                          : rotor_rs485_is_referenced();

    if (objects.homing_led) {
        const int err_led = rotor_error_app_get_error_code();
        if ((err_led != 0 && err_led != 10) || !rotor_rs485_is_startup_error_checked()) {
            lv_led_set_color(objects.homing_led, lv_color_hex(0xff0000));
        } else {
            lv_led_set_color(objects.homing_led,
                             ref ? lv_color_hex(0x43b302) : lv_color_hex(0xff0000));
        }
        lv_led_set_brightness(objects.homing_led, 255);
    }
    if (objects.grad_acc) {
        if (ref) {
            lv_obj_add_flag(objects.grad_acc, LV_OBJ_FLAG_CLICKABLE);
        } else {
            lv_obj_clear_flag(objects.grad_acc, LV_OBJ_FLAG_CLICKABLE);
        }
    }
    if (objects.grad_acc_el) {
        if (ref) {
            lv_obj_add_flag(objects.grad_acc_el, LV_OBJ_FLAG_CLICKABLE);
        } else {
            lv_obj_clear_flag(objects.grad_acc_el, LV_OBJ_FLAG_CLICKABLE);
        }
    }

    char buf[16];
    if (ref) {
        const float disp = bus_to_logical_display(s_last_bus_ist_deg, ant);
        fmt_de(buf, sizeof(buf), disp);
        if (objects.actual_dg) {
            actual_dg_set_display_text(buf, false);
        }
        fmt_taget_from_display_deg(buf, sizeof(buf), s_encoder_target_deg);
        taget_dg_set_display_text(buf, false);
        if (active_grad_acc()) {
            grad_acc_sync_bus(s_last_bus_ist_deg, ant, s_dipole_back_lobe_active);
        }
    } else if (objects.actual_dg) {
        actual_dg_set_display_text("-", false);
    }
    arc_target_marker_refresh();
}

extern "C" void rotor_app_toggle_axis(void)
{
    if (rotor_error_app_is_fault_locked()) {
        return;
    }
    const AxisMode next = (s_axis == AxisMode::Az) ? AxisMode::El : AxisMode::Az;
    const uint8_t next_id = (next == AxisMode::El) ? pwm_config_get_rotor_el_id()
                                                   : pwm_config_get_rotor_id();
    /* ID 0 = Achse aus — Umschalten nicht möglich */
    if (next_id == 0) {
        return;
    }

    s_encoder_adjusting = false;
    s_encoder_goto_retry_pending = false;
    s_encoder_retry_deadline_ms = 0;
    s_encoder_idle_deadline_ms = 0;
    s_arc_dragging = false;

    axis_cache_store_current();
    s_axis = next;
    rotor_rs485_set_slave_id(axis_slave_id());
    axis_cache_restore(s_axis);

    lvgl_port_lock(-1);
    apply_axis_background();
    apply_axis_arc_geometry();
    apply_axis_ui_after_switch();
    lvgl_port_unlock();

    rotor_rs485_send_getref();
    /* EL: Typ 2/3 (0…90 vs 0…180) nachziehen — Encoder/Arc hart begrenzen. */
    if (s_axis == AxisMode::El) {
        rotor_app_el_limits_changed();
        rotor_rs485_request_el_rotor_type();
    }
}

extern "C" void rotor_app_seed_axis_cache(uint8_t axis, float bus_ist_deg, bool referenced)
{
    if (axis > 1u) {
        return;
    }
    const float ist = (axis == 1u) ? clamp_el_deg(bus_ist_deg) : bus_ist_deg;
    s_axis_cache[axis].bus_ist_deg = ist;
    if (!s_axis_cache[axis].have) {
        s_axis_cache[axis].target_deg = ist;
        s_axis_cache[axis].tenths = deg_to_tenths_rounded(ist);
        s_axis_cache[axis].dipole_back = false;
    }
    s_axis_cache[axis].referenced = referenced;
    s_axis_cache[axis].have = true;
}

extern "C" void rotor_app_seed_axis_target_bus(uint8_t axis, float target_bus_deg)
{
    if (axis > 1u) {
        return;
    }
    const int ant = static_cast<int>(pwm_config_get_last_antenna());
    const float disp = (axis == 1u) ? clamp_el_deg(target_bus_deg)
                                    : bus_to_logical_display(target_bus_deg, ant);
    s_axis_cache[axis].target_deg = disp;
    s_axis_cache[axis].tenths = deg_to_tenths_rounded(disp);
    if (axis == 1u) {
        s_axis_cache[axis].dipole_back = false;
    }
    s_axis_cache[axis].have = true;
}

extern "C" void rotor_app_seed_axis_referenced(uint8_t axis, bool referenced)
{
    if (axis > 1u) {
        return;
    }
    s_axis_cache[axis].referenced = referenced;
    s_axis_cache[axis].have = true;
}

extern "C" void rotor_app_el_limits_changed(void)
{
    const float el_max = pwm_config_get_el_max_deg();
    /* EL-Cache immer an neues Limit klemmen (auch wenn gerade AZ aktiv). */
    if (s_axis_cache[1].have) {
        if (s_axis_cache[1].bus_ist_deg > el_max) {
            s_axis_cache[1].bus_ist_deg = el_max;
        }
        if (s_axis_cache[1].target_deg > el_max) {
            s_axis_cache[1].target_deg = el_max;
        }
        const int el_max_t = static_cast<int>(std::lround(static_cast<double>(el_max) * 10.0));
        if (s_axis_cache[1].tenths > el_max_t) {
            s_axis_cache[1].tenths = el_max_t;
        }
    }
    if (!axis_is_el()) {
        return;
    }
    lvgl_port_lock(-1);
    apply_axis_arc_geometry();
    s_last_bus_ist_deg = clamp_el_deg(s_last_bus_ist_deg);
    s_encoder_target_deg = clamp_el_deg(s_encoder_target_deg);
    s_encoder_tenths = wrap_tenths_deg(s_encoder_tenths);
    apply_axis_ui_after_switch();
    lvgl_port_unlock();
}

#ifndef ARC_TARGET_SHOW_EPS_DEG
#define ARC_TARGET_SHOW_EPS_DEG 0.8f
#endif

static void arc_target_marker_init(void)
{
    if (s_arc_target_marker || !objects.grad_acc) {
        return;
    }
    lv_obj_t *const parent = lv_obj_get_parent(objects.grad_acc);
    if (!parent) {
        return;
    }
    s_arc_target_marker = lv_arc_create(parent);
    /* 5 px weiter innen: Arc 10 px kleiner und um 5 px eingerückt */
    lv_obj_set_pos(s_arc_target_marker,
                   lv_obj_get_x(objects.grad_acc) + 5,
                   lv_obj_get_y(objects.grad_acc) + 5);
    lv_obj_set_size(s_arc_target_marker,
                    lv_obj_get_width(objects.grad_acc) - 10,
                    lv_obj_get_height(objects.grad_acc) - 10);
    lv_arc_set_range(s_arc_target_marker, 0, 360);
    lv_arc_set_bg_start_angle(s_arc_target_marker, 0);
    lv_arc_set_bg_end_angle(s_arc_target_marker, 360);
    lv_arc_set_rotation(s_arc_target_marker,
                        grad_acc_rotation_from_antoff(static_cast<int>(pwm_config_get_last_antenna())));
    lv_arc_set_value(s_arc_target_marker, 0);
    /* Nur Knauf sichtbar — Spur/Indikator unsichtbar */
    lv_obj_set_style_arc_width(s_arc_target_marker, 0, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_arc_target_marker, 0, LV_PART_INDICATOR);
    lv_obj_set_style_arc_opa(s_arc_target_marker, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(s_arc_target_marker, LV_OPA_TRANSP, LV_PART_INDICATOR);
    /* Rot (Arc hat blauen Streifen); 3 px größer als zuvor (pad 7 → 10) */
    lv_obj_set_style_bg_color(s_arc_target_marker, lv_color_hex(0xff0000), LV_PART_KNOB);
    lv_obj_set_style_bg_opa(s_arc_target_marker, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_pad_all(s_arc_target_marker, 10, LV_PART_KNOB);
    lv_obj_set_style_radius(s_arc_target_marker, LV_RADIUS_CIRCLE, LV_PART_KNOB);
    lv_obj_set_style_border_width(s_arc_target_marker, 0, LV_PART_KNOB);
    lv_obj_clear_flag(s_arc_target_marker, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_arc_target_marker, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_add_flag(s_arc_target_marker, LV_OBJ_FLAG_HIDDEN);
    /* Über grad_acc, unter Hauptanzeige (Zentrum bleibt bedienbar) */
    const uint32_t idx = lv_obj_get_index(objects.grad_acc);
    lv_obj_move_to_index(s_arc_target_marker, idx + 1);
    s_arc_target_int_cached = -32768;
    s_arc_target_visible = false;
}

static void arc_target_marker_refresh(void)
{
    lv_obj_t *const arc = active_grad_acc();
    if (!s_arc_target_marker || !arc) {
        return;
    }
    const int ant = static_cast<int>(pwm_config_get_last_antenna());
    const float ist_disp = bus_to_logical_display(s_last_bus_ist_deg, ant);
    const float d = min_angle_diff_display(ist_disp, s_encoder_target_deg);
    const bool moving = rotor_rs485_is_position_polling() || rotor_rs485_is_remote_setpos_motion();
    /* Während Arc-Drag ist der rote Knauf selbst das Ziel — Blau erst danach / bei Encoder-Vorwahl. */
    const bool show = !s_arc_dragging &&
                      (s_encoder_adjusting || moving || d > ARC_TARGET_SHOW_EPS_DEG);

    if (!show) {
        if (s_arc_target_visible) {
            lv_obj_add_flag(s_arc_target_marker, LV_OBJ_FLAG_HIDDEN);
            s_arc_target_visible = false;
            s_arc_target_int_cached = -32768;
        }
        return;
    }

    const MotionBusResolve r =
        resolve_motion_bus(s_encoder_target_deg, s_last_bus_ist_deg, ant);
    float arc_bus = r.bus_cmd;
    if (!axis_is_el() && pwm_config_get_antdp(ant) && r.use_back_lobe) {
        arc_bus = norm_span_add(r.bus_cmd + 180.0f);
    }
    if (axis_is_el()) {
        arc_bus = clamp_el_deg(arc_bus);
    }
    const int v = deg_to_arc_value(arc_bus);
    /* EL: EEZ-Rotation von grad_acc_el belassen (Marker wurde in apply_axis_arc_geometry synct). */
    if (!axis_is_el()) {
        const int rot = grad_acc_rotation_from_antoff(ant);
        lv_arc_set_rotation(s_arc_target_marker, rot);
    }
    if (!s_arc_target_visible || v != s_arc_target_int_cached) {
        lv_arc_set_value(s_arc_target_marker, v);
        s_arc_target_int_cached = v;
    }
    if (!s_arc_target_visible) {
        lv_obj_clear_flag(s_arc_target_marker, LV_OBJ_FLAG_HIDDEN);
        s_arc_target_visible = true;
    }
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
    if (axis_is_el()) {
        const float d = clamp_el_deg(deg);
        const int t = static_cast<int>(
            std::floor(static_cast<double>(d) * 10.0 + 1e-4));
        fmt_taget_from_wrapped_tenths(buf, n, t);
        return;
    }
    const float span = pwm_config_get_axis_span_deg();
    /* Bei Span 360: Homing-Endlage 360,0 anzeigen (wrap_tenths_deg würde exakt 360,0 sonst auf 0,0
     * zurückfalten). Schwelle eng um 360,0 (±0,05°) halten — nicht wie beim 1°-Arc (deg_to_arc_value)
     * bei 359,5°: sonst zeigt jeder Zehntelwert 359,5…359,9 fälschlich schon 360,0 statt des
     * tatsächlich angefahrenen Winkels. Bei Span>360 (Typ 3) ist 361…span ohnehin normal darstellbar. */
    if (span <= 360.5f && deg >= 359.95f) {
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
static void taget_dg_set_display_text(const char *buf, bool sync_full_refr_now)
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
static void actual_dg_set_display_text(const char *buf, bool sync_full_refr_now)
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
    /* on_ref_status wird nur bei echtem Referenz-Wechsel aufgerufen (notify_ref defert nur bei Änderung).
     * Nach dem Homing (unref→ref) ist keine Dipol-Fahrt aktiv → Rückkeulen-Flag löschen, sonst zeigt
     * die Anzeige die Homing-Position +180° (z. B. 90°→270° bei 90°-Versatz). */
    s_dipole_back_lobe_active = false;
    if (!referenced) {
        s_encoder_adjusting = false;
        s_encoder_goto_retry_pending = false;
        s_encoder_retry_deadline_ms = 0;
        s_encoder_idle_deadline_ms = 0;
        s_encoder_arc_int_cached = -32768;
    }
    lvgl_port_lock(-1);
    if (objects.grad_acc) {
        if (referenced) {
            lv_obj_add_flag(objects.grad_acc, LV_OBJ_FLAG_CLICKABLE);
        } else {
            lv_obj_clear_flag(objects.grad_acc, LV_OBJ_FLAG_CLICKABLE);
        }
    }
    if (objects.grad_acc_el) {
        if (referenced) {
            lv_obj_add_flag(objects.grad_acc_el, LV_OBJ_FLAG_CLICKABLE);
        } else {
            lv_obj_clear_flag(objects.grad_acc_el, LV_OBJ_FLAG_CLICKABLE);
        }
    }
    if (objects.homing_led) {
        /* Harte Fehler / Start-GETERR ausstehend: nicht grün nur wegen GETREF=1 (10 = Timeout, quittierbar). */
        const int err_led = rotor_error_app_get_error_code();
        if ((err_led != 0 && err_led != 10) || !rotor_rs485_is_startup_error_checked()) {
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

static int wrap_tenths_delta(int d)
{
    const int span_t = static_cast<int>(std::lround(
        static_cast<double>(pwm_config_get_axis_span_deg()) * 10.0));
    const int half = (span_t > 0) ? (span_t / 2) : 1800;
    const int full = (span_t > 0) ? span_t : 3600;
    if (d > half) {
        d -= full;
    }
    if (d < -half) {
        d += full;
    }
    return d;
}

/**
 * True, wenn bus_to_display(Soll) in Zehntelgrad nur um typisches Float-Rauschen von der Encoder-Session
 * abweicht (z. B. 353,7499° → floor → 353,7 angezeigt obwohl Soll 353,8). Dann taget_dg nicht überschreiben.
 * Fremd-Master-Fahrt: nie wegfiltern — PC-Soll kann bewusst um 0,1° anders sein.
 */
static bool bus_target_matches_session_tenth_noise(float disp_deg, int enc_tenths)
{
    const int span_t = static_cast<int>(std::lround(
        static_cast<double>(pwm_config_get_axis_span_deg()) * 10.0));
    const double half = (span_t > 0) ? (span_t / 2.0) : 1800.0;
    const double full = (span_t > 0) ? static_cast<double>(span_t) : 3600.0;
    double u = static_cast<double>(disp_deg) * 10.0;
    double e = static_cast<double>(enc_tenths);
    double diff = u - e;
    if (diff > half) {
        diff -= full;
    } else if (diff < -half) {
        diff += full;
    }
    return std::fabs(diff) < 0.5;
}

/** Soll aus Bus (Echo, Ankunft, PC-Loopback) — taget_dg in Anzeige-Koordinaten (mit Versatz) */
static void on_target_deg(float bus_deg)
{
    /* Während Encoder aktiv ist: Bus darf taget_dg NICHT überschreiben.
     * Sonst überschreibt z. B. „Ankunft am alten SETPOS“ (notify_target(s_goto_commanded_deg))
     * einen bereits weiter gedrehten Soll — wirkt wie „1. Klick fehlt“ / falsches Ziel.
     * Auch fremde SETPOSDG-Bewegungen beenden die lokale Encoder-Session hier nicht mehr:
     * der Controller muss nach Encoder-Ruhe sein eigenes finales SETPOSDG senden. */
    if (s_encoder_adjusting) {
        return;
    }
    /* loop(): serial_bridge/RS485 vor encoder_process_pending — alter Bus-Soll sonst vor dem Encoder-Tick. */
    if (rotor_encoder_pending_detents() != 0) {
        return;
    }
    /* Kurz nach Antennenwechsel: Slave meldet noch den alten Bus-Soll — mit neuem Versatz falsch in Anzeige;
     * erst Echo zum neuen SETPOSDG (gleicher Anzeige-Soll wie s_encoder_tenths) anwenden. */
    if (millis() < s_taget_ignore_bus_target_until_ms) {
        const int ant_probe = static_cast<int>(pwm_config_get_last_antenna());
        const float disp_probe = bus_to_logical_display(bus_deg, ant_probe);
        const int incoming_t = deg_to_tenths_rounded(disp_probe);
        const int d = wrap_tenths_delta(incoming_t - s_encoder_tenths);
        if (d > 2 || d < -2) {
            return;
        }
    }
    const int ant = static_cast<int>(pwm_config_get_last_antenna());
    const float disp = bus_to_logical_display(bus_deg, ant);
    /* Nicht um 0,1° zurückspringen: gleicher physikalischer Soll, nur floor/Float unterhalb der Session-Zehntel. */
    if (!rotor_rs485_is_remote_setpos_motion() &&
        bus_target_matches_session_tenth_noise(disp, s_encoder_tenths)) {
        return;
    }
    /* 0,1°/Klick: Bus-Soll (notify_target / Echo) oft exakt eine Zehntelstufe hinter der Encoder-Session
     * (Rundung/Slave noch nicht nachgezogen). Früher nur 900 ms Schutz → danach sprang taget zurück,
     * SETPOSDG lief mit veraltetem s_encoder_target_deg / gar kein sinnvoller Versand. Mehrere Zehntel
     * auf einmal: d != -1, kein Konflikt. */
    const int bus_t = deg_to_tenths_rounded(disp);
    {
        const int d = wrap_tenths_delta(bus_t - s_encoder_tenths);
        constexpr uint32_t kBusTargetOneTenthLagProtectMs = 4500u;
        if (!rotor_rs485_is_remote_setpos_motion() &&
            (uint32_t)(millis() - s_last_encoder_step_ms) < kBusTargetOneTenthLagProtectMs &&
            (d == -1 || d == 1)) {
            return;
        }
    }
    /* Gleiche Zehntel wie fmt_de/Encoder-Session — sonst zeigt taget_dg X, intern bleibt Y,
     * nächster Encoder-Tick parst X und „verschluckt“ einen Schritt / doppelter Sprung. */
    s_encoder_target_deg = disp;
    s_encoder_tenths = deg_to_tenths_rounded(disp);

    lvgl_port_lock(-1);
    char buf[16];
    fmt_taget_from_display_deg(buf, sizeof(buf), disp);
    /* Kein lv_refr_now: bei Bus-Flut blockiert synchrones Rendering die RS485-Zeilenverarbeitung. */
    taget_dg_set_display_text(buf, false);
    arc_target_marker_refresh();
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
    const int ant = static_cast<int>(pwm_config_get_last_antenna());
    const float disp = bus_to_logical_display(bus_deg_ui, ant);
    fmt_de(buf, sizeof(buf), disp);
    if (objects.actual_dg) {
        actual_dg_set_display_text(buf, false);
    }
    /* Während Positionsfahrt: Arc aus Ist — auch wenn Encoder-Flag noch gesetzt ist. */
    if (!s_arc_dragging &&
        (!s_encoder_adjusting || rotor_rs485_is_position_polling()) &&
        active_grad_acc()) {
        grad_acc_sync_bus(bus_deg_ui, ant, s_dipole_back_lobe_active);
    }
    arc_target_marker_refresh();
    lvgl_port_unlock();
}

/** Arc-Drag: SETPOSCC an PC-/Zweit-Master (Vorschau wie Encoder), max. alle 100 ms. */
static void arc_drag_send_setposcc(float display_deg, int ant_1_to_3)
{
    s_arc_drag_last_disp_deg = display_deg;
    s_arc_drag_cc_have_deg = true;
    const uint32_t now = millis();
    if ((int32_t)(now - s_arc_drag_cc_next_ms) < 0) {
        return;
    }
    s_arc_drag_cc_next_ms = now + ARC_DRAG_SETPOSCC_MS;
    rotor_rs485_send_setposcc_degrees(display_to_bus_for_idx(display_deg, ant_1_to_3));
}

static void arc_drag_setposcc_loop(void)
{
    if (!s_arc_dragging || !s_arc_drag_cc_have_deg || !rotor_rs485_is_referenced()) {
        return;
    }
    arc_drag_send_setposcc(
        s_arc_drag_last_disp_deg,
        static_cast<int>(pwm_config_get_last_antenna()));
}

static void on_arc(lv_event_t *e)
{
    lv_event_code_t c = lv_event_get_code(e);
    lv_obj_t *arc = lv_event_get_target(e);

    if (c == LV_EVENT_PRESSED) {
        if (!rotor_rs485_is_referenced()) {
            return;
        }
        s_arc_dragging = true;
        s_arc_moved_this_press = false;
        s_arc_value_at_press = lv_arc_get_value(arc);
        s_arc_drag_bus_ref_deg = s_last_bus_ist_deg;
        s_arc_drag_cc_next_ms = 0;
        s_arc_drag_cc_have_deg = false;
        arc_target_marker_refresh(); /* Blau aus — roter Knauf ist während Drag das Ziel */
        /* Encoder-Session hier NICHT abbrechen: kurzer Touch ohne Drehen soll keine Klicks „verschlucken“
         * und kein ausstehendes SETPOSDG verwerfen — erst bei echtem Drag (VALUE_CHANGED). */
    }
    if (c == LV_EVENT_VALUE_CHANGED) {
        if (!rotor_rs485_is_referenced()) {
            return;
        }
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
        const int ant = static_cast<int>(pwm_config_get_last_antenna());
        const float disp = bus_to_display_for_idx(
            arc_value_to_bus_shortest(v, s_arc_drag_bus_ref_deg), ant);
        s_encoder_target_deg = disp;
        char buf[16];
        fmt_taget_from_display_deg(buf, sizeof(buf), disp);
        /* Kein lv_refr_now — sonst flackert der NeoPixel-Ring bei jedem Grad. */
        taget_dg_set_display_text(buf, false);
        arc_drag_send_setposcc(disp, ant);
    }
    if (c == LV_EVENT_RELEASED) {
        s_arc_dragging = false;
        s_arc_drag_cc_have_deg = false;
        /* Nach langer SETPOSCC-Vorschau: Watchdog neu armieren (kein Slave-ACK während Drag). */
        rotor_rs485_arm_conn_watchdog();
        /* Pieps immer beim Loslassen (Arc-Callback nur bei Touch auf dem Arc) — nicht hinter
         * s_arc_updating verstecken: sonst kein Ton und kein GOTO, wenn zufällig Flag noch stand. */
        touch_feedback_arc_release();
        if (!rotor_rs485_is_referenced()) {
            arc_target_marker_refresh();
            return;
        }
        if (rotor_error_app_is_fault_locked()) {
            arc_target_marker_refresh();
            return;
        }
        const int target_v = lv_arc_get_value(arc);
        /* VALUE_CHANGED fehlt manchmal (kurzer Zug, gleicher ganzzahliger Grad, Coalescing) — Abgleich Wert Press/Release. */
        const bool moved = s_arc_moved_this_press || (target_v != s_arc_value_at_press);
        /* Nur nach echtem Drehen: GOTO — sonst (Finger kurz auf Arc) Encoder/Bus nicht mit Arc-Wert überschreiben. */
        if (!moved) {
            arc_target_marker_refresh();
            return;
        }
        const int ant = static_cast<int>(pwm_config_get_last_antenna());
        const float logical_tgt =
            bus_to_display_for_idx(arc_value_to_bus_shortest(target_v, s_arc_drag_bus_ref_deg), ant);
        char buf[16];
        fmt_taget_from_display_deg(buf, sizeof(buf), logical_tgt);
        taget_dg_set_display_text(buf, false);
        s_encoder_target_deg = logical_tgt;
        if (active_grad_acc()) {
            grad_acc_sync_bus(s_last_bus_ist_deg, ant, s_dipole_back_lobe_active);
        }
        arc_target_marker_refresh();
        const MotionBusResolve r = resolve_motion_bus(
            logical_tgt, s_last_bus_ist_deg, ant);
        s_dipole_back_lobe_active = r.use_back_lobe;
        (void)rotor_rs485_goto_degrees(r.bus_cmd);
    }
}

static void on_ref_btn(lv_event_t *e)
{
    /* Objekt „ref“ / Label „Homing“ in EEZ: Laufzeit = Encoder 0,1° ↔ 1° (wie früher encoder_delta). */
    on_encoder_delta_btn(e);
}

extern "C" void rotor_app_init(void)
{
    /* Nur EL konfiguriert (AZ=0): auf Elevation starten. */
    if (pwm_config_get_rotor_id() == 0u && pwm_config_get_rotor_el_id() != 0u) {
        s_axis = AxisMode::El;
    }
    rotor_rs485_set_master_id(pwm_config_get_master_id());
    rotor_rs485_set_slave_id(axis_slave_id());
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
        if (!rotor_rs485_is_referenced()) {
            lv_obj_clear_flag(objects.grad_acc, LV_OBJ_FLAG_CLICKABLE);
        }
    }
    if (objects.grad_acc_el) {
        lv_obj_add_event_cb(objects.grad_acc_el, on_arc, LV_EVENT_ALL, nullptr);
        lv_obj_set_style_bg_color(objects.grad_acc_el, lv_color_hex(0xff0000), LV_PART_KNOB);
        lv_obj_set_style_bg_opa(objects.grad_acc_el, LV_OPA_COVER, LV_PART_KNOB);
        if (!rotor_rs485_is_referenced()) {
            lv_obj_clear_flag(objects.grad_acc_el, LV_OBJ_FLAG_CLICKABLE);
        }
        /* EEZ startet oft sichtbar — bis zum ersten Toggle ausblenden */
        lv_obj_add_flag(objects.grad_acc_el, LV_OBJ_FLAG_HIDDEN);
    }
    if (objects.grad_acc || objects.grad_acc_el) {
        /* Soll-Punkt (zweiter Arc) — Laufzeit, ohne src/ui */
        arc_target_marker_init();
    }
    /* AZ-Start: EL-Hintergrund/EL-Arc verstecken; bei AZ=0 bereits s_axis=EL */
    apply_axis_background();
    apply_axis_arc_geometry();
    if (axis_is_el()) {
        apply_axis_ui_after_switch();
    }
    s_pwm_ui_is_fast = pwm_config_get_pwm_ui_fast() != 0;
    pwm_style_slow_fast(s_pwm_ui_is_fast);
    if (objects.slow) {
        lv_obj_add_event_cb(objects.slow, on_slow_btn, LV_EVENT_CLICKED, nullptr);
    }
    if (objects.fast) {
        lv_obj_add_event_cb(objects.fast, on_fast_btn, LV_EVENT_CLICKED, nullptr);
    }
    if (objects.ref) {
        lv_obj_add_event_cb(objects.ref, on_ref_btn, LV_EVENT_CLICKED, nullptr);
    }
    encoder_delta_apply_button_label();
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
    if (objects.rotor_az) {
        lv_obj_add_event_cb(objects.rotor_az, on_id_field_event, LV_EVENT_ALL, nullptr);
    }
    if (objects.rotor_el) {
        lv_obj_add_event_cb(objects.rotor_el, on_id_field_event, LV_EVENT_ALL, nullptr);
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
    if (!rotor_rs485_is_referenced()) {
        return false;
    }
    if (axis_is_el()) {
        target_deg = clamp_el_deg(target_deg);
    }
    const int ant = static_cast<int>(pwm_config_get_last_antenna());
    const MotionBusResolve r = resolve_motion_bus(
        target_deg, s_last_bus_ist_deg, ant);
    lvgl_port_lock(-1);
    /* Rot bleibt auf Ist; blauer Marker zeigt Encoder-Soll (Vorwahl). */
    if (ENCODER_MOVES_ARC && active_grad_acc() && !rotor_rs485_is_position_polling()) {
        grad_acc_sync_bus(s_last_bus_ist_deg, ant, s_dipole_back_lobe_active);
    }
    s_dipole_back_lobe_active = r.use_back_lobe;
    arc_target_marker_refresh();
    lvgl_port_unlock();
    return rotor_rs485_goto_degrees(r.bus_cmd);
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
        lv_obj_t *ta = objects.controller_id;
        uint8_t kind = 2;
        if (s_id_field_focus == IdFieldFocus::RotorAz) {
            ta = objects.rotor_az;
            kind = 0;
        } else if (s_id_field_focus == IdFieldFocus::RotorEl) {
            ta = objects.rotor_el;
            kind = 1;
        }
        if (ta) {
            const uint8_t v = id_field_display_or_saved_config(ta, kind);
            const int min_id = (kind == 1) ? 0 : 1;
            int nv = static_cast<int>(v) + sign;
            if (nv < min_id) {
                nv = min_id;
            }
            if (nv > 254) {
                nv = 254;
            }
            id_fields_set_text(ta, static_cast<uint8_t>(nv));
        }
        lvgl_port_unlock();
        return;
    }

    if (!rotor_rs485_is_referenced()) {
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
            s_encoder_tenths = deg_to_tenths_rounded(s_encoder_target_deg);
        }
    }
    s_encoder_adjusting = true;

    s_encoder_tenths = wrap_tenths_deg(s_encoder_tenths + delta_tenths);

    const float deg = static_cast<float>(s_encoder_tenths) / 10.0f;
    const int ant = static_cast<int>(pwm_config_get_last_antenna());
    /* Während Positionsfahrt: Arc (rot) bleibt auf Ist (on_position_deg).
     * Encoder-Vorwahl: Rot bleibt auf Ist, blauer Marker folgt dem Soll. */
    if (ENCODER_MOVES_ARC && active_grad_acc() && !rotor_rs485_is_position_polling()) {
        grad_acc_sync_bus(s_last_bus_ist_deg, ant, s_dipole_back_lobe_active);
        s_encoder_arc_int_cached = deg_to_arc_value(s_last_bus_ist_deg);
    }

    char buf[16];
    fmt_taget_from_wrapped_tenths(buf, sizeof(buf), s_encoder_tenths);
    taget_dg_set_display_text(buf, false);
    s_encoder_target_deg = deg;
    arc_target_marker_refresh();

    lvgl_port_unlock();

    /* PC/Bus: bei jedem neuen taget vor dem verzögerten SETPOSDG (gleiche Buslage wie späteres SETPOSDG) */
    rotor_rs485_send_setposcc_degrees(display_to_bus(deg));

    /* Kein Telegramm während Drehen: Pause neu starten; ausstehenden Bus-Retry verwirft neuer Takt */
    s_encoder_goto_retry_pending = false;
    s_encoder_retry_deadline_ms = 0;
    const uint32_t idle_ms = rotor_rs485_is_foreign_pc_listen_mode()
        ? ENCODER_SEND_IDLE_MS_FOREIGN_PC
        : ENCODER_SEND_IDLE_MS;
    s_encoder_idle_deadline_ms = millis() + idle_ms;
    s_last_encoder_step_ms = millis();
}

extern "C" bool rotor_app_encoder_id_field_focused(void)
{
    return s_id_field_focus != IdFieldFocus::None;
}

extern "C" bool rotor_app_is_ui_preview_active(void)
{
    return s_arc_dragging || s_encoder_adjusting;
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
    if (rotor_rs485_is_referenced()) {
        /* Nach Boot (GETANTOFF/GETANTDP) ist s_goto_commanded_deg oft noch 0.
         * Soll muss hier auf dem aktuellen Ist starten, damit Encoder von der
         * realen Position zählt und nicht von 0/360. */
        on_target_deg(s_last_bus_ist_deg);
    }
}

/**
 * Nach pwm_config_set_last_antenna(neu): Ist und Soll auf die neue Antennen-Anzeige gleichziehen
 * (gleicher Buswinkel + neuer Versatz) — kein SETPOS nur wegen des Wechsels.
 * send_bus_goto: ungenutzt (API/Mitläufer); Strahl-Erhalt war früher concha=1.
 */
static void rotor_app_antenna_switch_from_ui(uint8_t prev_antenna_1_to_3, bool send_bus_goto)
{
    (void)send_bus_goto;
    s_encoder_adjusting = false;
    s_encoder_goto_retry_pending = false;
    s_encoder_retry_deadline_ms = 0;
    s_encoder_idle_deadline_ms = 0;

    const uint8_t now_ant = pwm_config_get_last_antenna();
    if (prev_antenna_1_to_3 < 1u || prev_antenna_1_to_3 > 3u || now_ant < 1u || now_ant > 3u) {
        return;
    }

    /* Soll = Ist unter neuer Antenne (Versatz) — Anzeige bleibt konsistent, Rotor steht. */
    const float soll_display =
        bus_to_logical_display(s_last_bus_ist_deg, static_cast<int>(now_ant));
    s_encoder_target_deg = soll_display;
    s_encoder_tenths = deg_to_tenths_rounded(soll_display);
    s_taget_ignore_bus_target_until_ms = 0;

    on_position_deg(s_last_bus_ist_deg);

    if (!rotor_rs485_is_referenced() || !objects.taget_dg) {
        return;
    }

    lvgl_port_lock(-1);
    char buf[16];
    fmt_taget_from_display_deg(buf, sizeof(buf), soll_display);
    taget_dg_set_display_text(buf);
    arc_target_marker_refresh();
    lvgl_port_unlock();
}

/**
 * Außentemp (ACK_GETTEMPA): Wetter-Tab (temperature); Motortemp immer.
 * Wind nur bei anemometer=1.
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
        if (objects.temperature) {
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
    arc_drag_setposcc_loop();

    if (!s_encoder_adjusting) {
        return;
    }
    if (!rotor_rs485_is_referenced()) {
        s_encoder_goto_retry_pending = false;
        s_encoder_idle_deadline_ms = 0;
        s_encoder_adjusting = false;
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
        rotor_rs485_arm_conn_watchdog();
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
    rotor_rs485_arm_conn_watchdog();
    s_encoder_retry_deadline_ms = 0;
}

extern "C" void rotor_app_snap_target_to_deg(float bus_deg)
{
    s_encoder_adjusting = false;
    s_encoder_goto_retry_pending = false;
    s_encoder_retry_deadline_ms = 0;
    s_encoder_idle_deadline_ms = 0;
    const int ant = static_cast<int>(pwm_config_get_last_antenna());
    const float disp = bus_to_logical_display(bus_deg, ant);
    s_encoder_target_deg = disp;
    s_encoder_tenths = deg_to_tenths_rounded(disp);

    lvgl_port_lock(-1);
    char buf[16];
    fmt_taget_from_display_deg(buf, sizeof(buf), disp);
    taget_dg_set_display_text(buf, false);
    if (ENCODER_MOVES_ARC && active_grad_acc()) {
        const MotionBusResolve r = resolve_motion_bus(disp, s_last_bus_ist_deg, ant);
        s_dipole_back_lobe_active = r.use_back_lobe;
        /* Rot = Ist; Blau = Soll */
        grad_acc_sync_bus(s_last_bus_ist_deg, ant, s_dipole_back_lobe_active);
    }
    arc_target_marker_refresh();
    lvgl_port_unlock();
}
