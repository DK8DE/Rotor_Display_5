#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Nach ui_init() aufrufen (mit LVGL-Lock wie in main). Registriert Homing-Button, RS485, erste GETREF. */
void rotor_app_init(void);

/** Slow/Fast-SETPWM: erneuter Versuch wenn Bus frei (nach Klick) + einmal Boot Fast-PWM. */
void rotor_pwm_ui_loop(void);

/** Nach serial_bridge::poll + rotor_rs485_idle_tasks aus loop() — Label wind_speed / temperature. */
void rotor_app_weather_ui_poll(void);

/** Nach pwm_config-Änderung per RS485/USB (Labels, IDs, PWM-Anzeige; LVGL intern). */
void rotor_app_config_changed_from_bus(void);

/**
 * Antennenwahl per Bus/USB (SETASELECT): nur aus rotor_rs485_idle_tasks() im Haupt-loop —
 * nicht aus Parser-/Bridge-Tasks (Flash/LVGL sonst WDT/Neustart).
 * @param prev_1_to_3 Antenne vor dem Wechsel (für Kompass-/SETPOS-Umrechnung)
 * @param new_1_to_3 Zielantenne 1…3
 */
void rotor_app_apply_remote_antenna_selection_deferred(uint8_t prev_1_to_3, uint8_t new_1_to_3);

/**
 * Encoder: delta_tenths = Änderung in Zehntelgraden (Skalierung z. B. in main).
 * SETPOSDG erst nach 200 ms ohne neuen Tick; rotor_app_loop() aus loop() aufrufen (Retry bei blockiertem Bus).
 * Encoder-Ticks kommen typ. aus dem FreeRTOS-Task encReader (main.cpp), nicht aus loop().
 */
void rotor_app_encoder_step(int delta_tenths);
void rotor_app_loop(void);

/** 0 = Azimut, 1 = Elevation — aktive Achse (Hardware-Long-Press schaltet um). */
uint8_t rotor_app_get_axis(void);
/** AZ ↔ EL umschalten (Hintergrund, Arc, Slave-ID, Ist/Soll). */
void rotor_app_toggle_axis(void);

/**
 * Boot/Status/Mitläufer: Ist + Referenz einer Achse in den App-Cache schreiben (ohne UI zu wechseln).
 * axis: 0 = AZ, 1 = EL.
 */
void rotor_app_seed_axis_cache(uint8_t axis, float bus_ist_deg, bool referenced);

/**
 * Mitläufer: Soll einer inaktiven Achse aus Bus-SETPOSDG (Busgrad → Anzeige-Soll im Cache).
 * axis: 0 = AZ, 1 = EL.
 */
void rotor_app_seed_axis_target_bus(uint8_t axis, float target_bus_deg);

/** Mitläufer: nur Referenzstatus einer Achse setzen (Ist unverändert). */
void rotor_app_seed_axis_referenced(uint8_t axis, bool referenced);

/** Nach GETROTORTYPE / EL-Limit-Änderung: Arc-Range und Clamp neu anwenden (falls EL aktiv). */
void rotor_app_el_limits_changed(void);

/** Ist in Anzeige-Koordinaten — für NeoPixel-Ring (AZ: +Versatz; EL: 0…180). */
float rotor_app_get_display_direction_deg(void);

/** Soll in Anzeige-Koordinaten — für NeoPixel-Ring / Zielmarker. */
float rotor_app_get_display_target_deg(void);

/** Noch nicht abgearbeitete Encoder-Rasten ±1 (main.cpp); für on_target_deg: Bus nicht vor encoder_process_pending überschreiben. */
int rotor_encoder_pending_detents(void);

/** true: Fokus auf rotor_az / rotor_el / controller_id — main bündelt Encoder nicht. */
bool rotor_app_encoder_id_field_focused(void);

/** true: Arc-Drag oder Encoder-Vorwahl (SETPOSCC) — kein Verbindungs-Watchdog. */
bool rotor_app_is_ui_preview_active(void);

/**
 * Hardware-Taster: wenn ein ID-Feld fokussiert ist — Fokus weg.
 * Flash (config.json) nur wenn die Zahl gültig ist und sich von der gespeicherten unterscheidet.
 * @return true wenn behandelt (kein Homing/Stop/Snap in diesem Klick).
 */
bool rotor_app_commit_id_field_on_hw_click(void);

/**
 * Encoder-Session beenden, Soll-Anzeige + internen Soll auf deg setzen (z. B. Hardware-Taster: Ziel = Ist).
 * Muss vor rotor_rs485_goto_degrees(deg) aufgerufen werden, sonst blockiert on_target_deg und
 * rotor_app_loop kann ein veraltetes SETPOSDG erneut senden.
 */
void rotor_app_snap_target_to_deg(float deg);

#ifdef __cplusplus
}
#endif
