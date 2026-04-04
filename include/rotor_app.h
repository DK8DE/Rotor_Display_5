#pragma once

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
 * Encoder: delta_tenths = Änderung in Zehntelgraden (Skalierung z. B. in main).
 * SETPOSDG erst nach 200 ms ohne neuen Tick; rotor_app_loop() aus loop() aufrufen (Retry bei blockiertem Bus).
 * Encoder-Ticks kommen typ. aus dem FreeRTOS-Task encReader (main.cpp), nicht aus loop().
 */
void rotor_app_encoder_step(int delta_tenths);
void rotor_app_loop(void);

/** Ist in Anzeige-Koordinaten (Bus + Antennenversatz der gewählten Antenne), wie LVGL-Arc — für NeoPixel-Ring. */
float rotor_app_get_display_direction_deg(void);

/** Noch nicht abgearbeitete Encoder-Rasten ±1 (main.cpp); für on_target_deg: Bus nicht vor encoder_process_pending überschreiben. */
int rotor_encoder_pending_detents(void);

/**
 * Hardware-Taster: wenn ein ID-Feld (rotor_id / controller_id) fokussiert ist — Fokus weg.
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
