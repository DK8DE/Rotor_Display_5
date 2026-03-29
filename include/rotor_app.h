#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** Nach ui_init() aufrufen (mit LVGL-Lock wie in main). Registriert Homing-Button, RS485, erste GETREF. */
void rotor_app_init(void);

/** Slow/Fast-SETPWM: erneuter Versuch wenn Bus frei (nach Klick) + einmal Boot Fast-PWM. */
void rotor_pwm_ui_loop(void);

/**
 * Encoder: delta_tenths = Änderung in Zehntelgraden (Skalierung z. B. in main).
 * SETPOSDG erst nach 200 ms ohne neuen Tick; rotor_app_loop() aus loop() aufrufen (Retry bei blockiertem Bus).
 * Encoder-Ticks kommen typ. aus dem FreeRTOS-Task encReader (main.cpp), nicht aus loop().
 */
void rotor_app_encoder_step(int delta_tenths);
void rotor_app_loop(void);

/**
 * Encoder-Session beenden, Soll-Anzeige + internen Soll auf deg setzen (z. B. Hardware-Taster: Ziel = Ist).
 * Muss vor rotor_rs485_goto_degrees(deg) aufgerufen werden, sonst blockiert on_target_deg und
 * rotor_app_loop kann ein veraltetes SETPOSDG erneut senden.
 */
void rotor_app_snap_target_to_deg(float deg);

#ifdef __cplusplus
}
#endif
