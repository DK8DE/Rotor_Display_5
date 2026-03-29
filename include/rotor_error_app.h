/**
 * Fehleranzeige (GETERR / async ERR), Meldetext, NeoPixel-Ring, Homing-LED.
 * Siehe Info/RotorController_RS485.html (GETERR → ACK_ERR, ERR-Telegramm).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void rotor_error_app_init(void);
/** Aus loop(): Meldetext, LED-Blink (LVGL-Lock intern) */
void rotor_error_app_loop(uint32_t now_ms);

/** Letzter Fehlercode (0 = keiner); gesetzt durch GETERR oder #…:ERR:… */
void rotor_error_app_set_error_code(int code);
int rotor_error_app_get_error_code(void);

/** NeoPixel: kompletter Ring rot (nur bei aktivem Fehler, nicht während Referenzfahrt) */
bool rotor_error_app_is_fault_ring_red(void);

/** Encoder-Taster: true, wenn Fehler aktiv → SingleClick soll zuerst SETREF (Homing) senden */
bool rotor_error_app_encoder_click_triggers_homing_only(void);

#ifdef __cplusplus
}
#endif
