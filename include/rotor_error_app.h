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

/**
 * Fehlercode setzen (0 = keiner).
 * Typisch: lokaler Verbindungs-Watchdog → Code 10 (soft, quittierbar bei Slave-Verkehr).
 */
void rotor_error_app_set_error_code(int code);

/**
 * Fehler vom Rotor (asynchrones ERR / ACK_ERR).
 * Auch Code 10 latched als Störung (nicht sofort durch nächstes ACK löschen).
 * ACK_ERR:0 → report_rotor_err(0) quittiert softes/Rotor-10.
 */
void rotor_error_app_report_rotor_err(int code);

int rotor_error_app_get_error_code(void);

/** True wenn aktiver Fehler vom Rotor gemeldet wurde (ERR/ACK_ERR). */
bool rotor_error_app_is_rotor_reported(void);

/** NeoPixel: kompletter Ring rot bei aktivem Fehler */
bool rotor_error_app_is_fault_ring_red(void);

/**
 * True bei Störung: kein Homing/Tasten.
 * Lokaler Watchdog-Code 10 ausgenommen; Rotor-ERR:10 sperrt.
 */
bool rotor_error_app_is_fault_locked(void);

#ifdef __cplusplus
}
#endif
