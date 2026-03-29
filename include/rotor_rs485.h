/**
 * Rotor-Controller RS485 (Format aus Info/RotorController_RS485.html).
 * Master-ID / Slave-ID konfigurierbar; Checksumme: SRC + DST + letzter Zahlenwert in PARAMS.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Callback: true = referenziert (GETREF 1), false = nicht referenziert (0). */
typedef void (*rotor_rs485_ref_cb_t)(bool referenced);

/** Aktuelle Position in Grad (GETPOSDG / ACK_GETPOSDG). */
typedef void (*rotor_rs485_pos_cb_t)(float deg);

/**
 * Soll-Position (z. B. SETPOSDG vom Bus — PC oder lokales Senden).
 * Nur wenn DST = konfigurierte Rotor-ID (Slave).
 */
typedef void (*rotor_rs485_target_cb_t)(float deg_deg);

void rotor_rs485_set_master_id(uint8_t id);
void rotor_rs485_set_slave_id(uint8_t id);

/** Boot: 2 s nach Start erste GETREF, dann optional GETPOSDG (nur wenn referenziert). */
void rotor_rs485_init(void);
/**
 * Rohdaten vom RS485-UART (z. B. von serial_bridge nach HW→USB-Spiegelung).
 * Verarbeitet denselben Byte-Strom wie früher drain_rx intern.
 */
void rotor_rs485_rx_bytes(const uint8_t *data, size_t len);

/** Aus loop() aufrufen: ggf. GETREF/GETPOSDG-Polling (Empfang passiert in serial_bridge::poll). */
void rotor_rs485_loop(void);

void rotor_rs485_set_ref_callback(rotor_rs485_ref_cb_t cb);
void rotor_rs485_set_position_callback(rotor_rs485_pos_cb_t cb);
void rotor_rs485_set_target_callback(rotor_rs485_target_cb_t cb);

/** Einmal GETREF (z. B. beim Start). */
void rotor_rs485_send_getref(void);
/** SETREF:1 — Homing starten; danach GETREF alle 200 ms bis referenziert. */
void rotor_rs485_send_setref_homing(void);

/** STOP:0 — weicher Stopp (Positionsfahrt beenden). true wenn gesendet. */
bool rotor_rs485_send_stop(void);

/** Einmal GETPOSDG (z. B. Ist-Position beim Start, ohne Polling). */
void rotor_rs485_send_getposdg(void);

/** Letzter bekannter Referenzstatus (zuletzt ACK_GETREF). */
bool rotor_rs485_is_referenced(void);

/**
 * Rotor „fährt“ aus Sicht des Masters: Positions-Polling aktiv oder Homing-Polling.
 * (Kein separates Motor-Flag vom Slave.)
 */
bool rotor_rs485_is_moving(void);

/** Homing läuft: GETREF-Polling nach SETREF, noch nicht referenziert. */
bool rotor_rs485_is_homing(void);

/** True nach SETPOSDG bis Ist ≈ Soll (GETPOSDG-Polling / Positionsfahrt). */
bool rotor_rs485_is_position_polling(void);

/** Letzte Ist-Position für UI (GETPOSDG), in Grad. */
float rotor_rs485_get_last_position_deg(void);

/**
 * SETPOSDG mit Zielgrad (0…360°), dann GETPOSDG alle 200 ms bis Ist ≈ Soll.
 * Grad werden mit zwei Nachkommastellen und Komma im Telegramm gesendet (wie Doku).
 * Ruft den Target-Callback mit dem übergebenen deg auf (taget_dg), sobald gesendet.
 * @return false nur wenn z. B. SETREF/SETPOS noch ACK aussteht; ein ausstehendes GETPOSDG
 *         wird abgebrochen, damit Encoder-Befehle nicht „hängen bleiben“.
 */
bool rotor_rs485_goto_degrees(float deg);

/**
 * HW-Taster „Ziel = aktuelle Position“: SETPOSDG mit deg wiederholt senden, bis der Bus frei ist
 * und ACK_SETPOSDG eintrifft (sonst Retry in rotor_rs485_loop nach Timeout / wenn goto_degrees fehlschlug).
 */
void rotor_rs485_hw_snap_retarget_request(float deg);

#ifdef __cplusplus
}
#endif
