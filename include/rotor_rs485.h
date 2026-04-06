/**
 * Rotor-Controller RS485 (Format aus Info/RotorController_RS485.html).
 * Master-ID / Slave-ID konfigurierbar; Checksumme: SRC + DST + letzter Zahlenwert in PARAMS.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Broadcast-Ziel (SETASELECT u. a.) — CS = SRC + 255 + letzte Zahl in PARAMS */
#ifndef ROTOR_RS485_BROADCAST_ID
#define ROTOR_RS485_BROADCAST_ID 255u
#endif

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
/** True nach erstem abgeschlossenen Boot-Handshake (GETREF/ggf. GETPOSDG). */
bool rotor_rs485_is_boot_done(void);
/**
 * True nach erstem GETERR (ACK/NAK) vor dem Boot-GETREF, oder Mitläufer-Modus / mitgehörtem ACK_ERR.
 * Bis dahin soll die UI kein „Betriebsbereit“ nur wegen GETREF=1 zeigen (Fehler hat Vorrang).
 */
bool rotor_rs485_is_startup_error_checked(void);
/**
 * Rohdaten vom RS485-UART (z. B. von serial_bridge nach HW→USB-Spiegelung).
 * Verarbeitet denselben Byte-Strom wie früher drain_rx intern.
 */
void rotor_rs485_rx_bytes(const uint8_t *data, size_t len);

/** Aus loop() aufrufen: ggf. GETREF/GETPOSDG-Polling (Empfang passiert in serial_bridge::poll). */
void rotor_rs485_loop(void);

/**
 * True, wenn ein anderer Master (SRC ≠ rotor_rs485_set_master_id) den Rotor (DST = Slave-ID) anfragt
 * und kürzlich Traffic war — dann sendet der Controller kein eigenes GET-Polling (nur Mitlesen).
 * Voraussetzung: PC/Software nutzt eine andere Master-ID als der Controller.
 */
bool rotor_rs485_is_foreign_pc_listen_mode(void);

void rotor_rs485_set_ref_callback(rotor_rs485_ref_cb_t cb);
void rotor_rs485_set_position_callback(rotor_rs485_pos_cb_t cb);
void rotor_rs485_set_target_callback(rotor_rs485_target_cb_t cb);

/**
 * Nach serial_bridge::poll() aufrufen: Antennen-UI nachziehen wenn Versatz vom Bus kam —
 * darf nicht im RS485-Zeilenparser laufen (LVGL).
 */
void rotor_rs485_idle_tasks(void);

/** Letzte Werte aus ACK_GETANEMO / ACK_GETTEMPA / ACK_GETTEMPM / ACK_WINDDIR (auch PC-Mitlesen). */
float rotor_rs485_get_last_wind_kmh(void);
float rotor_rs485_get_last_tempa_c(void);
/** Motortemperatur °C (GETTEMPM). */
float rotor_rs485_get_last_tempm_c(void);
/** Windrichtung 0…360° (meteorologisch, wie Slave). */
float rotor_rs485_get_last_wind_dir_deg(void);
/** Bit 0x1 Wind km/h, 0x2 Außentemp, 0x4 Windrichtung, 0x8 Motortemp — liest aus und löscht die Maske. */
uint8_t rotor_rs485_weather_ui_take_mask(void);

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

/**
 * True, wenn SETPOSDG vom Bus geschnüffelt wurde (z. B. PC über USB), ohne dass das Display
 * GETPOSDG-Polling gestartet hat — Fahrt läuft, aber is_moving() ist false.
 * Für HW-Taster STOP (siehe main SingleClickCb).
 */
bool rotor_rs485_is_remote_setpos_motion(void);

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
 * SETPOSCC: Zielgrad wie SETPOSDG (0…360°, zwei Nachkommastellen, Komma), ohne ACK/Pending.
 * Nur im Mitläufer-Modus (fremder Master spricht den Rotor an): Telegramm geht an diesen Master (DST),
 * nicht an den Rotor — der Slave liefert sonst NAK_SETPOSCC:NOTIMPL. Eigenbetrieb: kein Versand.
 * Wird gequeued und in rotor_rs485_loop geflusht, sobald kein anderes Telegramm aussteht.
 */
void rotor_rs485_send_setposcc_degrees(float deg);

/**
 * HW-Taster „Ziel = aktuelle Position“: SETPOSDG mit deg wiederholt senden, bis der Bus frei ist
 * und ACK_SETPOSDG eintrifft (sonst Retry in rotor_rs485_loop nach Timeout / wenn goto_degrees fehlschlug).
 */
void rotor_rs485_hw_snap_retarget_request(float deg);

/**
 * SETPWM: PWM-Limit 0…100 % (Laufzeit, ohne NVS). true wenn das Telegramm gestartet wurde.
 * Bei blockiertem Bus (anderes Pending) false — Aufrufer kann später erneut versuchen.
 */
bool rotor_rs485_send_set_pwm_limit(uint8_t pct);

/**
 * SETASELECT:1…3 an Broadcast (255), kein ACK — gleicher Pfad wie andere Befehle (USB + RS485).
 */
void rotor_rs485_send_setaselect(uint8_t antenna_1_to_3);

#ifdef __cplusplus
}
#endif
