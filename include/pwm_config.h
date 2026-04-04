#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Defaults: Slow 50 %, Fast 100 % */
void pwm_config_load_defaults(void);
/** Liest config.json auf der FFat-Partition (data/ → uploadfs); bei Fehlern bleiben Defaults. */
void pwm_config_load(void);
/** Schreibt aktuelle Werte (PWM, Antennen-Labels, Versätze, last_antenna). */
void pwm_config_save(void);

uint8_t pwm_config_get_slow(void);
uint8_t pwm_config_get_fast(void);

/** RS485: eigene Master-ID (1…254), Default 2 */
uint8_t pwm_config_get_master_id(void);
void pwm_config_set_master_id(uint8_t id);
/** RS485: Rotor-Slave-ID (1…254, nicht 255/Broadcast), Default 20 */
uint8_t pwm_config_get_rotor_id(void);
void pwm_config_set_rotor_id(uint8_t id);

void pwm_config_set_slow(uint8_t pct);
void pwm_config_set_fast(uint8_t pct);

/** Antennen-UI: 1…3 */
uint8_t pwm_config_get_last_antenna(void);
void pwm_config_set_last_antenna(uint8_t n);
/** Zeiger auf internen Puffer (bis nächstem load/save); idx 1…3 */
const char *pwm_config_get_antenna_label(int idx);
/** idx 1…3, max. 47 Zeichen + NUL */
void pwm_config_set_antenna_label(int idx, const char *s);

/** Versatz in Grad (Rotor GETANTOFF / SETANTOFF), idx 1…3 */
float pwm_config_get_antoff_deg(int idx);
void pwm_config_set_antoff_deg(int idx, float deg);

/** Öffnungswinkel in Grad (Rotor GETANGLEn / SETANGLEn), idx 1…3 — Anzeige NeoPixel-Sektor */
float pwm_config_get_opening_deg(int idx);
void pwm_config_set_opening_deg(int idx, float deg);

/** UI-Touch-Pieps (Signals/ATtiny): Frequenz Hz (200…4000), Lautstärke 0…50 */
uint16_t pwm_config_get_touch_beep_freq_hz(void);
void pwm_config_set_touch_beep_freq_hz(uint16_t hz);
uint8_t pwm_config_get_touch_beep_vol(void);
void pwm_config_set_touch_beep_vol(uint8_t vol);

/** Anemometer/Wetter-Tab: 1 = Wind + Außentemp + Richtung im Wetter-Tab; 0 = Wetter-Tab aus, GETTEMPA für Außentemp (Rotor_Info) bleibt */
uint8_t pwm_config_get_anemometer(void);
void pwm_config_set_anemometer(uint8_t on_0_or_1);

/** Encoder-Schritt in Zehntelgrad: 1 = 0,1°/Raste, 10 = 1°/Raste (GETCONDELTA/SETCONDELTA) */
uint8_t pwm_config_get_encoder_delta_tenths(void);
void pwm_config_set_encoder_delta_tenths(uint8_t tenths_1_or_10);

/**
 * Antennenwechsel (GETCONCHA/SETCONCHA, JSON concha): 1 = Anzeige-Soll (taget) beibehalten, SETPOS für neue
 * Geometrie; 0 = taget auf aktuelle Ist-Anzeige (Kompass) setzen, kein zusätzliches SETPOS.
 */
uint8_t pwm_config_get_concha(void);
void pwm_config_set_concha(uint8_t zero_or_one);

#ifdef __cplusplus
}
#endif
