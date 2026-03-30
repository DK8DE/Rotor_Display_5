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
/** RS485: Rotor-Slave-ID (1…254, nicht 255/Broadcast), Default 20 */
uint8_t pwm_config_get_rotor_id(void);

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

#ifdef __cplusplus
}
#endif
