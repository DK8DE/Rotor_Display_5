/**
 * NeoPixel-Ring über Signals (direkt GPIO39): Homing, Referenz, Richtungsanzeige.
 * Helligkeit: pwm_config_scale_led_ring_brightness (SETCONLEDP / conledp).
 */
#pragma once

#include <stdint.h>

class Signals;

void signals_ring_app_init(Signals *signals, uint8_t num_leds);
void signals_ring_app_loop(uint32_t now_ms);
/** Sperrt Ring-Updates kurzzeitig (ms); bei direkter Ansteuerung meist unnötig, API bleibt. */
void signals_ring_app_pause_updates_for(uint16_t hold_ms);
