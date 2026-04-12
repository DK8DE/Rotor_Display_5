/**
 * NeoPixel-Ring über Signals (Serial1 → ATtiny): Homing, Referenz, Richtungsanzeige.
 */
#pragma once

#include <stdint.h>

class Signals;

void signals_ring_app_init(Signals *signals, uint8_t num_leds);
void signals_ring_app_loop(uint32_t now_ms);
/** Sperrt Ring-Updates kurzzeitig (ms), z. B. waehrend eines Touch-Beep-Kommandos. */
void signals_ring_app_pause_updates_for(uint16_t hold_ms);
