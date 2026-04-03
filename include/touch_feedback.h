#pragma once

#include <stdint.h>

#ifdef __cplusplus
class Signals;

/** ATtiny-Tonausgabe (Serial1); vor erstem Touch aufrufen. */
void touch_feedback_set_signals(Signals *sig);

/**
 * Aufruf aus dem Touch-read_cb bei Flanke (nur pressed=true wertet Pieps aus).
 */
void touch_feedback_touchpad_edge(bool pressed, int16_t x, int16_t y);

/** Kurzer Pieps beim Loslassen des Kompass-Arcs (rotor_app on_arc RELEASED). */
void touch_feedback_arc_release(void);
#endif
