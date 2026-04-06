#pragma once

#include <stdint.h>

#ifdef __cplusplus
class Signals;

/** ATtiny-Tonausgabe (Serial1); vor erstem Touch aufrufen. */
void touch_feedback_set_signals(Signals *sig);

/**
 * Kurzer Button-Pieps (LV_EVENT_CLICKED-Handler in rotor_app — nicht vom Touch-read_cb).
 * Gleiche Parameter wie konfigurierbarer Touch-Beep; zuverlässig mit der tatsächlichen Klick-Aktion.
 */
void touch_feedback_button_click(void);

/** Kurzer Pieps beim Loslassen des Kompass-Arcs (rotor_app on_arc RELEASED). */
void touch_feedback_arc_release(void);
#endif
