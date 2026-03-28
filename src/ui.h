/**
 * @file ui.h
 *
 * Lokaler UI-Wrapper für dieses Beispielprojekt.
 *
 * Ziel:
 * - Statt der bisher eingebundenen externen UI-Library ("<ui.h>") wird hier die von EEZ Studio
 *   generierte UI aus dem Unterordner "src/ui/" verwendet.
 * - Zusätzlich stellen wir (vorerst als Platzhalter) die beiden Funktionen bereit, die das
 *   Demo bereits aufruft: LVGL_knob_event() und LVGL_button_event().
 *   -> Die konkrete Logik (z.B. Fokuswechsel, Screenwechsel, Werte ändern) machen wir später.
 */

#ifndef APP_UI_WRAPPER_H
#define APP_UI_WRAPPER_H

#include <stdint.h>

/*
 * EEZ-Studio UI (src/ui/ui.h) inkludiert C++-Code (eez-flow.h, Templates).
 * Darf nicht in extern "C" stehen — nur in C++-TUs einbinden.
 * Reine C-Dateien (z. B. ui_input.c) sehen nur die Callback-Deklarationen unten.
 */
#ifdef __cplusplus
#include "ui/ui.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Ereignisse für den Rotary-Encoder.
 *
 * Hinweis:
 * - Im bisherigen Demo wurden diese Werte als (void*) übergeben.
 * - Wir lassen die Namen bewusst identisch, damit du später ohne Umbau an den Callbacks
 *   weiterarbeiten kannst.
 */


/**
 * Platzhalter-Funktion für Rotary-Events.
 *
 * @param event Als (void*) übergebener Event-Code (siehe knob_event_t)
 */
void LVGL_knob_event(void *event);

/**
 * Platzhalter-Funktion für Button-Events.
 *
 * @param event Als (void*) übergebener Event-Code (siehe button_event_t)
 */
void LVGL_button_event(void *event);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* APP_UI_WRAPPER_H */
