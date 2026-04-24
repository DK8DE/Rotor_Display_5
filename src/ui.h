/**
 * @file ui.h
 *
 * Lokaler UI-Wrapper für dieses Beispielprojekt.
 *
 * Aufbau:
 * - Statt einer externen UI-Library (`<ui.h>`) liegt die EEZ-Studio-generierte UI unter `src/ui/`.
 * - Platzhalter-Callbacks `LVGL_knob_event()` / `LVGL_button_event()` für die Demo-Hooks.
 * - Konkrete Logik (Fokus, Screens, Werte) kann später ergänzt werden.
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
 * Rotary-Encoder: Event-Codes wie im Demo als `(void*)` übergeben; Namen unverändert zu den Demo-Typen.
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
