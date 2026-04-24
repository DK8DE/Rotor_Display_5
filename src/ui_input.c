/**
 * @file ui_input.c
 *
 * Platzhalter-Implementierung für Eingabe-Events (Rotary + Taster).
 *
 * Hintergrund:
 * - Das Demo registriert Callbacks für Rotary und Taster.
 * - Früher gingen die Events an eine externe UI-Library.
 * - Mit EEZ-Studio-UI (`src/ui/`) sind die konkreten Aktionen noch offen.
 *
 * Stand:
 * - Touch / Rotary / Taster bleiben hardwareseitig aktiv.
 * - Die EEZ-GUI nutzt Rotary/Taster hier noch nicht.
 * - Die Callbacks sind No-Ops, damit der Build stabil bleibt und Erweiterungen einfach bleiben.
 */

#include "ui.h"

#include <stdint.h>

/*
 * Optional: letzte Event-Codes (volatile, Events aus ISR/Task-Kontext).
 */
static volatile int32_t g_last_knob_event = -1;
static volatile int32_t g_last_button_event = -1;

void LVGL_knob_event(void *event)
{
    /* Event-Code wie im Demo als (void*) übergeben — hier nur speichern. */
    g_last_knob_event = (int32_t)(intptr_t)event;

    /*
     * TODO (später):
     * - Fokus-Navigation über lv_group (lv_group_focus_next/prev)
     * - Werte ändern
     * - Screenwechsel
     */
}

void LVGL_button_event(void *event)
{
    g_last_button_event = (int32_t)(intptr_t)event;

    /*
     * TODO (später):
     * - Bestätigen/OK
     * - Zurück
     * - Long-Press Funktionen
     */
}

