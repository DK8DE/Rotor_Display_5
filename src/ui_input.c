/**
 * @file ui_input.c
 *
 * Platzhalter-Implementierung für Eingabe-Events (Rotary + Taster).
 *
 * Warum existiert diese Datei?
 * - Das ursprüngliche Demo hat bereits Callbacks für Rotary und Taster.
 * - Bisher wurden diese Events an eine externe UI-Library übergeben.
 * - Nach dem Austausch auf die EEZ-Studio UI fehlen diese Funktionen.
 *
 * Aktueller Stand (wie von dir gewünscht):
 * - Touch / Rotary / Taster bleiben im Demo weiterhin "hardwareseitig" aktiv.
 * - Was die EEZ-GUI mit Rotary/Taster macht, kommt später.
 * - Deshalb machen diese Funktionen aktuell nichts (No-Op),
 *   damit der Code sauber kompiliert und später leicht erweiterbar ist.
 */

#include "ui.h"

#include <stdint.h>

/*
 * Optionale Debug-/Status-Variablen (kannst du später nutzen, um Events zu prüfen).
 * Achtung: volatile, weil die Events aus verschiedenen Kontexten kommen können.
 */
static volatile int32_t g_last_knob_event = -1;
static volatile int32_t g_last_button_event = -1;

void LVGL_knob_event(void *event)
{
    /*
     * Im bisherigen Demo wurde der Event-Code als (void*) übergeben.
     * Wir speichern ihn nur ab.
     */
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

