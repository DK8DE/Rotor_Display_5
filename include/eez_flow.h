/**
 * Brücke für EEZ Studio: generiertes ui.h nutzt #include "eez_flow.h".
 * Liegt bewusst unter include/ (nicht unter src/ui/), damit UI-Exporte
 * diese Datei nicht überschreiben oder löschen.
 */
#pragma once

#if defined(EEZ_FOR_LVGL)
#include <eez/flow/lvgl_api.h>
/* ui.c nutzt native_var_t / NATIVE_VAR_TYPE_NONE; EEZ exportiert oft auskommentiertes
 * <eez/core/vars.h> – hier zentral, ohne src/ui/ anzufassen. */
#include <eez/core/vars.h>
#else
#error "EEZ_FOR_LVGL must be defined (see platformio.ini build_flags)"
#endif
