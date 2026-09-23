#ifndef EEZ_LVGL_UI_STYLES_H
#define EEZ_LVGL_UI_STYLES_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

// Style: Grosse_Zahlen
lv_style_t *get_style_grosse_zahlen_MAIN_DEFAULT();
lv_style_t *get_style_grosse_zahlen_MAIN_FOCUS_KEY();
void add_style_grosse_zahlen(lv_obj_t *obj);
void remove_style_grosse_zahlen(lv_obj_t *obj);

// Style: Runde Tabview
lv_style_t *get_style_runde_tabview_MAIN_SCROLLED();
void add_style_runde_tabview(lv_obj_t *obj);
void remove_style_runde_tabview(lv_obj_t *obj);

// Style: Tabs
lv_style_t *get_style_tabs_MAIN_FOCUS_KEY();
lv_style_t *get_style_tabs_MAIN_SCROLLED();
lv_style_t *get_style_tabs_MAIN_DEFAULT();
void add_style_tabs(lv_obj_t *obj);
void remove_style_tabs(lv_obj_t *obj);

// Style: Menue_Tabs
lv_style_t *get_style_menue_tabs_MAIN_SCROLLED();
void add_style_menue_tabs(lv_obj_t *obj);
void remove_style_menue_tabs(lv_obj_t *obj);

// Style: Menue_Tabview
lv_style_t *get_style_menue_tabview_MAIN_SCROLLED();
void add_style_menue_tabview(lv_obj_t *obj);
void remove_style_menue_tabview(lv_obj_t *obj);

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_STYLES_H*/