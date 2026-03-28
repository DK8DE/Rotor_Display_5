#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

// Screens

enum ScreensEnum {
    _SCREEN_ID_FIRST = 1,
    SCREEN_ID_MAIN = 1,
    _SCREEN_ID_LAST = 1
};

typedef struct _objects_t {
    lv_obj_t *main;
    lv_obj_t *kompass_bg;
    lv_obj_t *grad_acc;
    lv_obj_t *hauptanzeige;
    lv_obj_t *position;
    lv_obj_t *label_gradzeichen_a;
    lv_obj_t *label_gradzeichen_t;
    lv_obj_t *actual_dg;
    lv_obj_t *taget_dg;
    lv_obj_t *label_actual;
    lv_obj_t *meldetext;
    lv_obj_t *label_target;
    lv_obj_t *homing_led;
    lv_obj_t *fast_menue_1;
    lv_obj_t *antenna_1;
    lv_obj_t *antenna_1_label;
    lv_obj_t *antenna_2;
    lv_obj_t *antenna_2_label;
    lv_obj_t *antenna_3;
    lv_obj_t *antenna_3_label;
    lv_obj_t *fast_menue;
    lv_obj_t *speed_anzeige;
    lv_obj_t *label_actual_1;
    lv_obj_t *speed_slider;
    lv_obj_t *ref_label;
    lv_obj_t *temperaturen_wind;
    lv_obj_t *label_wind_speed;
    lv_obj_t *pfeil_wind;
    lv_obj_t *label_aussen_temp;
    lv_obj_t *engine_temperature;
    lv_obj_t *wind_speed;
} objects_t;

extern objects_t objects;

void create_screen_main();
void tick_screen_main();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/