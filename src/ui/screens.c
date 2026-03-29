#include <string.h>

#include "screens.h"
#include "images.h"
#include "fonts.h"
#include "actions.h"
#include "vars.h"
#include "styles.h"
#include "ui.h"

#include <string.h>

objects_t objects;

static const char *screen_names[] = { "Main" };
static const char *object_names[] = { "main", "kompass_bg", "grad_acc", "hauptanzeige", "position", "label_gradzeichen_a", "label_gradzeichen_t", "actual_dg", "taget_dg", "label_actual", "meldetext", "label_target", "homing_led", "fast_menue", "speed_anzeige", "label_actual_1", "speed_slider", "ref", "ref_label", "fast_menue_1", "antenna_1", "antenna_1_label", "antenna_2", "antenna_2_label", "antenna_3", "antenna_3_label", "temperaturen_wind", "label_wind_speed", "pfeil_wind", "label_aussen_temp", "engine_temperature", "wind_speed" };

//
// Event handlers
//

lv_obj_t *tick_value_change_obj;

//
// Screens
//

void create_screen_main() {
    void *flowState = getFlowState(0, 0);
    (void)flowState;
    lv_obj_t *obj = lv_obj_create(0);
    objects.main = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 466, 466);
    {
        lv_obj_t *parent_obj = obj;
        {
            // Kompass_BG
            lv_obj_t *obj = lv_img_create(parent_obj);
            objects.kompass_bg = obj;
            lv_obj_set_pos(obj, 0, 0);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_img_set_src(obj, "S:/img/ui_image_kompass_bg.bin");
            lv_img_set_size_mode(obj, LV_IMG_SIZE_MODE_REAL);
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_ADV_HITTEST|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
        }
        {
            // Grad_acc
            lv_obj_t *obj = lv_arc_create(parent_obj);
            objects.grad_acc = obj;
            lv_obj_set_pos(obj, 14, 15);
            lv_obj_set_size(obj, 438, 437);
            lv_arc_set_range(obj, 0, 360); /* 1° pro Einheit; Feinjustierung Soll per Encoder 0,1° */
            lv_arc_set_value(obj, 0);
            lv_arc_set_bg_start_angle(obj, 0);
            lv_arc_set_bg_end_angle(obj, 360);
            lv_arc_set_rotation(obj, 270);
            lv_obj_add_state(obj, LV_STATE_FOCUSED|LV_STATE_FOCUS_KEY);
            lv_obj_set_style_arc_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            // Hauptanzeige
            lv_obj_t *obj = lv_tabview_create(parent_obj, LV_DIR_BOTTOM, 0);
            objects.hauptanzeige = obj;
            lv_obj_set_pos(obj, 88, 90);
            lv_obj_set_size(obj, 291, 287);
            lv_obj_set_scroll_dir(obj, LV_DIR_LEFT);
            lv_obj_set_scroll_snap_x(obj, LV_SCROLL_SNAP_CENTER);
            lv_obj_set_scroll_snap_y(obj, LV_SCROLL_SNAP_NONE);
            add_style_runde_tabview(obj);
            lv_obj_set_style_radius(obj, 140, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_clip_corner(obj, true, LV_PART_MAIN | LV_STATE_DEFAULT);
            {
                lv_obj_t *parent_obj = obj;
                {
                    // Position
                    lv_obj_t *obj = lv_tabview_add_tab(parent_obj, "Tab");
                    objects.position = obj;
                    lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_OFF);
                    lv_obj_set_scroll_dir(obj, LV_DIR_HOR);
                    add_style_tabs(obj);
                    {
                        lv_obj_t *parent_obj = obj;
                        {
                            // Label Gradzeichen A
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_gradzeichen_a = obj;
                            lv_obj_set_pos(obj, 201, 175);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "°");
                        }
                        {
                            // Label Gradzeichen T
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_gradzeichen_t = obj;
                            lv_obj_set_pos(obj, 201, 41);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "°");
                        }
                        {
                            // Actual DG
                            lv_obj_t *obj = lv_textarea_create(parent_obj);
                            objects.actual_dg = obj;
                            lv_obj_set_pos(obj, 48, 175);
                            lv_obj_set_size(obj, 150, 43);
                            lv_textarea_set_accepted_chars(obj, "1234567890,.");
                            lv_textarea_set_max_length(obj, 6);
                            lv_textarea_set_text(obj, "360");
                            lv_textarea_set_one_line(obj, true);
                            lv_textarea_set_password_mode(obj, false);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE);
                            add_style_grosse_zahlen(obj);
                        }
                        {
                            // Taget_DG
                            lv_obj_t *obj = lv_textarea_create(parent_obj);
                            objects.taget_dg = obj;
                            lv_obj_set_pos(obj, 48, 41);
                            lv_obj_set_size(obj, 151, 43);
                            lv_textarea_set_accepted_chars(obj, "1234567890,.");
                            lv_textarea_set_max_length(obj, 6);
                            lv_textarea_set_text(obj, "360");
                            lv_textarea_set_placeholder_text(obj, "360");
                            lv_textarea_set_one_line(obj, true);
                            lv_textarea_set_password_mode(obj, false);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE);
                            add_style_grosse_zahlen(obj);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_radius(obj, 30, LV_PART_MAIN | LV_STATE_EDITED);
                            lv_obj_set_style_clip_corner(obj, true, LV_PART_MAIN | LV_STATE_EDITED);
                        }
                        {
                            // Label Actual
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_actual = obj;
                            lv_obj_set_pos(obj, 19, 183);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "A");
                        }
                        {
                            // Meldetext
                            lv_obj_t *obj = lv_textarea_create(parent_obj);
                            objects.meldetext = obj;
                            lv_obj_set_pos(obj, -5, 112);
                            lv_obj_set_size(obj, 257, 35);
                            lv_textarea_set_max_length(obj, 255);
                            lv_textarea_set_text(obj, "Meldungen");
                            lv_textarea_set_placeholder_text(obj, "360");
                            lv_textarea_set_one_line(obj, true);
                            lv_textarea_set_password_mode(obj, false);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_PRESS_LOCK);
                            lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_OFF);
                            lv_obj_set_scroll_dir(obj, LV_DIR_LEFT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_top(obj, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_bottom(obj, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_left(obj, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_right(obj, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            // Label Target
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_target = obj;
                            lv_obj_set_pos(obj, 19, 50);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "T");
                        }
                        {
                            // Homing Led
                            lv_obj_t *obj = lv_led_create(parent_obj);
                            objects.homing_led = obj;
                            lv_obj_set_pos(obj, 111, 3);
                            lv_obj_set_size(obj, 24, 23);
                            lv_led_set_color(obj, lv_color_hex(0xff43b302));
                            lv_led_set_brightness(obj, 255);
                        }
                    }
                }
                {
                    // Fast Menue
                    lv_obj_t *obj = lv_tabview_add_tab(parent_obj, "Tab");
                    objects.fast_menue = obj;
                    lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_OFF);
                    lv_obj_set_scroll_dir(obj, LV_DIR_HOR);
                    add_style_tabs(obj);
                    lv_obj_set_style_radius(obj, 60, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_clip_corner(obj, true, LV_PART_MAIN | LV_STATE_DEFAULT);
                    {
                        lv_obj_t *parent_obj = obj;
                        {
                            // Speed_Anzeige
                            lv_obj_t *obj = lv_textarea_create(parent_obj);
                            objects.speed_anzeige = obj;
                            lv_obj_set_pos(obj, 44, 50);
                            lv_obj_set_size(obj, 150, 35);
                            lv_textarea_set_accepted_chars(obj, "1234567890");
                            lv_textarea_set_max_length(obj, 3);
                            lv_textarea_set_text(obj, "100");
                            lv_textarea_set_placeholder_text(obj, "100");
                            lv_textarea_set_one_line(obj, true);
                            lv_textarea_set_password_mode(obj, false);
                            add_style_grosse_zahlen(obj);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            // Label Actual_1
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_actual_1 = obj;
                            lv_obj_set_pos(obj, 202, 54);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "%");
                        }
                        {
                            // Speed_Slider
                            lv_obj_t *obj = lv_slider_create(parent_obj);
                            objects.speed_slider = obj;
                            lv_obj_set_pos(obj, 0, 117);
                            lv_obj_set_size(obj, 248, 10);
                            lv_slider_set_value(obj, 100, LV_ANIM_OFF);
                        }
                        {
                            // REF
                            lv_obj_t *obj = lv_btn_create(parent_obj);
                            objects.ref = obj;
                            lv_obj_set_pos(obj, 44, 180);
                            lv_obj_set_size(obj, 158, 44);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    // REF LABEL
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    objects.ref_label = obj;
                                    lv_obj_set_pos(obj, 0, -1);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text(obj, "Homing");
                                }
                            }
                        }
                    }
                }
                {
                    // Fast Menue_1
                    lv_obj_t *obj = lv_tabview_add_tab(parent_obj, "Tab");
                    objects.fast_menue_1 = obj;
                    lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_OFF);
                    lv_obj_set_scroll_dir(obj, LV_DIR_HOR);
                    add_style_tabs(obj);
                    lv_obj_set_style_radius(obj, 60, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_clip_corner(obj, true, LV_PART_MAIN | LV_STATE_DEFAULT);
                    {
                        lv_obj_t *parent_obj = obj;
                        {
                            // Antenna 1
                            lv_obj_t *obj = lv_btn_create(parent_obj);
                            objects.antenna_1 = obj;
                            lv_obj_set_pos(obj, 28, 23);
                            lv_obj_set_size(obj, 185, 44);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff2196f3), LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    // Antenna 1 label
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    objects.antenna_1_label = obj;
                                    lv_obj_set_pos(obj, 0, 0);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text(obj, "KW Beam");
                                }
                            }
                        }
                        {
                            // Antenna 2
                            lv_obj_t *obj = lv_btn_create(parent_obj);
                            objects.antenna_2 = obj;
                            lv_obj_set_pos(obj, 28, 100);
                            lv_obj_set_size(obj, 185, 44);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    // Antenna 2 label
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    objects.antenna_2_label = obj;
                                    lv_obj_set_pos(obj, 0, 1);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text(obj, "2m / 70cm");
                                }
                            }
                        }
                        {
                            // Antenna 3
                            lv_obj_t *obj = lv_btn_create(parent_obj);
                            objects.antenna_3 = obj;
                            lv_obj_set_pos(obj, 28, 174);
                            lv_obj_set_size(obj, 185, 44);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    // Antenna 3 label
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    objects.antenna_3_label = obj;
                                    lv_obj_set_pos(obj, 0, 0);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text(obj, "23 cm");
                                }
                            }
                        }
                    }
                }
                {
                    // Temperaturen_Wind
                    lv_obj_t *obj = lv_tabview_add_tab(parent_obj, "Tab");
                    objects.temperaturen_wind = obj;
                    lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_OFF);
                    lv_obj_set_scroll_dir(obj, LV_DIR_HOR);
                    add_style_tabs(obj);
                    lv_obj_set_style_radius(obj, 60, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_clip_corner(obj, true, LV_PART_MAIN | LV_STATE_DEFAULT);
                    {
                        lv_obj_t *parent_obj = obj;
                        {
                            // Label Wind Speed
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_wind_speed = obj;
                            lv_obj_set_pos(obj, 151, 30);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "kmh");
                        }
                        {
                            // pfeilWind
                            lv_obj_t *obj = lv_img_create(parent_obj);
                            objects.pfeil_wind = obj;
                            lv_obj_set_pos(obj, 91, 77);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_img_set_src(obj, "S:/img/ui_image_pfeil_wind.bin");
                            lv_img_set_size_mode(obj, LV_IMG_SIZE_MODE_REAL);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICK_FOCUSABLE);
                        }
                        {
                            // Label aussen temp
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.label_aussen_temp = obj;
                            lv_obj_set_pos(obj, 152, 215);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "°C");
                        }
                        {
                            // Engine Temperature
                            lv_obj_t *obj = lv_textarea_create(parent_obj);
                            objects.engine_temperature = obj;
                            lv_obj_set_pos(obj, 71, 211);
                            lv_obj_set_size(obj, 69, 37);
                            lv_textarea_set_max_length(obj, 128);
                            lv_textarea_set_text(obj, "25.1");
                            lv_textarea_set_one_line(obj, true);
                            lv_textarea_set_password_mode(obj, false);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE);
                            add_style_grosse_zahlen(obj);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            // WindSpeed
                            lv_obj_t *obj = lv_textarea_create(parent_obj);
                            objects.wind_speed = obj;
                            lv_obj_set_pos(obj, 43, 26);
                            lv_obj_set_size(obj, 97, 37);
                            lv_textarea_set_max_length(obj, 5);
                            lv_textarea_set_text(obj, "33.4");
                            lv_textarea_set_placeholder_text(obj, "25.1");
                            lv_textarea_set_one_line(obj, true);
                            lv_textarea_set_password_mode(obj, false);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE);
                            add_style_grosse_zahlen(obj);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                    }
                }
            }
        }
    }
    
    tick_screen_main();
}

void tick_screen_main() {
    void *flowState = getFlowState(0, 0);
    (void)flowState;
}

typedef void (*tick_screen_func_t)();
tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_main,
};
void tick_screen(int screen_index) {
    tick_screen_funcs[screen_index]();
}
void tick_screen_by_id(enum ScreensEnum screenId) {
    tick_screen_funcs[screenId - 1]();
}

//
// Styles
//

static const char *style_names[] = { "Grosse_Zahlen", "Runde Tabview", "Tabs", "Menue_Tabs", "Menue_Tabview" };

extern void add_style(lv_obj_t *obj, int32_t styleIndex);
extern void remove_style(lv_obj_t *obj, int32_t styleIndex);

//
// Fonts
//

ext_font_desc_t fonts[] = {
#if LV_FONT_MONTSERRAT_8
    { "MONTSERRAT_8", &lv_font_montserrat_8 },
#endif
#if LV_FONT_MONTSERRAT_10
    { "MONTSERRAT_10", &lv_font_montserrat_10 },
#endif
#if LV_FONT_MONTSERRAT_12
    { "MONTSERRAT_12", &lv_font_montserrat_12 },
#endif
#if LV_FONT_MONTSERRAT_14
    { "MONTSERRAT_14", &lv_font_montserrat_14 },
#endif
#if LV_FONT_MONTSERRAT_16
    { "MONTSERRAT_16", &lv_font_montserrat_16 },
#endif
#if LV_FONT_MONTSERRAT_18
    { "MONTSERRAT_18", &lv_font_montserrat_18 },
#endif
#if LV_FONT_MONTSERRAT_20
    { "MONTSERRAT_20", &lv_font_montserrat_20 },
#endif
#if LV_FONT_MONTSERRAT_22
    { "MONTSERRAT_22", &lv_font_montserrat_22 },
#endif
#if LV_FONT_MONTSERRAT_24
    { "MONTSERRAT_24", &lv_font_montserrat_24 },
#endif
#if LV_FONT_MONTSERRAT_26
    { "MONTSERRAT_26", &lv_font_montserrat_26 },
#endif
#if LV_FONT_MONTSERRAT_28
    { "MONTSERRAT_28", &lv_font_montserrat_28 },
#endif
#if LV_FONT_MONTSERRAT_30
    { "MONTSERRAT_30", &lv_font_montserrat_30 },
#endif
#if LV_FONT_MONTSERRAT_32
    { "MONTSERRAT_32", &lv_font_montserrat_32 },
#endif
#if LV_FONT_MONTSERRAT_34
    { "MONTSERRAT_34", &lv_font_montserrat_34 },
#endif
#if LV_FONT_MONTSERRAT_36
    { "MONTSERRAT_36", &lv_font_montserrat_36 },
#endif
#if LV_FONT_MONTSERRAT_38
    { "MONTSERRAT_38", &lv_font_montserrat_38 },
#endif
#if LV_FONT_MONTSERRAT_40
    { "MONTSERRAT_40", &lv_font_montserrat_40 },
#endif
#if LV_FONT_MONTSERRAT_42
    { "MONTSERRAT_42", &lv_font_montserrat_42 },
#endif
#if LV_FONT_MONTSERRAT_44
    { "MONTSERRAT_44", &lv_font_montserrat_44 },
#endif
#if LV_FONT_MONTSERRAT_46
    { "MONTSERRAT_46", &lv_font_montserrat_46 },
#endif
#if LV_FONT_MONTSERRAT_48
    { "MONTSERRAT_48", &lv_font_montserrat_48 },
#endif
};

//
//
//

void create_screens() {
    // Initialize styles
    eez_flow_init_styles(add_style, remove_style);
    eez_flow_init_style_names(style_names, sizeof(style_names) / sizeof(const char *));

eez_flow_init_fonts(fonts, sizeof(fonts) / sizeof(ext_font_desc_t));

// Set default LVGL theme
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    
    // Initialize screens
    eez_flow_init_screen_names(screen_names, sizeof(screen_names) / sizeof(const char *));
    eez_flow_init_object_names(object_names, sizeof(object_names) / sizeof(const char *));
    
    // Create screens
    create_screen_main();
}