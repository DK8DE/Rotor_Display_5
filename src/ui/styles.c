#include "styles.h"
#include "images.h"
#include "fonts.h"

#include "ui.h"
#include "screens.h"

//
// Style: Grosse_Zahlen
//

void init_style_grosse_zahlen_MAIN_DEFAULT(lv_style_t *style) {
    lv_style_set_text_font(style, &lv_font_montserrat_32);
    lv_style_set_pad_top(style, 2);
    lv_style_set_pad_bottom(style, 2);
    lv_style_set_pad_left(style, 2);
    lv_style_set_pad_right(style, 2);
    lv_style_set_text_align(style, LV_TEXT_ALIGN_CENTER);
};

lv_style_t *get_style_grosse_zahlen_MAIN_DEFAULT() {
    static lv_style_t *style;
    if (!style) {
        style = (lv_style_t *)lv_mem_alloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_grosse_zahlen_MAIN_DEFAULT(style);
    }
    return style;
};

void init_style_grosse_zahlen_MAIN_FOCUS_KEY(lv_style_t *style) {
    lv_style_set_text_font(style, &lv_font_montserrat_32);
};

lv_style_t *get_style_grosse_zahlen_MAIN_FOCUS_KEY() {
    static lv_style_t *style;
    if (!style) {
        style = (lv_style_t *)lv_mem_alloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_grosse_zahlen_MAIN_FOCUS_KEY(style);
    }
    return style;
};

void add_style_grosse_zahlen(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_grosse_zahlen_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_style(obj, get_style_grosse_zahlen_MAIN_FOCUS_KEY(), LV_PART_MAIN | LV_STATE_FOCUS_KEY);
};

void remove_style_grosse_zahlen(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_grosse_zahlen_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_remove_style(obj, get_style_grosse_zahlen_MAIN_FOCUS_KEY(), LV_PART_MAIN | LV_STATE_FOCUS_KEY);
};

//
// Style: Runde Tabview
//

void init_style_runde_tabview_MAIN_SCROLLED(lv_style_t *style) {
    lv_style_set_radius(style, 50);
    lv_style_set_clip_corner(style, true);
};

lv_style_t *get_style_runde_tabview_MAIN_SCROLLED() {
    static lv_style_t *style;
    if (!style) {
        style = (lv_style_t *)lv_mem_alloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_runde_tabview_MAIN_SCROLLED(style);
    }
    return style;
};

void add_style_runde_tabview(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_runde_tabview_MAIN_SCROLLED(), LV_PART_MAIN | LV_STATE_SCROLLED);
};

void remove_style_runde_tabview(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_runde_tabview_MAIN_SCROLLED(), LV_PART_MAIN | LV_STATE_SCROLLED);
};

//
// Style: Tabs
//

void init_style_tabs_MAIN_FOCUS_KEY(lv_style_t *style) {
    lv_style_set_radius(style, 50);
    lv_style_set_clip_corner(style, true);
};

lv_style_t *get_style_tabs_MAIN_FOCUS_KEY() {
    static lv_style_t *style;
    if (!style) {
        style = (lv_style_t *)lv_mem_alloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_tabs_MAIN_FOCUS_KEY(style);
    }
    return style;
};

void init_style_tabs_MAIN_SCROLLED(lv_style_t *style) {
    lv_style_set_radius(style, 60);
    lv_style_set_clip_corner(style, true);
    lv_style_set_border_post(style, false);
};

lv_style_t *get_style_tabs_MAIN_SCROLLED() {
    static lv_style_t *style;
    if (!style) {
        style = (lv_style_t *)lv_mem_alloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_tabs_MAIN_SCROLLED(style);
    }
    return style;
};

void init_style_tabs_MAIN_DEFAULT(lv_style_t *style) {
    lv_style_set_radius(style, 150);
    lv_style_set_clip_corner(style, true);
};

lv_style_t *get_style_tabs_MAIN_DEFAULT() {
    static lv_style_t *style;
    if (!style) {
        style = (lv_style_t *)lv_mem_alloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_tabs_MAIN_DEFAULT(style);
    }
    return style;
};

void add_style_tabs(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_tabs_MAIN_FOCUS_KEY(), LV_PART_MAIN | LV_STATE_FOCUS_KEY);
    lv_obj_add_style(obj, get_style_tabs_MAIN_SCROLLED(), LV_PART_MAIN | LV_STATE_SCROLLED);
    lv_obj_add_style(obj, get_style_tabs_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

void remove_style_tabs(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_tabs_MAIN_FOCUS_KEY(), LV_PART_MAIN | LV_STATE_FOCUS_KEY);
    lv_obj_remove_style(obj, get_style_tabs_MAIN_SCROLLED(), LV_PART_MAIN | LV_STATE_SCROLLED);
    lv_obj_remove_style(obj, get_style_tabs_MAIN_DEFAULT(), LV_PART_MAIN | LV_STATE_DEFAULT);
};

//
// Style: Menue_Tabs
//

void init_style_menue_tabs_MAIN_SCROLLED(lv_style_t *style) {
    lv_style_set_radius(style, 200);
    lv_style_set_clip_corner(style, true);
};

lv_style_t *get_style_menue_tabs_MAIN_SCROLLED() {
    static lv_style_t *style;
    if (!style) {
        style = (lv_style_t *)lv_mem_alloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_menue_tabs_MAIN_SCROLLED(style);
    }
    return style;
};

void add_style_menue_tabs(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_menue_tabs_MAIN_SCROLLED(), LV_PART_MAIN | LV_STATE_SCROLLED);
};

void remove_style_menue_tabs(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_menue_tabs_MAIN_SCROLLED(), LV_PART_MAIN | LV_STATE_SCROLLED);
};

//
// Style: Menue_Tabview
//

void init_style_menue_tabview_MAIN_SCROLLED(lv_style_t *style) {
    lv_style_set_radius(style, 200);
    lv_style_set_clip_corner(style, true);
};

lv_style_t *get_style_menue_tabview_MAIN_SCROLLED() {
    static lv_style_t *style;
    if (!style) {
        style = (lv_style_t *)lv_mem_alloc(sizeof(lv_style_t));
        lv_style_init(style);
        init_style_menue_tabview_MAIN_SCROLLED(style);
    }
    return style;
};

void add_style_menue_tabview(lv_obj_t *obj) {
    (void)obj;
    lv_obj_add_style(obj, get_style_menue_tabview_MAIN_SCROLLED(), LV_PART_MAIN | LV_STATE_SCROLLED);
};

void remove_style_menue_tabview(lv_obj_t *obj) {
    (void)obj;
    lv_obj_remove_style(obj, get_style_menue_tabview_MAIN_SCROLLED(), LV_PART_MAIN | LV_STATE_SCROLLED);
};

//
//
//

void add_style(lv_obj_t *obj, int32_t styleIndex) {
    typedef void (*AddStyleFunc)(lv_obj_t *obj);
    static const AddStyleFunc add_style_funcs[] = {
        add_style_grosse_zahlen,
        add_style_runde_tabview,
        add_style_tabs,
        add_style_menue_tabs,
        add_style_menue_tabview,
    };
    add_style_funcs[styleIndex](obj);
}

void remove_style(lv_obj_t *obj, int32_t styleIndex) {
    typedef void (*RemoveStyleFunc)(lv_obj_t *obj);
    static const RemoveStyleFunc remove_style_funcs[] = {
        remove_style_grosse_zahlen,
        remove_style_runde_tabview,
        remove_style_tabs,
        remove_style_menue_tabs,
        remove_style_menue_tabview,
    };
    remove_style_funcs[styleIndex](obj);
}