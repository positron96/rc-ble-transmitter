#ifndef DEVICE_SETTINGS_HPP_
#define DEVICE_SETTINGS_HPP_

#include <Arduino.h>

#include "ble.h"

#include <lvgl.h>
#include <misc/lv_event.h>

#include <cstddef>


struct dev_settings {

    static lv_obj_t *scr;



    static lv_obj_t* init() {
        lv_group_t *g = lv_group_create();
        lv_group_set_default(g);

        scr = lv_obj_create(nullptr);
        lv_obj_set_layout(scr, LV_FLEX_FLOW_ROW_WRAP);

        lv_obj_set_style_pad_top(scr, lv_obj_get_height(lv_layer_top())+3, LV_PART_MAIN);
        lv_obj_set_style_pad_bottom(scr_devices, 5, LV_PART_MAIN);
        lv_obj_set_style_pad_left(scr_devices, 5, LV_PART_MAIN);
        lv_obj_set_style_pad_right(scr_devices, 5, LV_PART_MAIN);

        lv_obj_add_event_cb(scr_devices, scr_load_cb, LV_EVENT_SCREEN_LOADED, nullptr);

        lv_obj_t * lbl;
        lbl = lv_label_create(scr);
        lv_label_set_text(lbl, "Device name");

        lv_obj_t * kb = lv_keyboard_create(scr);
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);

        ta_devname = lv_textarea_create(scr);
        lv_obj_add_event_cb(ta_devname, ta_event_cb, LV_EVENT_ALL, kb);
        lv_textarea_set_one_line(ta_devname, true);
        // lv_obj_set_size(ta_devname, 140, 80);

        lv_obj_t *b;

        b = lv_button_create(scr);
        lbl = lv_label_create(b);
        lv_label_set_text(lbl, "Save");
        lv_obj_add_event_cb(b, bt_save_cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_center(lbl);

        b = lv_button_create(scr);
        lbl = lv_label_create(b);
        lv_label_set_text(lbl, "Reboot to DFU");
        lv_obj_add_event_cb(b, bt_dfu_cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_center(lbl);

        b = lv_button_create(scr);
        //lbl = lv_label_create(b);
        //lv_label_set_text(lbl, "Back");
        lv_image_set_src(lv_image_create(b), LV_SYMBOL_PREV);
        lv_obj_add_event_cb(b, bt_back_cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_center(lbl);

        return scr;
    }

    static lv_obj_t *screen_back;

    private:

    static lv_obj_t * ta_devname;

    static void ta_event_cb(lv_event_t * e) {
        lv_obj_t *ta = (lv_obj_t*)lv_event_get_target(e);
        lv_event_code_t code = lv_event_get_code(e);

        // Serial.printf("Textarea event %X, text='%s'\n",
        //     code,  lv_textarea_get_text(ta));

        lv_indev_t * indev = lv_indev_active();
        if(indev == NULL) return;
        lv_indev_type_t indev_type = lv_indev_get_type(indev);

        lv_obj_t * kb = (lv_obj_t*)lv_event_get_user_data(e);

        if(code == LV_EVENT_CLICKED && indev_type == LV_INDEV_TYPE_ENCODER) {
            lv_keyboard_set_textarea(kb, ta);
            lv_obj_remove_flag(kb, LV_OBJ_FLAG_HIDDEN);
            lv_group_focus_obj(kb);
            lv_group_set_editing(lv_obj_get_group(kb), kb != NULL);
            // lv_obj_set_height(tv, LV_VER_RES / 2);
            lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, 0);
        }

        if(code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
            lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
            // lv_obj_set_height(tv, LV_VER_RES);
        }
    }

    static void bt_dfu_cb(lv_event_t * e) {
        Serial.println("DFU clicked");
        ble::send("!dfu\n");
    }

    static void bt_back_cb(lv_event_t * e) {
        if(screen_back != nullptr) {
            lv_screen_load_anim(screen_back, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
        }
    }

    static void bt_save_cb(lv_event_t * e) {
        const std::string &current_name = ble::get_connected_dev_name();
        if(current_name != lv_textarea_get_text(ta_devname)) {
            Serial.println("Need to save new name");
        }
    }

    static void scr_load_cb(lv_event_t * e) {
        lv_textarea_set_text(ta_devname, ble::get_connected_dev_name().c_str());

    }

};

#endif // DEVICE_SETTINGS_HPP_
