#ifndef DEVICE_SETTINGS_HPP_
#define DEVICE_SETTINGS_HPP_

#include <Arduino.h>

#include <lvgl.h>
#include <misc/lv_event.h>

#include <cstddef>


struct dev_settings {

    static lv_obj_t *scr;

    static void ta_event_cb(lv_event_t * e) {
        lv_obj_t *ta = (lv_obj_t*)lv_event_get_target(e);
        Serial.printf("Textarea event %X, text='%s'\n",
            lv_event_get_code(e),  lv_textarea_get_text(ta));
    }

    static void bt_dfu_cb(lv_event_t * e) {
        Serial.println("DFU clicked");
    }

    static void bt_back_cb(lv_event_t * e) {
        Serial.println("back");
    }

    static void bt_save_cb(lv_event_t * e) {
        Serial.println("Save");
    }

    static lv_obj_t* init() {
        lv_group_t *g = lv_group_create();
        lv_group_set_default(g);

        scr = lv_obj_create(nullptr);
        lv_obj_set_layout(scr, LV_LAYOUT_FLEX);

        lv_obj_t * lbl;
        lbl = lv_label_create(scr);
        lv_label_set_text(lbl, "Device name");

        lv_obj_t * ta_devname;
        ta_devname = lv_textarea_create(scr);
        // LV_EVENT_READY
        lv_obj_add_event_cb(ta_devname, ta_event_cb, LV_EVENT_ALL, nullptr);
        lv_obj_set_size(ta_devname, 140, 80);
        //lv_keyboard_set_textarea(kb, ta1);

        lv_obj_t *b;
        lv_obj_t * lbl;

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


};

#endif // DEVICE_SETTINGS_HPP_
