#include <type_traits>

#include <Arduino.h>

#include <lvgl.h>
#include <misc/lv_event.h>
#include <TFT_eSPI.h>

#include <NimBLEDevice.h>

#include <etl/debounce.h>
#include <etl/vector.h>
#include <etl/array.h>
#include <etl/bitset.h>
#include <etl/queue.h>
#include <etl/utility.h>

#include "device_settings.hpp"
#include "ble.h"

etl::vector<NimBLEAdvertisedDevice*, 5> found_devs;

#define TFT_HOR_RES   TFT_WIDTH
#define TFT_VER_RES   TFT_HEIGHT
#define TFT_ROTATION  LV_DISPLAY_ROTATION_270
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

uint8_t rem_batt_val = 0;

static lv_indev_t * indev;

static lv_obj_t *list_devs;
static lv_obj_t *statusbar;
static lv_obj_t *lb_batt_intl;
static lv_obj_t *im_batt_intl;
static lv_obj_t *lb_batt_rem;
static lv_obj_t *im_batt_rem;
static lv_obj_t *im_bt;

static lv_obj_t *pnl_inputs;

static lv_obj_t *scr_devices;
static lv_obj_t *scr_control;
static lv_obj_t *scr_dev_settings;
static etl::array<lv_obj_t*, 4> bt_functions;

static void update_connected(bool c) {
    if(c) {
        found_devs.clear();
        lv_obj_remove_flag(im_bt, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(lb_batt_rem, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(im_batt_rem, LV_OBJ_FLAG_HIDDEN);
        //lv_screen_load(scr_control);
        lv_screen_load_anim(scr_devices, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0, false);
    } else {
        ble::start_scan();
        lv_obj_add_flag(im_bt, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(lb_batt_rem, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(im_batt_rem, LV_OBJ_FLAG_HIDDEN);

        lv_obj_clean(list_devs);
        lv_list_add_text(list_devs, "Receivers found:");

        //lv_screen_load(scr_devices);
        lv_screen_load_anim(scr_devices, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0, false);
    }
}

static void scr_load_cb(lv_event_t * e) {
    lv_group_t* g = (lv_group_t*)lv_event_get_user_data(e);
    lv_indev_set_group(indev, g);
    lv_group_set_default(g);

    // Serial.printf("group cnt:%d, active: %X\n",
    //     lv_group_get_obj_count(g),
    //     lv_group_get_focused(g));

}


static void on_dev_selected(lv_event_t * e) {
    NimBLEAdvertisedDevice *dev = (NimBLEAdvertisedDevice*)lv_event_get_user_data(e);
    ble::connect(dev);
    update_connected(true);
}

void on_dev_found(NimBLEAdvertisedDevice *dev) {
    //Serial.printf("found: %s (%s)\n",  dev->getName().c_str(), dev->getAddress().toString().c_str());

    if(found_devs.full()) return;

    found_devs.push_back(dev);

    const char* name = dev->getName().c_str();
    lv_obj_t *btn;
    if(strcmp(name, "MicroRC") == 0) {
        char msg[50];
        snprintf(msg, sizeof(msg), "%s (%s)",
            name,
            dev->getAddress().toString().c_str()
        );
        btn = lv_list_add_button(list_devs, nullptr, msg);
    } else {
        btn = lv_list_add_button(list_devs, nullptr, name);
    }
    lv_obj_add_event_cb(btn, on_dev_selected, LV_EVENT_CLICKED, dev);
}

void on_battery_updated(uint8_t val) {
    rem_batt_val = val;
}

void on_dev_disconnected(NimBLEClient *dev) {
    Serial.print(dev->getPeerAddress().toString().c_str());
    Serial.println(" Disconnected; starting scan");

    update_connected(false);
}

void disconnect_request_cb(lv_event_t * e) {
    ble::disconnect();
}

constexpr auto PIN_ANALOG = etl::make_array<int>(33, 32, 39, 38);
constexpr auto PIN_JX = PIN_ANALOG[0];
constexpr auto PIN_JY = PIN_ANALOG[1];
constexpr int PIN_J = 25;
constexpr auto PIN_HAT = etl::make_array<int>(37, 36, 15, 13, 12);
constexpr auto HAT_MAP = etl::make_array<int>(LV_KEY_DOWN, LV_KEY_ENTER, LV_KEY_LEFT, LV_KEY_UP, LV_KEY_RIGHT);
constexpr auto PIN_SWITCHES = etl::make_array<int>(2, 17, 22, 21);
constexpr int PIN_BLINKER = PIN_SWITCHES[3];



static void inputdev_cb(lv_indev_t *indev, lv_indev_data_t *data ) {
    static int last_key = 0;
    int key_idx = -1;
    for(size_t i=0; i<PIN_HAT.size(); i++) {
        if(digitalRead(PIN_HAT[i]) == LOW) key_idx = i;
    }
    if (key_idx!=-1) {
        last_key = HAT_MAP[key_idx];
        data->state = LV_INDEV_STATE_PRESSED;
    }  else data->state = LV_INDEV_STATE_RELEASED;
    data->key = last_key;
}

static uint32_t my_tick(void) {
    return millis();
}


void fn_cb(lv_event_t *e) {
    lv_obj_t* obj = (lv_obj_t*)lv_event_get_target(e);
    int fn = (int)lv_event_get_user_data(e);
    if(fn==0) return;
    bool v = lv_obj_has_state(obj, LV_STATE_CHECKED);
    char msg[32];
    snprintf(msg, sizeof(msg), "%d=%d\n", fn, v?255:0);
    ble::send(msg);
}

#if LV_USE_LOG != 0
void my_print( lv_log_level_t level, const char * buf )
{
    LV_UNUSED(level);
    Serial.print(buf);
    Serial.flush();
}
#endif

//#define VBAT  10

void setup () {
    Serial.begin(115200);
    Serial.println("Starting BLE Transmitter");

    analogReadResolution(10);
    // pinMode(LEFT_BUTTON, INPUT_PULLUP);
    // pinMode(RIGHT_BUTTON, INPUT_PULLUP);

    for(auto p: PIN_HAT) pinMode(p, INPUT);
    for(auto p: PIN_SWITCHES) pinMode(p, INPUT_PULLUP);
    pinMode(PIN_J, INPUT);

    lv_init();
    lv_tick_set_cb(my_tick); // millis

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print );
#endif

#if LV_USE_TFT_ESPI
    lv_display_t * disp = lv_tft_espi_create(TFT_HOR_RES, TFT_VER_RES, draw_buf, sizeof(draw_buf));
    lv_display_set_rotation(disp, TFT_ROTATION);
#else
    #error "TFT_eSPI nto set"
#endif

    indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_read_cb(indev, inputdev_cb);
    // lv_indev_set_group(indev, g);

    statusbar = lv_layer_top();
    lv_obj_set_flex_flow(statusbar, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(statusbar, 5, 0);
    lv_obj_set_style_text_color(statusbar, lv_color_hex(0x888888), LV_PART_MAIN);

    im_batt_intl = lv_image_create(statusbar); lv_image_set_src(im_batt_intl, LV_SYMBOL_BATTERY_EMPTY);

    lb_batt_intl = lv_label_create(statusbar);
    im_bt = lv_image_create(statusbar);  lv_image_set_src(im_bt, LV_SYMBOL_BLUETOOTH);
    im_batt_rem = lv_image_create(statusbar); lv_image_set_src(im_batt_rem, LV_SYMBOL_BATTERY_EMPTY);
    lb_batt_rem = lv_label_create(statusbar);

    //lv_obj_delete(lv_screen_active());

    //####### devices screen

    lv_group_t *g = lv_group_create();
    lv_group_set_default(g);
    scr_devices = lv_screen_active();//lv_obj_create(nullptr);

    lv_obj_set_style_pad_top(scr_devices, 18, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(scr_devices, 5, LV_PART_MAIN);
    lv_obj_set_style_pad_left(scr_devices, 5, LV_PART_MAIN);
    lv_obj_set_style_pad_right(scr_devices, 5, LV_PART_MAIN);
    lv_obj_add_event_cb(scr_devices, scr_load_cb, LV_EVENT_SCREEN_LOADED, g);
    // lv_obj_set_user_data(scr_devices, g);

    lv_obj_t * label;

    list_devs = lv_list_create(scr_devices);
    lv_obj_set_size(list_devs, lv_pct(100), lv_pct(100));

    //####### control screen

    g = lv_group_create();
    lv_group_set_default(g);
    scr_control = lv_obj_create(nullptr);

    lv_obj_set_style_pad_top(scr_control, 18, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(scr_control, 5, LV_PART_MAIN);
    lv_obj_set_style_pad_left(scr_control, 5, LV_PART_MAIN);
    lv_obj_set_style_pad_right(scr_control, 5, LV_PART_MAIN);
    lv_obj_add_event_cb(scr_control, scr_load_cb, LV_EVENT_SCREEN_LOADED, g);
    // lv_obj_set_user_data(scr_control, g);

    pnl_inputs = lv_obj_create(scr_control);
    lv_obj_align(pnl_inputs, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_size(pnl_inputs, 100, 100);
    lv_obj_set_scrollbar_mode(pnl_inputs, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *ball = lv_obj_create(pnl_inputs);
    // lv_obj_set_pos(ball, lv_pct(50), lv_pct(50));
    lv_obj_align(ball, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_size(ball, 15, 15);
    lv_obj_set_style_radius(ball , LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(ball, lv_palette_main(LV_PALETTE_CYAN), LV_PART_MAIN);

    lv_obj_t *l = lv_obj_create(scr_control);
    lv_obj_align(l, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_flex_flow(l, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_size(l, 125, lv_pct(100));
    lv_obj_set_style_pad_top(l, 5, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(l, 5, LV_PART_MAIN);

    lv_obj_t *b;

    b = lv_button_create(l);
    lv_label_set_text(lv_label_create(b), "Headlight");
    lv_obj_remove_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_user_data(b, (int*)2);
    bt_functions[0] = b;

    b = lv_button_create(l);
    lv_label_set_text(lv_label_create(b), "Marker");
    lv_obj_remove_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_user_data(b, (int*)3);
    bt_functions[1] = b;

    b = lv_button_create(l);
    lv_label_set_text(lv_label_create(b), "<");
    lv_obj_remove_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CHECKABLE);
    bt_functions[2] = b;
    b = lv_button_create(l);
    lv_label_set_text(lv_label_create(b), ">");
    lv_obj_remove_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CHECKABLE);
    bt_functions[3] = b;

    b = lv_button_create(l);
    lv_image_set_src(lv_image_create(b), LV_SYMBOL_CLOSE);
    lv_obj_add_event_cb(b, disconnect_request_cb, LV_EVENT_CLICKED, nullptr);

    //###### Device settings
    scr_dev_settings = dev_settings::init();

    lv_group_set_default(nullptr);

    ble::init();
    ble::set_dev_found_cb(on_dev_found);
    ble::set_battery_update_cb(on_battery_updated);
    ble::set_disconnected_cb(on_dev_disconnected);
    update_connected(false);

}

/** Rounded division. */
template<typename T, typename U, std::enable_if_t<std::is_unsigned_v<U> >* =nullptr >
T rdiv(const T x, const U y) {
    T quot = x / y;
    T rem = x % y;
    U ty = y-1;
    if (x >= 0) {
        return quot + (rem > (ty/2));
    } else {
        return quot - (rem < (-ty/2));
    }
}

int16_t to_centered(const uint16_t v, const uint16_t center = 512, const uint16_t deadzone = 0) {
    //constexpr uint8_t center = 512;
    constexpr uint16_t in_range = 512;
    const uint16_t out_range = in_range - deadzone;
    int16_t ret = v - center;
    if(deadzone != 0) {
        uint16_t mag = abs(ret);
        if(mag < deadzone) return 0;
        else return (ret>0?1:-1) * rdiv((mag-deadzone)*in_range, out_range);
    } else {
        return ret;
    }
}


/** Returns -1, 0, 1 if pin is LOW, floating and HIGH. */
int read_tristate(int pin) {
    pinMode(pin, INPUT_PULLUP);
    int v1 = digitalRead(pin);
    pinMode(pin, INPUT_PULLDOWN);
    int v2 = digitalRead(pin);
    // Serial.printf("v1=%d v2=%d\n", v1, v2);
    if(v1==v2) return v1 == LOW ? -1 : 1;
    return 0;
}

void set_checked_state(lv_obj_t *bt, bool checked) {
    bool cur = lv_obj_has_state(bt, LV_STATE_CHECKED);
    if(cur!=checked) {
        lv_obj_set_state(bt, LV_STATE_CHECKED, checked);

        int fn = (int)lv_obj_get_user_data(bt);
        if(fn!=0) {
            char msg[32];
            snprintf(msg, sizeof(msg), "%d=%d\n", fn, checked?255:0);
            ble::send(msg);
        }

        //lv_obj_send_event(bt, LV_EVENT_VALUE_CHANGED, nullptr);
    }
}

void read_controls_input() {
    int x = analogRead(PIN_JX);
    int y = analogRead(PIN_JY);
    bool down = digitalRead(PIN_J);
    x = -to_centered(x, 465, 10);
    y = to_centered(y, 445, 10);

    static int last_x=-1000, last_y=0;
    if(x!=last_x || y!=last_y) {
        char msg[32];
        int sx = 128 + x*128/512, sy = 128 + y*128/512;
        sx = constrain(sx, 0, 255);
        sy = constrain(sy, 0, 255);
        snprintf(msg, sizeof(msg), "1=%d\n0=%d\n",
            sx,
            sy
        );
        ble::send(msg);

        // lv_label_set_text(pnl_inputs, msg);
        lv_obj_t *ball = lv_obj_get_child(pnl_inputs, 0);
        lv_obj_set_pos(ball, x*50/512, - y*50/512);

        // snprintf(msg, sizeof(msg), "%d/%d",
        //     50 + x*50/512,
        //     50 - y*50/512
        // );
        // lv_label_set_text(lb_batt_rem, msg);
    }
    last_x = x;
    last_y = y;

    bool st = digitalRead(PIN_SWITCHES[1]) == LOW;
    set_checked_state(bt_functions[0], st);

    st = digitalRead(PIN_SWITCHES[2]) == LOW;
    set_checked_state(bt_functions[1], st);

    int t = read_tristate(PIN_BLINKER);
    if(t>0) {
        set_checked_state(bt_functions[3], true);
    } else if(t<0) {
        set_checked_state(bt_functions[2], true);
    } else {
        set_checked_state(bt_functions[3], false);
        set_checked_state(bt_functions[2], false);
    }
}

void draw_batteries() {
    constexpr int ADC2 = 580, V2 = 4200,
        ADC1 = 428, V1 = 3200;
    int int_batt = map(analogRead(VBAT), ADC1, ADC2, V1, V2);

    char glyph[] = {'\xEF', '\x89', '\x84', 0};

    static int last_int_batt;
    if(last_int_batt != int_batt/100) {
        int idx = map(int_batt, 3300, 4000, 0, 4);
        idx = constrain(idx, 0, 4);
        glyph[2] = '\x84' - idx;
        lv_image_set_src(im_batt_intl, glyph);
        lv_label_set_text_fmt(lb_batt_intl, "%dmV ", int_batt);
        last_int_batt = int_batt/100;
    }

    static int last_rem_batt=-1;
    if(ble::is_connected()) {
        if(last_rem_batt != rem_batt_val) {
            int idx = map(rem_batt_val, 0, 100, 0, 4);
            idx = constrain(idx, 0, 4);
            glyph[2] = '\x84' - idx;
            lv_image_set_src(im_batt_rem, glyph);
            lv_label_set_text_fmt(lb_batt_rem, "%d%%", rem_batt_val);
            last_rem_batt = rem_batt_val;
        }
    }
}


void loop () {

    static size_t last_t = 0;
    static bool fn_lights = false;
    if(millis() - last_t > 1000) {
        last_t = millis();
        draw_batteries();
    }

    if(ble::is_connected()) {
        read_controls_input();
    }

    lv_timer_handler();

    delay(5);

}
