#include <type_traits>

#include <Arduino.h>

#include <lvgl.h>
#include <misc/lv_event.h>
#include <TFT_eSPI.h>

#include <NimBLEDevice.h>

#include <etl/debounce.h>
#include <etl/vector.h>
#include <etl/array.h>

#include "ble.h"

etl::vector<NimBLEAdvertisedDevice*, 5> found_devs;

#define TFT_HOR_RES   TFT_WIDTH
#define TFT_VER_RES   TFT_HEIGHT
#define TFT_ROTATION  LV_DISPLAY_ROTATION_270
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

uint8_t remote_batt_value;

static lv_obj_t *list_devs;
static lv_obj_t *lb_battery;
static lv_obj_t *pnl_inputs;

static lv_obj_t *scr_devices;
static lv_obj_t *scr_control;

static void on_dev_selected(lv_event_t * e) {
    NimBLEAdvertisedDevice *dev = (NimBLEAdvertisedDevice*)lv_event_get_user_data(e);
    ble::connect(dev);
    lv_scr_load(scr_control);
}

void on_dev_found(NimBLEAdvertisedDevice *dev) {
    //Serial.printf("found: %s (%s)\n",  dev->getName().c_str(), dev->getAddress().toString().c_str());

    if(found_devs.full()) return;

    found_devs.push_back(dev);

    char msg[50];
    size_t l = snprintf(
        msg,
        sizeof(msg),
        "%s (%s)",
        dev->getName().c_str(),
        dev->getAddress().toString().c_str());
    lv_obj_t *btn = lv_list_add_button(list_devs, nullptr, msg);
    lv_obj_add_event_cb(btn, on_dev_selected, LV_EVENT_CLICKED, dev);
}

void on_battery_updated(uint8_t val) {
    remote_batt_value = val;
}

void on_dev_disconnected(NimBLEClient *dev) {
    Serial.print(dev->getPeerAddress().toString().c_str());
    Serial.println(" Disconnected; starting scan");

    ble::start_scan();
    lv_screen_load(scr_devices);
}

void disconnect_cb(lv_event_t * e) {
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
    if(key_idx!=-1)
        Serial.printf("%d, %d, %d\n", data->key, data->state, key_idx);
}

static uint32_t my_tick(void) {
    return millis();
}

void scr_load_cb(lv_event_t * e) {
    lv_obj_t *t = lv_event_get_target_obj(e);
    //if(t == scr_devices) {
        found_devs.clear();
        lv_obj_clean(list_devs);
        lv_list_add_text(list_devs, "Receivers found:");
        //lv_group_focus_obj(list_devs);

    //     lv_obj_remove_flag(scr_devices, LV_OBJ_FLAG_HIDDEN);
    //     lv_obj_add_flag(scr_control, LV_OBJ_FLAG_HIDDEN);
    // } else {
    //     lv_obj_remove_flag(scr_control, LV_OBJ_FLAG_HIDDEN);
    //     lv_obj_add_flag(scr_devices, LV_OBJ_FLAG_HIDDEN);
    // }
}

void fn_cb(lv_event_t *e) {
    lv_obj_t* obj = (lv_obj_t*)lv_event_get_target(e);
    int fn = (int)lv_event_get_user_data(e);
    bool v = lv_obj_has_state(obj, LV_STATE_CHECKED);
    char msg[32];
    snprintf(msg, sizeof(msg), "%d=%d\n", fn, v?255:0);
    ble::send(msg);
}


void setup () {
    Serial.begin(115200);
    Serial.println("Starting NimBLE Client");

    analogReadResolution(10);
    pinMode(LEFT_BUTTON, INPUT_PULLUP);
    pinMode(RIGHT_BUTTON, INPUT_PULLUP);

    for(auto p: PIN_HAT) pinMode(p, INPUT);
    for(auto p: PIN_SWITCHES) pinMode(p, INPUT_PULLUP);
    pinMode(PIN_J, INPUT);

    lv_init();

    lv_tick_set_cb(my_tick);

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print );
#endif

    lv_display_t * disp;
#if LV_USE_TFT_ESPI
    /*TFT_eSPI can be enabled lv_conf.h to initialize the display in a simple way*/
    disp = lv_tft_espi_create(TFT_HOR_RES, TFT_VER_RES, draw_buf, sizeof(draw_buf));
    lv_display_set_rotation(disp, TFT_ROTATION);
#else
    #error "TFT_eSPI nto set"
#endif

    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_ENCODER );
    lv_indev_set_read_cb(indev, inputdev_cb);
    lv_group_t *g = lv_group_create();
    lv_group_set_default(g);
    lv_indev_set_group(indev, g);

    lb_battery = lv_label_create(lv_layer_top());
    lv_obj_align(lb_battery, LV_ALIGN_TOP_LEFT, 0, 0 );

    scr_devices = lv_obj_create(nullptr);
    lv_obj_set_style_pad_top(scr_devices, 20, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(scr_devices, 5, LV_PART_MAIN);
    lv_obj_add_event_cb(scr_devices, scr_load_cb, LV_EVENT_SCREEN_LOAD_START, nullptr);

    lv_obj_t * label;

    list_devs = lv_list_create(scr_devices);
    lv_obj_set_size(list_devs, lv_pct(100), lv_pct(100));

    scr_control = lv_obj_create(nullptr);
    //lv_obj_add_flag(scr_control, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_pad_top(scr_control, 20, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(scr_control, 5, LV_PART_MAIN);
    //lv_obj_add_event_cb(scr_control, scr_load_cb, LV_EVENT_SCREEN_LOAD_START, nullptr);

    lv_screen_load(scr_devices);


    pnl_inputs = lv_label_create(scr_control);
    lv_obj_align(pnl_inputs, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *bt_disconnect = lv_button_create(scr_control);
    lv_obj_add_event_cb(bt_disconnect, disconnect_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_align(bt_disconnect, LV_ALIGN_BOTTOM_LEFT, -4, -4);
    label = lv_label_create(bt_disconnect); lv_label_set_text(label, "DISC");  lv_obj_center(label);

    lv_obj_t *bt1 = lv_button_create(scr_control);
    lv_obj_add_flag(bt1, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_align_to(bt1, bt_disconnect, LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
    label = lv_label_create(bt1); lv_label_set_text(label, "HDL"); lv_obj_center(label);
    lv_obj_add_event_cb(bt1, fn_cb, LV_EVENT_VALUE_CHANGED, (int*)2);
    bt_disconnect = bt1;

    bt1 = lv_button_create(scr_control);
    lv_obj_add_flag(bt1, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_align_to(bt1, bt_disconnect, LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
    label = lv_label_create(bt1); lv_label_set_text(label, "MKR"); lv_obj_center(label);
    lv_obj_add_event_cb(bt1, fn_cb, LV_EVENT_VALUE_CHANGED, (int*)3);

    ble::init();
    ble::set_dev_found_cb(on_dev_found);
    ble::set_battery_update_cb(on_battery_updated);
    ble::set_disconnected_cb(on_dev_disconnected);
    ble::start_scan();

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

// static void on_input_panel_evt(lv_event_t * e) {
//     lv_obj_t *obj = (lv_obj_t*)lv_event_get_target(e);
//     lv_layer_t * layer = lv_event_get_layer(e);
//     lv_draw_arc(layer, arc);
//     tft.fillRect(0, 20, tft.width(), 16, TFT_BLACK);

//     constexpr int CX = 65, CY=120, R=50;

//     tft.fillCircle(CX, CY, R, TFT_DARKGREEN);
//     tft.drawString(String("")+x+"/"+y, CX-25, CY-5);
//     tft.drawWideLine(CX, CY, CX+x*R/512, CY-y*R/512, 2, down ? TFT_ORANGE : TFT_BLUE);
// }

void tick() {
    int x = analogRead(PIN_JX);
    int y = analogRead(PIN_JY);
    bool down = digitalRead(PIN_J);
    x = -to_centered(x, 465, 10);
    y = to_centered(y, 445, 10);

    static int last_x=-1000, last_y=0;
    if(x!=last_x || y!=last_y) {
        char msg[32];
        snprintf(msg, sizeof(msg), "1=%d\n0=%d\n",
            128 + x*128/512,
            128 + y*128/512
        );
        ble::send(msg);

        //String s = String(x)+"/"+y;

        //lv_obj_invalidate(input_pnl);
        lv_label_set_text(pnl_inputs, msg);
    }
    last_x = x;
    last_y = y;
}

void draw_batteries() {
    constexpr int ADC2 = 580, V2 = 4200,
        ADC1 = 428, V1 = 3200;
    int int_batt = map(analogRead(VBAT), ADC1, ADC2, V1, V2);
    char msg[100]; size_t l=0;
    if(ble::is_connected()) {
        l = snprintf(msg, sizeof(msg), "R:%d  ", remote_batt_value);
    }
    snprintf(msg+l, sizeof(msg)-l, "I:%.2fV ", int_batt/1000.0f);

    lv_label_set_text(lb_battery, msg);
}

int read_tristate(int pin) {
    pinMode(pin, INPUT_PULLUP);
    int v1 = digitalRead(pin);
    pinMode(pin, INPUT_PULLDOWN);
    int v2 = digitalRead(pin);
    // Serial.printf("v1=%d v2=%d\n", v1, v2);
    if(v1==v2) return v1 == LOW ? -1 : 1;
    return 0;
}

// void draw() {
//     char msg[64];
//     snprintf(msg, 64, "_ _ _ _ ");
//     for(size_t i=0; i<PIN_SWITCHES.size(); i++) {
//         if(digitalRead(PIN_SWITCHES[i]) == LOW) msg[i*2] = '+';
//     }

//     int t = read_tristate(PIN_BLINKER);
//     msg[3*2] = t==-1 ? '-' : t==1 ? '+' : '_';
//     tft.drawString(msg, 0, 10);

//     snprintf(msg, 64, "_ _ _ _ _ _ ");
//     for(size_t i=0; i<PIN_HAT.size(); i++) {
//         if(digitalRead(PIN_HAT[i]) == LOW) msg[i*2] = '+';
//     }
//     if(digitalRead(PIN_J)==LOW) msg[5*2] = '+';
//     tft.drawString(msg, 0, 40);
// }

void loop () {

    // for(const auto &dev: *NimBLEDevice::getClientList()) {
    //     if(!dev->isConnected()) continue;
    //     Serial.printf("conn strength %d\n", dev->getRssi());
    // }
    // draw();
    // delay(10);
    // return;

    static size_t last_t = 0;
    static bool fn_lights = false;
    if(millis() - last_t > 1000) {
        last_t = millis();
        // if(ble::is_connected()) {
        //     ble::send(v ? "2=255\n": "2=0\n");
        //     v = !v;
        // }

        draw_batteries();
    }

    if(ble::is_connected()) {
        tick();
    }

    lv_timer_handler();

    delay(5);

}
