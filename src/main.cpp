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

#include "ble.h"

etl::vector<NimBLEAdvertisedDevice*, 5> found_devs;

#define TFT_HOR_RES   TFT_WIDTH
#define TFT_VER_RES   TFT_HEIGHT
#define TFT_ROTATION  LV_DISPLAY_ROTATION_270
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

uint8_t rem_batt_val;

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
static etl::array<lv_obj_t*, 4> bt_functions;

static void update_connected(bool c) {
    if(c) {
        lv_obj_remove_flag(im_bt, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(lb_batt_rem, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(im_batt_rem, LV_OBJ_FLAG_HIDDEN);
        lv_screen_load(scr_control);
    } else {
        ble::start_scan();
        lv_obj_add_flag(im_bt, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(lb_batt_rem, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(im_batt_rem, LV_OBJ_FLAG_HIDDEN);

        lv_obj_clean(list_devs);
        lv_list_add_text(list_devs, "Receivers found:");

        lv_screen_load(scr_devices);
    }
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
    rem_batt_val = val;
}

void on_dev_disconnected(NimBLEClient *dev) {
    Serial.print(dev->getPeerAddress().toString().c_str());
    Serial.println(" Disconnected; starting scan");

    update_connected(false);
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
    static etl::queue< etl::pair<size_t, bool>, 5> event_queue;
    static etl::bitset<PIN_HAT.size()> state;

    for(size_t i=0; i<PIN_HAT.size(); i++) {
        bool st = digitalRead(PIN_HAT[i]) == LOW;
        if(st!=state[i] && !event_queue.full()) {
            event_queue.push(etl::make_pair(i, st));
            state[i] = st;
        }
    }

    if(!event_queue.empty()) {
        const auto p = event_queue.front();
        data->key = HAT_MAP[p.first];
        data->state = p.second ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
        Serial.printf("%d (%d), %d\n", p.first, data->key, data->state);
        event_queue.pop();
    }

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

    statusbar = lv_layer_top();
    lv_obj_set_flex_flow(statusbar, LV_FLEX_FLOW_ROW);

    im_batt_intl = lv_image_create(statusbar); lv_image_set_src(im_batt_intl, LV_SYMBOL_BATTERY_EMPTY);
    lb_batt_intl = lv_label_create(statusbar);
    im_bt = lv_image_create(statusbar);  lv_image_set_src(im_bt, LV_SYMBOL_BLUETOOTH);
    im_batt_rem = lv_image_create(statusbar); lv_image_set_src(lb_batt_rem, LV_SYMBOL_BATTERY_EMPTY);
    lb_batt_rem = lv_label_create(statusbar);

    //lv_obj_align(lb_battery, LV_ALIGN_TOP_LEFT, 0, 0 );

    scr_devices = lv_screen_active();
    lv_obj_set_style_pad_top(scr_devices, 20, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(scr_devices, 5, LV_PART_MAIN);
    // lv_obj_add_event_cb(scr_devices, scr_load_cb, LV_EVENT_SCREEN_LOAD_START, nullptr);

    lv_obj_t * label;

    list_devs = lv_list_create(scr_devices);
    lv_obj_set_size(list_devs, lv_pct(100), lv_pct(100));


    scr_control = lv_obj_create(nullptr);
    //lv_obj_add_flag(scr_control, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_pad_top(scr_control, 20, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(scr_control, 5, LV_PART_MAIN);
    //lv_obj_add_event_cb(scr_control, scr_load_cb, LV_EVENT_SCREEN_LOAD_START, nullptr);

    pnl_inputs = lv_label_create(scr_control);
    // pnl_inputs = lv_obj_create(scr_control);
    // lv_obj_t *ball = lv_obj_create(pnl_inputs);
    // lv_obj_set_pos(ball, lv_pct(50), lv_pct(50));
    // lv_obj_set_size(ball, 10, 10);
    // lv_obj_set_style_radius(ball , LV_RADIUS_CIRCLE, 0);
    lv_obj_align(pnl_inputs, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_size(pnl_inputs, 50, 50);

    lv_obj_t *bt_disconnect = lv_button_create(scr_control);
    lv_obj_add_event_cb(bt_disconnect, disconnect_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_align(bt_disconnect, LV_ALIGN_BOTTOM_LEFT, -4, -4);
    //label = lv_label_create(bt_disconnect); lv_label_set_text(label, "DISC");  lv_obj_center(label);
    lv_image_set_src(lv_image_create(bt_disconnect), LV_SYMBOL_CLOSE);

    lv_obj_t *b = lv_button_create(scr_control);
    bt_functions[0] = b;
    lv_obj_add_flag(b, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_align_to(b, bt_disconnect, LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
    label = lv_label_create(b); lv_label_set_text(label, "H"); lv_obj_center(label);
    lv_obj_add_event_cb(b, fn_cb, LV_EVENT_VALUE_CHANGED, (int*)2);

    b = lv_button_create(scr_control);
    bt_functions[1] = b;
    lv_obj_add_flag(b, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_align_to(b, bt_functions[0], LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
    label = lv_label_create(b); lv_label_set_text(label, "M"); lv_obj_center(label);
    lv_obj_add_event_cb(b, fn_cb, LV_EVENT_VALUE_CHANGED, (int*)3);

    b = lv_button_create(scr_control);
    bt_functions[2] = b;
    lv_obj_add_flag(b, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_align_to(b, bt_functions[1], LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
    label = lv_label_create(b); lv_label_set_text(label, "<"); lv_obj_center(label);
    // lv_obj_add_event_cb(b, fn_cb, LV_EVENT_VALUE_CHANGED, nullptr);
    b = lv_button_create(scr_control);
    bt_functions[3] = b;
    lv_obj_add_flag(b, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_align_to(b, bt_functions[2], LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
    label = lv_label_create(b); lv_label_set_text(label, ">"); lv_obj_center(label);
    // lv_obj_add_event_cb(b, fn_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    ble::init();
    ble::set_dev_found_cb(on_dev_found);
    ble::set_battery_update_cb(on_battery_updated);
    ble::set_disconnected_cb(on_dev_disconnected);
    //ble::start_scan();

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

int read_tristate(int pin) {
    pinMode(pin, INPUT_PULLUP);
    int v1 = digitalRead(pin);
    pinMode(pin, INPUT_PULLDOWN);
    int v2 = digitalRead(pin);
    // Serial.printf("v1=%d v2=%d\n", v1, v2);
    if(v1==v2) return v1 == LOW ? -1 : 1;
    return 0;
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
        snprintf(msg, sizeof(msg), "1=%d\n0=%d\n",
            128 + x*128/512,
            128 + y*128/512
        );
        ble::send(msg);

        lv_label_set_text(pnl_inputs, msg);
        //lv_obj_t *ball = lv_obj_get_child(pnl_inputs, 0);
        //lv_obj_set_pos(ball, lv_pct(50 + x*50/512), lv_pct(50 + x*50/512));
    }
    last_x = x;
    last_y = y;

    bool st = digitalRead(PIN_SWITCHES[0]) == LOW;
    lv_obj_set_state(bt_functions[0], LV_STATE_CHECKED, st);

    int t = read_tristate(PIN_BLINKER);
    if(t>0) {
        lv_obj_set_state(bt_functions[2], LV_STATE_CHECKED, true);
    } else if(t<0) {
        lv_obj_set_state(bt_functions[3], LV_STATE_CHECKED, true);
    } else {
        lv_obj_set_state(bt_functions[2], LV_STATE_CHECKED, false);
        lv_obj_set_state(bt_functions[3], LV_STATE_CHECKED, false);
    }
}

void draw_batteries() {
    constexpr int ADC2 = 580, V2 = 4200,
        ADC1 = 428, V1 = 3200;
    int int_batt = map(analogRead(VBAT), ADC1, ADC2, V1, V2);

    int idx = map(int_batt, 3300, 4200, 0, 4);
    idx = constrain(idx, 0, 4);

    static int last_int_batt;
    if(last_int_batt!=int_batt/100) {
        lv_image_set_src(im_batt_intl, LV_SYMBOL_BATTERY_EMPTY - idx);
        lv_label_set_text_fmt(lb_batt_intl, "%dmV ", int_batt);
        last_int_batt = int_batt/100;
    }

    if(ble::is_connected()) {
        static int last_rem_batt;
        if(last_rem_batt != rem_batt_val) {
            lv_image_set_src(im_batt_intl, LV_SYMBOL_BATTERY_EMPTY);
            lv_label_set_text_fmt(lb_batt_rem, "%d", rem_batt_val);
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
