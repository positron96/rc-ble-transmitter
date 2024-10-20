#include <type_traits>
#include <Arduino.h>

#include <TFT_eSPI.h>       // Hardware-specific library

#include <NimBLEDevice.h>

#include <etl/debounce.h>
#include <etl/vector.h>
#include <etl/array.h>

#include "ble.h"

etl::vector<NimBLEAdvertisedDevice*, 5> found_devs;

TFT_eSPI tft;

uint8_t remote_batt_value;

void on_dev_found(NimBLEAdvertisedDevice *dev) {
    //tft.printf("\n", dev->getName().c_str(), dev->getAddress().toString().c_str());
    if(found_devs.available()>0) found_devs.push_back(dev);
    tft.fillScreen(TFT_BLACK);
    for(size_t i=0; i<found_devs.size(); i++) {
        const auto &d = found_devs[i];
        char msg[50];
        size_t l = snprintf(
            msg,
            sizeof(msg),
            "%s(%s)",
            dev->getName().c_str(),
            dev->getAddress().toString().c_str());
        tft.drawString(msg, 0, i*tft.fontHeight());
    }
}

void on_battery_updated(uint8_t val) {
    remote_batt_value = val;
}

void on_dev_disconnected(NimBLEClient *dev) {
    Serial.print(dev->getPeerAddress().toString().c_str());
    Serial.println(" Disconnected; starting scan");

    found_devs.clear();
    ble::start_scan();
}

constexpr auto PIN_ANALOG = etl::make_array<int>(33, 32, 39, 38);
constexpr auto PIN_JX = PIN_ANALOG[0];
constexpr auto PIN_JY = PIN_ANALOG[1];
constexpr int PIN_J = 25;
constexpr auto PIN_HAT = etl::make_array<int>(37, 36, 15, 13, 12);
constexpr auto PIN_SWITCHES = etl::make_array<int>(2, 17, 22, 21);
constexpr int PIN_BLINKER = PIN_SWITCHES[3];

void setup () {
    Serial.begin(115200);
    Serial.println("Starting NimBLE Client");

    analogReadResolution(10);
    pinMode(LEFT_BUTTON, INPUT_PULLUP);
    pinMode(RIGHT_BUTTON, INPUT_PULLUP);

    for(auto p: PIN_HAT) pinMode(p, INPUT);
    for(auto p: PIN_SWITCHES) pinMode(p, INPUT_PULLUP);
    pinMode(PIN_J, INPUT);

    tft.init();
    tft.setRotation(3);

    tft.fillScreen(TFT_BLACK);

    //tft.setFreeFont(&FreeSans9pt7b);
    tft.setFreeFont(&FreeSans12pt7b);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.println();
    tft.println("BLE");

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

void tick() {
    int x = analogRead(PIN_JX);
    int y = analogRead(PIN_JY);
    bool down = digitalRead(PIN_J);
    x = -to_centered(x, 465, 10);
    y = to_centered(y, 445, 10);

    static int last_x=-1000, last_y=0;
    if(x!=last_x || y!=last_y) {
        tft.fillRect(0, 20, tft.width(), 16, TFT_BLACK);

        constexpr int CX = 65, CY=120, R=50;

        tft.fillCircle(CX, CY, R, TFT_DARKGREEN);
        tft.drawString(String("")+x+"/"+y, CX-25, CY-5);
        tft.drawWideLine(CX, CY, CX+x*R/512, CY-y*R/512, 2, down ? TFT_ORANGE : TFT_BLUE);

        String s = String("1="); s+=128 + x*128/512; s+="\n";
        ble::send(s.c_str());
        s = String("0="); s+= 128 + y*128/512; s+="\n";
        ble::send(s.c_str());
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
    int16_t h = tft.fontHeight();
    tft.fillRect(0, tft.getViewportHeight() - h, tft.getViewportWidth(), h, TFT_BLACK);
    tft.drawString(msg, 1, tft.getViewportHeight() - h);
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

void draw() {
    char msg[64];
    snprintf(msg, 64, "_ _ _ _ ");
    for(size_t i=0; i<PIN_SWITCHES.size(); i++) {
        if(digitalRead(PIN_SWITCHES[i]) == LOW) msg[i*2] = '+';
    }

    int t = read_tristate(PIN_BLINKER);
    msg[3*2] = t==-1 ? '-' : t==1 ? '+' : '_';
    tft.drawString(msg, 0, 10);

    snprintf(msg, 64, "_ _ _ _ _ _ ");
    for(size_t i=0; i<PIN_HAT.size(); i++) {
        if(digitalRead(PIN_HAT[i]) == LOW) msg[i*2] = '+';
    }
    if(digitalRead(PIN_J)==LOW) msg[5*2] = '+';
    tft.drawString(msg, 0, 40);
}

void loop () {

    // for(const auto &dev: *NimBLEDevice::getClientList()) {
    //     if(!dev->isConnected()) continue;
    //     Serial.printf("conn strength %d\n", dev->getRssi());
    // }
    draw();
    delay(10);
    return;

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

    static etl::debounce<1, 20> bt_connect;
    static etl::debounce<1, 20> bt_func;

    // tft.fillRect(tft.width()-10, 0, 10, 10, rd?TFT_GREEN:TFT_RED);
    if(bt_connect.add(digitalRead(LEFT_BUTTON) == LOW)) {
        if(bt_connect.is_set() && found_devs.size()>0) {
            ble::connect(found_devs[0]);
        }
    }

    if(bt_connect.has_changed() && bt_connect.is_held() && ble::is_connected()) {
        ble::disconnect();
    }

    if(bt_func.add(digitalRead(RIGHT_BUTTON) == LOW)) {
        if(bt_func.is_set() && ble::is_connected()) {
            ble::send(fn_lights?"2=255\n" : "2=0\n");
            fn_lights = !fn_lights;
        }
    }

    if(ble::is_connected()) {
        tick();
    }

    delay(50);

}
