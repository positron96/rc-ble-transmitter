#include <type_traits>
#include <Arduino.h>

#include <TFT_eSPI.h>       // Hardware-specific library

#include <NimBLEDevice.h>

#include <etl/debounce.h>
#include <etl/vector.h>

#include "ble.h"

etl::vector<NimBLEAdvertisedDevice*, 5> found_devs;

TFT_eSPI tft;

uint8_t remote_batt_value;

void on_dev_found(NimBLEAdvertisedDevice *dev) {
    tft.printf("%s(%s)\n", dev->getName().c_str(), dev->getAddress().toString().c_str());
    found_devs.push_back(dev);
}

void on_battery_updated(uint8_t val) {
    remote_batt_value = val;
    //Serial.printf("Battery: %d\n", val);
    String ss = String("Bat:")+val+"  ";
    tft.drawString(ss, 1, tft.getViewportHeight() - tft.fontHeight());
}

void on_dev_disconnected(NimBLEClient *dev) {
    Serial.print(dev->getPeerAddress().toString().c_str());
    Serial.println(" Disconnected; starting scan");
    ble::start_scan();
}

void setup () {
    Serial.begin(115200);
    Serial.println("Starting NimBLE Client");

    analogReadResolution(10);
    pinMode(LEFT_BUTTON, INPUT);
    pinMode(RIGHT_BUTTON, INPUT);

    tft.init();

    tft.fillScreen(TFT_BLACK);
    tft.setFreeFont(&FreeMono9pt7b);
    tft.setCursor(0, 0, 4);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.println("BLE");

    ble::init();
    ble::set_dev_found_cb(on_dev_found);
    ble::set_battery_update_cb(on_battery_updated);
    ble::set_disconnected_cb(on_dev_disconnected);
    ble::start_scan();
}


constexpr int PIN_X = 2;
constexpr int PIN_Y = 15;
constexpr int PIN_C = 17;


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
    int x = analogRead(PIN_X), t=x;
    int y = analogRead(PIN_Y);
    bool down = digitalRead(PIN_C);
    x = to_centered(x, 465, 10);
    y = to_centered(y, 445, 10);

    static int last_x, last_y;
    if(x != last_x || y!=last_y) {
        tft.fillRect(0, 20, tft.width(), 16, TFT_BLACK);
        tft.setTextColor(TFT_CYAN);
        tft.drawString(String("")+t+"/"+x, 1, 20);

        constexpr int CX = 65, CY=100, R=50;

        tft.fillCircle(CX, CY, R, TFT_GREEN);
        tft.drawLine(CX, CY, CX+x*R/512, CY+y*R/512, down ? TFT_ORANGE : TFT_BLUE);
    }

    String s = String("0="); s+=x*128/512; s+="\n";
    ble::send(s.c_str());
    s = String("1="); s+= y*128/512; s+="\n";
    ble::send(s.c_str());
}

void draw_batteries() {

}

void loop () {

    // for(const auto &dev: *NimBLEDevice::getClientList()) {
    //     if(!dev->isConnected()) continue;
    //     Serial.printf("conn strength %d\n", dev->getRssi());
    // }

    static size_t last_t = 0;
    static bool v = false;
    if(millis() - last_t > 1000) {
        last_t = millis();
        if(ble::is_connected()) {
            ble::send(v ? "2=255\n": "2=0\n");
            v = !v;
        }

        Serial.printf("int bat: %d\n", analogRead(VBAT));
    }

    static etl::debounce<1, 20> bt_connect;
    static etl::debounce<1> bt_disconenct;

    if(bt_connect.add(digitalRead(LEFT_BUTTON) == LOW)) {
        if(bt_connect.is_set() && found_devs.size()>0) {
            ble::connect(found_devs[0]);
            found_devs.clear();
        }
    }

    if(bt_connect.has_changed() && bt_connect.is_held() && ble::is_connected()) {
        ble::disconnect();
    }

    if(ble::is_connected()) {
        tick();
    }

    delay(50);

}
