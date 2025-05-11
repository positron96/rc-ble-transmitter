#ifndef BLE_H_
#define BLE_H_

#include <type_traits>

#include <Arduino.h>

#include <NimBLEDevice.h>


namespace ble {

    using dev_found_cb_t = std::add_pointer_t<void(NimBLEAdvertisedDevice*)>;
    using dev_disconnected_cb_t = std::add_pointer_t<void(NimBLEClient*)>;
    using battery_callback_t = std::add_pointer_t<void(uint8_t)>;
    using rx_callback_t = std::add_pointer_t<void(const std::string&)>;

    void init();

    void start_scan();
    void stop_scan();

    bool connect(NimBLEAdvertisedDevice*);
    void disconnect();
    bool is_connected();

    const std::string & get_connected_dev_name();

    bool send(const char* msg);

    void set_battery_update_cb(battery_callback_t);
    void set_dev_found_cb(dev_found_cb_t);
    void set_disconnected_cb(dev_disconnected_cb_t);
    void set_rx_cb(rx_callback_t);

};

#endif // BLE_H_