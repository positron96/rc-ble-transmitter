#include "ble.h"

namespace ble {

    static battery_callback_t battery_cb = nullptr;

    void set_battery_update_cb(battery_callback_t cb) {
        battery_cb = cb;
    }

    static dev_found_cb_t dev_found_cb = nullptr;
    static dev_disconnected_cb_t disconnected_cb = nullptr;

    void set_dev_found_cb(dev_found_cb_t cb) {
        dev_found_cb = cb;
    }

    void set_disconnected_cb(dev_disconnected_cb_t cb) {
        disconnected_cb = cb;
    }


    static NimBLEUUID uart_service_uuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    static NimBLEUUID uart_char_tx_uuid("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
    static NimBLEUUID uart_char_rx_uuid("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

    static NimBLEUUID bat_service_uuid("180F");
    static NimBLEUUID bat_char_level_uuid("2A19");

    static NimBLERemoteCharacteristic* char_tx = nullptr;
    static NimBLEClient *client;

    constexpr uint32_t scan_duration = 0; /** 0 = scan forever */

    class ClientCallbacks : public NimBLEClientCallbacks {
        void onConnect(NimBLEClient* pClient) {
            Serial.println("Connected");
            client = pClient;
            /** After connection we should change the parameters if we don't need fast response times.
             *  These settings are 150ms interval, 0 latency, 450ms timout.
             *  Timeout should be a multiple of the interval, minimum is 100ms.
             *  I find a multiple of 3-5 * the interval works best for quick response/reconnect.
             *  Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
             */
            pClient->updateConnParams(120,120,0,60);
        };

        void onDisconnect(NimBLEClient* pClient) {
            if(disconnected_cb) {
                disconnected_cb(pClient);
            }
            char_tx = nullptr;
            client = nullptr;
        };

        /** Called when the peripheral requests a change to the connection parameters.
         *  Return true to accept and apply them or false to reject and keep
         *  the currently used parameters. Default will return true.
         */
        bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params) {
            // Serial.printf("conn update: conn:%d-%d, lat:%d; sup:%d\n",
            //     params->itvl_min, params->itvl_max,
            //     params->latency,
            //     params->supervision_timeout);
            if(params->itvl_min < BLE_GAP_CONN_ITVL_MS(10)) { /** 1.25ms units */
                return false;
            } else if(params->itvl_max > BLE_GAP_CONN_ITVL_MS(100)) { /** 1.25ms units */
                return false;
            } else if(params->latency > 5) { /** Number of intervals allowed to skip */
                return false;
            } else if(params->supervision_timeout > BLE_GAP_SUPERVISION_TIMEOUT_MS(15000)) { /** 10ms units */
                return false;
            }

            return true;
        };

        /********************* Security handled here **********************
        ****** Note: these are the same return values as defaults ********/
        uint32_t onPassKeyRequest(){
            Serial.println("Client Passkey Request");
            /** return the passkey to send to the server */
            return 123456;
        };

        bool onConfirmPIN(uint32_t pass_key){
            Serial.print("The passkey YES/NO number: ");
            Serial.println(pass_key);
        /** Return false if passkeys don't match. */
            return true;
        };

        /** Pairing process complete, we can check the results in ble_gap_conn_desc */
        void onAuthenticationComplete(ble_gap_conn_desc* desc){
            if(!desc->sec_state.encrypted) {
                Serial.println("Encrypt connection failed - disconnecting");
                /** Find the client with the connection handle provided in desc */
                NimBLEDevice::getClientByID(desc->conn_handle)->disconnect();
                return;
            }
        };
    };


    /** Define a class to handle the callbacks when advertisments are received */
    class AdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {

        void onResult(NimBLEAdvertisedDevice* dev) {

            // Serial.printf("Advertised Device found: '%s'(%s) %d services\n",
            //     dev->getName().c_str(),
            //     dev->getAddress().toString().c_str(),
            //     dev->getServiceUUIDCount());

            if(dev->getName().length()>0
                && dev->isAdvertisingService(uart_service_uuid)
                && dev_found_cb != nullptr
            ) {
                dev_found_cb(dev);
            }

            // if(dev->isAdvertisingService(uartServiceUUID))  {
            //     Serial.println("Found Our Service");
            //     /** stop scan before connecting */
            //     NimBLEDevice::getScan()->stop();
            //     /** Save the device reference in a global for the client to use*/
            //     advDevice = dev;
            //     /** Ready to connect now */
            //     doConnect = true;
            // }
        };
    };

    static ClientCallbacks client_cb;
    static AdvertisedDeviceCallbacks adv_cb;

    /** Notification / Indication receiving handler callback */
    void on_rx(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify){
        std::string str = (isNotify == true) ? "Notification" : "Indication";
        str += " from ";
        /** NimBLEAddress and NimBLEUUID have std::string operators */
        str += std::string(pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress());
        str += ": Service = " + std::string(pRemoteCharacteristic->getRemoteService()->getUUID());
        str += ", Characteristic = " + std::string(pRemoteCharacteristic->getUUID());
        str += ", Value = " + std::string((char*)pData, length);
        Serial.println(str.c_str());
    }

    void update_battery_value(uint8_t val) {
        if(battery_cb) battery_cb(val);
    }

    void on_battery_notification(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t len, bool isNotify){
        if(len<1) {
            Serial.println("Bad battery value");
            return;
        }
        update_battery_value(pData[0]);
    }


    bool connect(NimBLEAdvertisedDevice* advDevice) {

        stop_scan();

        NimBLEClient* pClient = nullptr;

        if(NimBLEDevice::getClientListSize() != 0) {
            pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
            if(pClient){
                if(!pClient->connect(advDevice, false)) {
                    Serial.println("Reconnect failed");
                    return false;
                }
                Serial.println("Reconnected client");
            } else {
                pClient = NimBLEDevice::getDisconnectedClient();
            }
        }

        /** No client to reuse? Create a new one. */
        if(!pClient) {
            if(NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS) {
                Serial.println("Max clients reached - no more connections available");
                return false;
            }

            pClient = NimBLEDevice::createClient();
            Serial.println("New client created");
            pClient->setClientCallbacks(&client_cb, false);
            /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
             *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
             *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
             */
            pClient->setConnectionParams(
                BLE_GAP_CONN_ITVL_MS(15),
                BLE_GAP_CONN_ITVL_MS(15),
                0,
                BLE_GAP_SUPERVISION_TIMEOUT_MS(510));
            /** Set how long we are willing to wait for the connection to complete (seconds), default is 30. */
            pClient->setConnectTimeout(5);


            if (!pClient->connect(advDevice)) {
                /** Created a client but failed to connect, don't need to keep it as it has no data */
                NimBLEDevice::deleteClient(pClient);
                Serial.println("Failed to connect, deleted client");
                return false;
            }
        }

        if(!pClient->isConnected()) {
            if (!pClient->connect(advDevice)) {
                Serial.println("Failed to connect");
                return false;
            }
        }

        Serial.print("Connected to: ");
        Serial.println(pClient->getPeerAddress().toString().c_str());
        Serial.print("RSSI: ");
        Serial.println(pClient->getRssi());

        NimBLERemoteService* svc = nullptr;

        svc = pClient->getService(uart_service_uuid);
        if(svc) {
            char_tx = svc->getCharacteristic(uart_char_tx_uuid);
            if(char_tx) {
                if(!char_tx->canWrite()) {
                    Serial.print("TX char is not writeable! ");
                    Serial.println(char_tx->getUUID().toString().c_str());
                    char_tx = nullptr;
                    pClient->disconnect();
                    return false;
                }
            }

            NimBLERemoteCharacteristic *char_rx = svc->getCharacteristic(uart_char_rx_uuid);
            if(char_rx) {
                if(char_rx->canNotify()) {
                    if(!char_rx->subscribe(true, on_rx)) {
                        pClient->disconnect();
                        return false;
                    }
                } else if(char_rx->canIndicate()) {
                    if(!char_rx->subscribe(false, on_rx)) {
                        pClient->disconnect();
                        return false;
                    }
                }
            }
        } else {
            Serial.println("Service not found.");
        }

        svc = pClient->getService(bat_service_uuid);
        if(svc) {
            NimBLERemoteCharacteristic* bat_char = svc->getCharacteristic(bat_char_level_uuid);

            if(bat_char) {
                if(bat_char->canRead()) {
                    NimBLEAttValue val = bat_char->readValue();
                    if(val.length()>0) update_battery_value(val.data()[0]);
                }

                if(bat_char->canNotify()) {
                    if(!bat_char->subscribe(true, on_battery_notification)) {
                        Serial.println("Subscribing to battery updates failed!");
                        return false;
                    }
                }
            }

        } else {
            Serial.println("battery service not found.");
        }

        Serial.println("Device set up");
        return true;
    }


    void init() {
        NimBLEDevice::init("");

        /** Set the IO capabilities of the device, each option will trigger a different pairing method.
         *  BLE_HS_IO_KEYBOARD_ONLY    - Passkey pairing
         *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
         *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
         */
        //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY); // use passkey
        //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

        /** 2 different ways to set security - both calls achieve the same result.
         *  no bonding, no man in the middle protection, secure connections.
         *
         *  These are the default values, only shown here for demonstration.
         */
        //NimBLEDevice::setSecurityAuth(false, false, true);
        NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);

#ifdef ESP_PLATFORM
        NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
#else
        NimBLEDevice::setPower(9); /** +9db */
#endif

        NimBLEScan* scan = NimBLEDevice::getScan();
        scan->setAdvertisedDeviceCallbacks(&adv_cb);
        scan->setInterval(45);
        scan->setWindow(15);
        scan->setActiveScan(true);
    }

    void on_scan_ended(NimBLEScanResults results){
        Serial.println("Scan Ended");
    }

    void start_scan() {
        NimBLEDevice::getScan()->start(scan_duration, on_scan_ended);
    }

    void stop_scan() {
        NimBLEDevice::getScan()->stop();
    }

    bool send(const char* msg) {
        if(char_tx == nullptr) return false;
        char_tx->writeValue(msg);
        return true;
    }

    bool is_connected() {
        return client != nullptr;
    }

    void disconnect() {
        if(client==nullptr) return;
        if(!client->isConnected()) return;
        client->disconnect();
    }

}


