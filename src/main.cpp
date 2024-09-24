#include <Arduino.h>

#include <TFT_eSPI.h>       // Hardware-specific library
TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

#include <NimBLEDevice.h>


static NimBLEUUID uartServiceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID uartCharTxUUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID uartCharRxUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

static NimBLEUUID batServiceUUID("180F");
static NimBLEUUID batCharLevelUUID("2A19");


void scanEndedCB(NimBLEScanResults results);

static NimBLEAdvertisedDevice* advDevice;

static bool doConnect = false;
constexpr uint32_t scanTime = 0; /** 0 = scan forever */

NimBLERemoteCharacteristic* txChar = nullptr;


/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        Serial.println("Connected");
        /** After connection we should change the parameters if we don't need fast response times.
         *  These settings are 150ms interval, 0 latency, 450ms timout.
         *  Timeout should be a multiple of the interval, minimum is 100ms.
         *  I find a multiple of 3-5 * the interval works best for quick response/reconnect.
         *  Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
         */
        pClient->updateConnParams(120,120,0,60);
    };

    void onDisconnect(NimBLEClient* pClient) {
        Serial.print(pClient->getPeerAddress().toString().c_str());
        Serial.println(" Disconnected - Starting scan");
        txChar = nullptr;
        NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
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
        Serial.printf("Advertised Device found: '%s'(%s) %d services\n",
            dev->getName().c_str(),
            dev->getAddress().toString().c_str(),
            dev->getServiceUUIDCount());

        if(dev->isAdvertisingService(uartServiceUUID))  {
            Serial.println("Found Our Service");
            /** stop scan before connecting */
            NimBLEDevice::getScan()->stop();
            /** Save the device reference in a global for the client to use*/
            advDevice = dev;
            /** Ready to connect now */
            doConnect = true;
        }
    };
};


/** Notification / Indication receiving handler callback */
void onRxNotification(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify){
    std::string str = (isNotify == true) ? "Notification" : "Indication";
    str += " from ";
    /** NimBLEAddress and NimBLEUUID have std::string operators */
    str += std::string(pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress());
    str += ": Service = " + std::string(pRemoteCharacteristic->getRemoteService()->getUUID());
    str += ", Characteristic = " + std::string(pRemoteCharacteristic->getUUID());
    str += ", Value = " + std::string((char*)pData, length);
    Serial.println(str.c_str());
}

void onBatteryNotification(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t len, bool isNotify){
    if(len<1) {
        Serial.println("Bad battery value");
        return;
    }

    Serial.printf("Battery notification (%s): %d\n",
        pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress().toString().c_str(),
        pData[0]);
}

/** Callback to process the results of the last scan or restart it */
void scanEndedCB(NimBLEScanResults results){
    Serial.println("Scan Ended");
}


/** Create a single global instance of the callback class to be used by all clients */
static ClientCallbacks clientCB;


/** Handles the provisioning of clients and connects / interfaces with the server */
bool connectToClient() {
    NimBLEClient* pClient = nullptr;

    /** Check if we have a client we should reuse first **/
    if(NimBLEDevice::getClientListSize() != 0) {
        /** Special case when we already know this device, we send false as the
         *  second argument in connect() to prevent refreshing the service database.
         *  This saves considerable time and power.
         */
        pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
        if(pClient){
            if(!pClient->connect(advDevice, false)) {
                Serial.println("Reconnect failed");
                return false;
            }
            Serial.println("Reconnected client");
        }
        /** We don't already have a client that knows this device,
         *  we will check for a client that is disconnected that we can use.
         */
        else {
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

        pClient->setClientCallbacks(&clientCB, false);
        /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
         *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
         *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
         *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
         */
        pClient->setConnectionParams(12,12,0,51);
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

    NimBLERemoteService* pSvc = nullptr;

    pSvc = pClient->getService(uartServiceUUID);
    if(pSvc) {
        txChar = pSvc->getCharacteristic(uartCharTxUUID);
        if(txChar) {
            if(!txChar->canWrite()) {
                Serial.print("TX char is not writeable! ");
                txChar = nullptr;
                Serial.println(txChar->getUUID().toString().c_str());
                pClient->disconnect();
                return false;
            }
        }

        NimBLERemoteCharacteristic *rxChar = pSvc->getCharacteristic(uartCharRxUUID);
        if(rxChar) {
            if(rxChar->canNotify()) {
                if(!rxChar->subscribe(true, onRxNotification)) {
                    pClient->disconnect();
                    return false;
                }
            } else if(rxChar->canIndicate()) {
                /** Send false as first argument to subscribe to indications instead of notifications */
                if(!rxChar->subscribe(false, onRxNotification)) {
                    /** Disconnect if subscribe failed */
                    pClient->disconnect();
                    return false;
                }
            }
        }

    } else {
        Serial.println("Service not found.");
    }

    pSvc = pClient->getService(batServiceUUID);
    if(pSvc) {
        NimBLERemoteCharacteristic* pChr = pSvc->getCharacteristic(batCharLevelUUID);

        if(pChr) {
            if(pChr->canRead()) {
                Serial.printf("Battery value: %d\n", pChr->readValue().data()[0]);
            }

            if(pChr->canNotify()) {
                if(!pChr->subscribe(true, onBatteryNotification)) {
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

void setup (){
    Serial.begin(115200);
    Serial.println("Starting NimBLE Client");
    /** Initialize NimBLE, no device name spcified as we are not advertising */
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

    /** Optional: set the transmit power, default is 3db */
#ifdef ESP_PLATFORM
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
#else
    NimBLEDevice::setPower(9); /** +9db */
#endif

    /** Optional: set any devices you don't want to get advertisments from */
    // NimBLEDevice::addIgnored(NimBLEAddress ("aa:bb:cc:dd:ee:ff"));

    /** create new scan */
    NimBLEScan* pScan = NimBLEDevice::getScan();

    /** create a callback that gets called when advertisers are found */
    pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());

    /** Set scan interval (how often) and window (how long) in milliseconds */
    pScan->setInterval(45);
    pScan->setWindow(15);

    /** Active scan will gather scan response data from advertisers
     *  but will use more energy from both devices
     */
    pScan->setActiveScan(true);
    /** Start scanning for advertisers for the scan time specified (in seconds) 0 = forever
     *  Optional callback for when scanning stops.
     */
    pScan->start(scanTime, scanEndedCB);
}


void loop () {

    if(doConnect) {
        doConnect = false;
        if(!connectToClient() ) {
            Serial.println("Failed to connect, starting scan");
            NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
        }
    }

    // for(const auto &dev: *NimBLEDevice::getClientList()) {
    //     if(!dev->isConnected()) continue;
    //     Serial.printf("conn strength %d\n", dev->getRssi());
    // }

    static size_t last_t = 0;
    static bool v = false;
    if(millis() - last_t > 1000) {
        last_t = millis();
        if(txChar != nullptr) {
            txChar->writeValue(v ? "2=255\n": "2=0\n");
            v = !v;
        }
    }

    delay(1000);

}
