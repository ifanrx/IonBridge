#ifndef BLE_CALLBACKS_H_
#define BLE_CALLBACKS_H_

#include "NimBLECharacteristic.h"
#include "NimBLELocalValueAttribute.h"
#include "NimBLEServer.h"

class ServerCallbacks : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo);
  void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo,
                    int reason);
  void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo);
};

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
 private:
  void logValue(NimBLECharacteristic *pCharacteristic, const char *name);

 public:
  void onWrite(NimBLECharacteristic *pCharacteristic,
               NimBLEConnInfo &connInfo) override;
  void onRead(NimBLECharacteristic *pCharacteristic,
              NimBLEConnInfo &connInfo) override;
  void onSubscribe(NimBLECharacteristic *pCharacteristic,
                   NimBLEConnInfo &connInfo, uint16_t subValue) override;
  void onStatus(NimBLECharacteristic *pCharacteristic, int code) override;
};

#endif
