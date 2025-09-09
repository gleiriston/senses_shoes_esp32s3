#pragma once
#include <NimBLEDevice.h>

class TxCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onSubscribe(NimBLECharacteristic* c,
                   NimBLEConnInfo& info,
                   uint16_t subValue) override;
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic* c,
               NimBLEConnInfo& info) override;
};
