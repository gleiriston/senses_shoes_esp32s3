#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ble_handler.hpp"
#include "device.hpp" 
#include "system_constants.hpp"
 
NimBLEServer*         g_server   = nullptr;
NimBLEService*        g_service  = nullptr;
NimBLECharacteristic* g_rxChar   = nullptr;
NimBLECharacteristic* g_txChar   = nullptr;

volatile bool         g_connected = false;
volatile uint16_t     g_txSubs    = 0;
volatile bool         g_streamEnabled = true;
 
void nusSend(const char* buf, size_t n){
  if(!g_txChar || g_txSubs == 0) return;
  g_txChar->setValue((uint8_t*)buf, n);
  g_txChar->notify();
}

 
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* srv, NimBLEConnInfo& info) override {
    g_connected = true;
    
    NimBLEDevice::startAdvertising();
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    g_connected = false; g_txSubs = 0; g_streamEnabled = false;
    NimBLEDevice::startAdvertising();
  }
};

extern void bleHandleCommand(const String&); 

class TxCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo&, uint16_t subValue) override {
    if(subValue & 0x0001) g_txSubs++;
    else if(g_txSubs > 0) g_txSubs--;
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    std::string v = c->getValue(); if(v.empty()) return;
    String s = v.c_str(); s.trim(); s.replace("\r",""); s.replace("\n","");
    bleHandleCommand(s);
  }
};

 
void bleStartAdvertising(){
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();

  NimBLEAdvertisementData advData;
  advData.setName(DEVICE_NAME);
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);

  NimBLEAdvertisementData srData;
  srData.addServiceUUID(NUS_SERVICE_UUID);

  adv->setAdvertisementData(advData);
  adv->setScanResponseData(srData);
  adv->setMinInterval(160);   
  adv->setMaxInterval(320);    
  adv->start();
}

 
void bleInit(){
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(185);
  NimBLEDevice::setDeviceName(DEVICE_NAME);

  g_server = NimBLEDevice::createServer();
  g_server->setCallbacks(new ServerCallbacks());

  g_service = g_server->createService(NUS_SERVICE_UUID);

  g_rxChar = g_service->createCharacteristic(
      NUS_RX_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  g_rxChar->setCallbacks(new RxCallbacks());

  g_txChar = g_service->createCharacteristic(
      NUS_TX_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  g_txChar->setCallbacks(new TxCallbacks());
  g_txChar->createDescriptor("2902");

  g_service->start();
  bleStartAdvertising();
}
