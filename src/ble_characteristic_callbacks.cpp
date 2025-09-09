#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ble_characteristic_callbacks.hpp"
#include "ble_handler.hpp"

 
extern volatile uint16_t g_txSubs;

 
extern void bleHandleCommand(const String& s);

 
void TxCallbacks::onSubscribe(NimBLECharacteristic* /*c*/,
                              NimBLEConnInfo& /*info*/,
                              uint16_t subValue) {
  if (subValue & 0x0001) {
    // Client Characteristic Configuration (CCC) => notifications ON
    g_txSubs++;
  } else {
    if (g_txSubs > 0) g_txSubs--;
  }
}

// ===== RX (write) =====
void RxCallbacks::onWrite(NimBLECharacteristic* c, NimBLEConnInfo& /*info*/) {
  std::string v = c->getValue();
  if (v.empty()) return;

  String s = v.c_str();
  s.trim();
  s.replace("\r", "");
  s.replace("\n", "");


  bleHandleCommand(s);
}
