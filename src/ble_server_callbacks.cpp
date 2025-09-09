#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ble_server_callbacks.hpp"
#include "ble_handler.hpp"    
#include "system_constants.hpp"
 
extern volatile bool     g_connected;
extern volatile uint16_t g_txSubs;
extern volatile bool     g_streamEnabled;

void ServerCallbacks::onConnect(NimBLEServer* srv, NimBLEConnInfo& info) {
  g_connected = true;
 
  NimBLEDevice::startAdvertising();
}

void ServerCallbacks::onDisconnect(NimBLEServer* /*srv*/, NimBLEConnInfo& /*info*/, int /*reason*/) {
  g_connected     = false;
  g_txSubs        = 0;
  g_streamEnabled = false;

   
  NimBLEDevice::startAdvertising();
}
