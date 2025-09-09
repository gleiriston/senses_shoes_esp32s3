#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

// Globais BLE (definidas em ble_handler.cpp)
extern NimBLEServer*         g_server;
extern NimBLEService*        g_service;
extern NimBLECharacteristic* g_rxChar;
extern NimBLECharacteristic* g_txChar;

extern volatile bool         g_connected;
extern volatile uint16_t     g_txSubs;
extern volatile bool         g_streamEnabled;

// API
void bleInit();
void bleStartAdvertising();
void nusSend(const char* buf, size_t n);
static inline void nusSendLit(const char* s){ nusSend(s, strlen(s)); }
void bleHandleCommand(const String& s);
