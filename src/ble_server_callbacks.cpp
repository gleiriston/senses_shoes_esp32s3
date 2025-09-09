#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ble_server_callbacks.hpp"
#include "ble_handler.hpp"     // para setAdvertising e protótipos
#include "system_constants.hpp"

// Variáveis globais definidas em ble_handler.cpp
extern volatile bool     g_connected;
extern volatile uint16_t g_txSubs;
extern volatile bool     g_streamEnabled;

void ServerCallbacks::onConnect(NimBLEServer* srv, NimBLEConnInfo& info) {
  g_connected = true;

  // (Opcional) Se sua build suportar, pode tentar ajustar params de conexão:
  // srv->updateConnParams(info, 6, 12, 0, 400); // 7.5–15ms, latency=0, timeout=4s

  // Continua anunciando para permitir outro central (comportamento opcional)
  NimBLEDevice::startAdvertising();
}

void ServerCallbacks::onDisconnect(NimBLEServer* /*srv*/, NimBLEConnInfo& /*info*/, int /*reason*/) {
  g_connected     = false;
  g_txSubs        = 0;
  g_streamEnabled = false;

  // Volta a anunciar para facilitar reconexão
  NimBLEDevice::startAdvertising();
}
