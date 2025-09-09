#pragma once
#include <NimBLEDevice.h>

class ServerCallbacks : public NimBLEServerCallbacks {
public:
  void onConnect(NimBLEServer* srv, NimBLEConnInfo& info) override;
  void onDisconnect(NimBLEServer* srv, NimBLEConnInfo& info, int reason) override;
};
