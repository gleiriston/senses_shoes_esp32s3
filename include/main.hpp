#pragma once

#include <SPIFFS.h>
#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>
#include <EEPROM.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "device.hpp"

#include "imu.hpp"

#include "max17048.hpp"
#include "system_constants.hpp"
#include "ble_characteristic_callbacks.hpp"
#include "ble_handler.hpp"
#include "ble_server_callbacks.hpp"
#include "kalman_filter.hpp"
