#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "status.hpp"
#include "system_constants.hpp"
#include "ble_handler.hpp"   // g_connected, g_txSubs, g_streamEnabled

// Task do LED de status
static void StatusTask(void*) {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  for (;;) {
    const bool tx_any = (g_connected && g_txSubs > 0 && g_streamEnabled);

    if (tx_any) {
      // Transmitindo: LED sólido
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(50)); // yield curto
    } else {
      // Não transmitindo: pisca
      const uint32_t interval = g_connected ? LED_BLINK_MS_CONNECTED
                                            : LED_BLINK_MS_DISCONNECTED;
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      vTaskDelay(pdMS_TO_TICKS(interval));
    }
  }
}

void statusInit() {
  xTaskCreatePinnedToCore(
    StatusTask, "status_led", 2048, nullptr, 1,
    nullptr, ARDUINO_RUNNING_CORE
  );
}
