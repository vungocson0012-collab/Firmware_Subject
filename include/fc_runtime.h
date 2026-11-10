#pragma once

#include <cstdint>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static_assert(configTICK_RATE_HZ == 1000,
              "Set CONFIG_FREERTOS_HZ=1000 for the 2 ms control loop");

constexpr float FC_DEG_TO_RAD = 0.017453292519943295f;
constexpr float FC_RAD_TO_DEG = 57.29577951308232f;

inline uint32_t fc_time_us() {
  return static_cast<uint32_t>(esp_timer_get_time());
}
inline uint32_t fc_time_ms() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}
inline void fc_delay_ms(uint32_t ms) {
  if (ms != 0) vTaskDelay(pdMS_TO_TICKS(ms) > 0 ? pdMS_TO_TICKS(ms) : 1);
}
