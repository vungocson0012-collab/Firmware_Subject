#pragma once
#include <cstdint>
#include <cstdlib>
#include "freertos/FreeRTOS.h"
constexpr float FC_DEG_TO_RAD = 0.017453292519943295f;
constexpr float FC_RAD_TO_DEG = 57.29577951308232f;
extern uint32_t mock_ms;
inline uint32_t fc_time_ms() { return mock_ms; }
inline uint32_t fc_time_us() { return mock_ms * 1000; }
inline void fc_delay_ms(uint32_t ms) { mock_ms += ms; }
inline TickType_t xTaskGetTickCount() { return mock_ms; }
void vTaskDelayUntil(TickType_t*, TickType_t);
constexpr int ESP_OK=0;
#define ESP_ERROR_CHECK(expr) do { if ((expr) != ESP_OK) std::abort(); } while(0)

constexpr int ESP_ERR_NO_MEM=0x101;
int xTaskCreatePinnedToCore(void (*)(void*), const char*, uint32_t, void*, unsigned, void*, int);
