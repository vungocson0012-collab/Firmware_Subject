#pragma once
#include "FreeRTOS.h"
using SemaphoreHandle_t=void*;
int xSemaphoreTake(SemaphoreHandle_t, TickType_t);
int xSemaphoreGive(SemaphoreHandle_t);

SemaphoreHandle_t xSemaphoreCreateMutex();
