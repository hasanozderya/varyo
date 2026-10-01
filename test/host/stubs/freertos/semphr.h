#pragma once
#include "FreeRTOS.h"
#include <mutex>
using SemaphoreHandle_t = std::mutex*;
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return new std::mutex; }
inline int xSemaphoreTake(SemaphoreHandle_t m, TickType_t) { return m->try_lock() ? pdTRUE : 0; }
inline void xSemaphoreGive(SemaphoreHandle_t m) { m->unlock(); }
