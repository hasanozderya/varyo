#include "boot_sync.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>

namespace {
    std::atomic<bool> s_sensorsReady{false};
}

void BootSync::markSensorsReady() {
    s_sensorsReady = true;
}

bool BootSync::sensorsReady() {
    return s_sensorsReady;
}

void BootSync::waitForSensorsReady(uint32_t timeoutMs) {
    uint32_t t0 = millis();
    while (!s_sensorsReady && (millis() - t0) < timeoutMs) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
