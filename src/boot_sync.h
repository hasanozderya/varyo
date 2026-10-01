#pragma once
#include <Arduino.h>

// Vario task, barometre ve IMU baslatmasini tamamlayinca bu bayragi
// isaretler. BLE gorevi reklam ve servisleri sensor baslatmasindan sonra
// acar. Atomik bayrak icin ek mutex gerekmez.
namespace BootSync {
    void markSensorsReady();
    bool sensorsReady();

    // Sensorlerden biri yoksa BLE'nin sonsuza kadar beklememesi icin
    // timeout sonunda devam eder.
    void waitForSensorsReady(uint32_t timeoutMs);
}
