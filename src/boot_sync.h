#pragma once
#include <Arduino.h>

// Cok kucuk bir senkronizasyon bayragi: vario task, baro+IMU begin()/
// baslatma tamamlaninca bunu isaretler; web task, WiFi AP'yi
// baslatmadan once bunu bekler. Amac: WiFi.softAP()'nin cektigi ani
// akim darbesinin, MS5607/MPU6050'nin begin() sirasindaki hassas
// I2C/guc penceresiyle cakismasini onlemek (bkz. web_task.cpp'deki
// guc dekuplaj notu). bool'un tek kelime read/write'i ESP32'de zaten
// atomik oldugu icin ekstra mutex'e gerek yok.
namespace BootSync {
    void markSensorsReady();
    bool sensorsReady();

    // Web task bunu cagirir: sensorsReady() true olana kadar bekler,
    // ama sensor init tamamen basarisiz olursa sonsuza kadar WiFi'siz
    // kalinmasin diye timeoutMs sonra da pes edip devam eder.
    void waitForSensorsReady(uint32_t timeoutMs);
}
