#include <Arduino.h>
#include "config.h"
#include "i2c_bus.h"
#include "shared_state.h"
#include "tunables.h"
#include "debug_log.h"
#include "vario_task.h"
#include "gps_ui_task.h"
#include "web_task.h"
#include "ble_task.h"
#include <time.h>
#include "mpu6050.h"
#include "maintenance_task.h"
#include "audio_output.h"

// Task priorities: all kept well below the range ESP-IDF system tasks
// (including the WiFi driver's own internal task) use internally, but
// the vario task has a clear edge over UI/web so GPS/OLED/HTTP work can
// never starve the real-time sensor-fusion+audio loop.
namespace {
    constexpr UBaseType_t VARIO_TASK_PRIORITY = 5;
    constexpr UBaseType_t UI_TASK_PRIORITY    = 2;
    constexpr UBaseType_t WEB_TASK_PRIORITY   = 1;
    // Stack sizes are in BYTES (ESP-IDF's xTaskCreate convention, which
    // Arduino-ESP32's xTaskCreatePinnedToCore inherits — NOT words as in
    // vanilla FreeRTOS).
    constexpr uint32_t VARIO_TASK_STACK = 4096;
    constexpr uint32_t UI_TASK_STACK    = 8192; // TinyGPS++ + U8g2 + snprintf headroom
    constexpr uint32_t WEB_TASK_STACK   = 8192; // WiFi + WebServer + HTML page
}

void setup() {
    Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
    // Native USB-CDC: host baglantisi beklenmedik sekilde kesilirse
    // (host FIFO'yu bosaltmayi birakirsa) Serial.print()/write()'in
    // suresiz bloklamasini engelle. DebugLog artik Serial'e sadece
    // web_task'tan (dusuk oncelikli, non-blocking kontrolluyle)
    // yaziyor olsa da, bu Core 1'i (UI/GPS ile paylasilan) her ihtimale
    // karsi tam korur. Arduino-ESP32 core v2.0.14+ / v3.x'te mevcut.
    Serial.setTxTimeoutMs(0);
#endif
    // Start sensors immediately; opening in flight must not wait for USB.

    setenv("TZ", "UTC0", 1);
    tzset();
    I2CBus::begin();
    SharedState::init();
    TunablesStore::init();
    DebugLog::init();
    MPU6050::initStorage();
    if (!AudioOutput::begin()) DebugLog::log("Audio: task allocation failed; no speaker output\n");

    const BaseType_t varioCreated = xTaskCreatePinnedToCore(varioTaskFunc, "VarioTask", VARIO_TASK_STACK,
                             nullptr, VARIO_TASK_PRIORITY, nullptr, 0 /* Core 0 */);
    if (varioCreated != pdPASS) abort();
    const BaseType_t uiCreated = xTaskCreatePinnedToCore(uiTaskFunc, "UiTask", UI_TASK_STACK,
                             nullptr, UI_TASK_PRIORITY, nullptr, 1 /* Core 1 */);
    if (uiCreated != pdPASS) abort();
    const BaseType_t maintenanceCreated = xTaskCreatePinnedToCore(maintenanceTaskFunc,
        "Maintenance", 8192, nullptr, 1, nullptr, 1);
    if (maintenanceCreated != pdPASS) abort();
#if VARIO_USE_WIFI
    const BaseType_t webCreated = xTaskCreatePinnedToCore(webTaskFunc, "WebTask", WEB_TASK_STACK,
                            nullptr, WEB_TASK_PRIORITY, nullptr, 1 /* Core 1 */);
    if (webCreated != pdPASS) abort();
#else
    const BaseType_t bleCreated = xTaskCreatePinnedToCore(bleTaskFunc, "BleTask", WEB_TASK_STACK,
                            nullptr, WEB_TASK_PRIORITY, nullptr, 1 /* Core 1 */);
    if (bleCreated != pdPASS) abort();
#endif
}

void loop() {
    // All real work happens in the pinned tasks above — put
    // Arduino's own loop task to sleep indefinitely instead of
    // busy-polling an empty loop.
    vTaskDelay(portMAX_DELAY);
}
