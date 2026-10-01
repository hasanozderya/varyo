#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

namespace Timing {
constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint16_t I2C_TIMEOUT_MS = 4;
constexpr uint32_t GPS_BAUD = 9600;
constexpr uint32_t VARIO_TASK_HZ = 100;
constexpr uint32_t UI_TASK_HZ = 500;
constexpr uint32_t DISPLAY_REFRESH_HZ = 5;
constexpr TickType_t I2C_MUTEX_TIMEOUT = pdMS_TO_TICKS(50);
} // namespace Timing

namespace VarioDebug {
constexpr bool SERIAL_CSV = false;
constexpr uint32_t SERIAL_CSV_HZ = 12;
constexpr bool MUTE_BUZZER_DURING_DIAG = false;
} // namespace VarioDebug
