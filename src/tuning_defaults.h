#pragma once

#include <stdint.h>

// These defaults apply only when NVS has no saved Android-app setting.
namespace VarioTuning {
constexpr float OUTPUT_LPF_TAU_S = 0.40f;
constexpr float COMP_FILTER_ALPHA = 0.98f;
constexpr float BARO_PRESSURE_LPF_TAU_S = 0.10f;
constexpr float KF_ACCEL_VAR = 7.01f;
constexpr float KF_ACCEL_BIAS_VAR = 1e-6f;
constexpr float KF_BARO_VAR = 0.2804f;
constexpr float KF_ADAPT_FACTOR = 1.0f;
constexpr float KF_BIAS_LIMIT_MPS2 = 0.50f;
constexpr float CLIMB_DEADBAND_MPS = 0.10f;
constexpr float CLIMB_MAX_MPS = 5.0f;
constexpr float SINK_ALARM_MPS = -2.0f;
constexpr bool USE_BARO_REGRESSION_OUTPUT = false;
constexpr uint32_t BARO_REGRESSION_WINDOW_MS = 3000;
constexpr uint8_t BARO_REGRESSION_MIN_SAMPLES = 12;
constexpr float KF_BARO_VAR_FLOOR = 1e-4f;
} // namespace VarioTuning

namespace AudioTuning {
constexpr int TONE_MIN_HZ = 700;
constexpr int TONE_MAX_HZ = 2200;
constexpr int SINK_TONE_HZ = 450;
constexpr uint32_t BEEP_PERIOD_MAX_MS = 700;
constexpr uint32_t BEEP_PERIOD_MIN_MS = 120;
constexpr float BEEP_DUTY_FRACTION = 0.45f;
} // namespace AudioTuning
