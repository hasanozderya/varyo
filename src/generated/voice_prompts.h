#pragma once
#include <stddef.h>
#include <stdint.h>

namespace VoicePrompts {
    extern const int16_t FLIGHT_STARTED[];
    inline constexpr size_t FLIGHT_STARTED_COUNT = 16219;
    extern const int16_t FLIGHT_ENDED[];
    inline constexpr size_t FLIGHT_ENDED_COUNT = 19183;
    extern const int16_t GPS_ACQUIRED[];
    inline constexpr size_t GPS_ACQUIRED_COUNT = 29419;
    extern const int16_t GPS_LOST[];
    inline constexpr size_t GPS_LOST_COUNT = 31405;
    extern const int16_t ALTITUDE_LIMIT_APPROACHING[];
    inline constexpr size_t ALTITUDE_LIMIT_APPROACHING_COUNT = 48768;
}
