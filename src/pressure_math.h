#pragma once
#include <stdint.h>

// MS5607 compensated pressure, Pa. Keep the fractional Pascal until the
// final float store; truncating the final division creates ~8 cm altitude
// steps near sea level even when the raw ADC changes smoothly.
inline float ms5607CompensatedPressurePa(uint32_t d1, int64_t sens, int64_t off) {
    return (float)(((double)d1 * (double)sens / 2097152.0 - (double)off) / 32768.0);
}
