#pragma once
#include <Arduino.h>
#include <stddef.h>

struct TrendPoint {
    uint32_t tMs;
    float altitudeM;
    float varioMps;
    float pressurePa;
};

namespace TrendBuffer {
    // 30 minutes at 1 Hz. Allocation prefers PSRAM and falls back to
    // regular heap automatically; nothing is written to NVS/flash.
    constexpr size_t CAPACITY = 1800;

    bool init();
    void clear();
    void add(uint32_t tMs, float altitudeM, float varioMps, float pressurePa);

    size_t count();
    size_t capacity();
    bool usingPsram();

    // Copies points oldest -> newest. dst must hold at least maxPoints.
    size_t snapshot(TrendPoint* dst, size_t maxPoints);
    size_t page(uint32_t sinceMs, TrendPoint* dst, size_t limit, bool& hasMore);
}
