#include "trend_buffer.h"
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <string.h>

namespace {
    TrendPoint* s_buf = nullptr;
    size_t s_head = 0;   // next write position
    size_t s_count = 0;
    uint32_t s_writes = 0, s_generation = 0;
    bool s_psram = false;
    portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
}

bool TrendBuffer::init() {
    if (s_buf) return true;

    const size_t bytes = CAPACITY * sizeof(TrendPoint);

    // Prefer external PSRAM when present.
    s_buf = static_cast<TrendPoint*>(
        heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
    );
    s_psram = (s_buf != nullptr);

    // Boards without PSRAM still have enough room for ~29 kB on ESP32-S3.
    if (!s_buf) {
        s_buf = static_cast<TrendPoint*>(
            heap_caps_malloc(bytes, MALLOC_CAP_8BIT)
        );
        s_psram = false;
    }

    if (!s_buf) return false;

    memset(s_buf, 0, bytes);
    clear();
    return true;
}

void TrendBuffer::clear() {
    portENTER_CRITICAL(&s_mux);
    s_head = 0;
    s_count = 0;
    s_writes = 0;
    ++s_generation;
    portEXIT_CRITICAL(&s_mux);
}

void TrendBuffer::add(uint32_t tMs, float altitudeM, float varioMps, float pressurePa) {
    if (!s_buf && !init()) return;

    TrendPoint p{tMs, altitudeM, varioMps, pressurePa};

    portENTER_CRITICAL(&s_mux);
    s_buf[s_head] = p;
    s_head = (s_head + 1) % CAPACITY;
    if (s_count < CAPACITY) s_count++;
    ++s_writes;
    portEXIT_CRITICAL(&s_mux);
}

size_t TrendBuffer::page(uint32_t sinceMs, TrendPoint* dst, size_t limit, bool& hasMore) {
    hasMore = false;
    if (!dst || !limit || !s_buf) return 0;
    portENTER_CRITICAL(&s_mux);
    const size_t count = s_count, start = (s_head+CAPACITY-count)%CAPACITY;
    const uint32_t end = s_writes, generation = s_generation;
    portEXIT_CRITICAL(&s_mux);
    size_t copied = 0;
    for (size_t i = 0; i < count; ++i) {
        // One point per critical section; never copy the whole history while
        // interrupts are disabled. Ignore entries overwritten by the producer.
        portENTER_CRITICAL(&s_mux);
        const bool reset = s_generation != generation;
        const bool overwritten = (uint32_t)(s_writes-(end-(uint32_t)count+(uint32_t)i)) > CAPACITY;
        const TrendPoint point = s_buf[(start+i)%CAPACITY];
        portEXIT_CRITICAL(&s_mux);
        if (reset) { hasMore = false; return 0; }
        if (overwritten || (sinceMs && (int32_t)(point.tMs-sinceMs) <= 0)) continue;
        if (copied == limit) { hasMore = true; break; }
        dst[copied++] = point;
    }
    return copied;
}

size_t TrendBuffer::count() {
    portENTER_CRITICAL(&s_mux);
    const size_t n = s_count;
    portEXIT_CRITICAL(&s_mux);
    return n;
}

size_t TrendBuffer::capacity() {
    return CAPACITY;
}

bool TrendBuffer::usingPsram() {
    return s_psram;
}

size_t TrendBuffer::snapshot(TrendPoint* dst, size_t maxPoints) {
    if (!dst || !s_buf || maxPoints == 0) return 0;

    portENTER_CRITICAL(&s_mux);

    size_t n = s_count;
    if (n > maxPoints) n = maxPoints;

    // If caller asks for fewer than all points, return the newest n.
    size_t start = (s_head + CAPACITY - n) % CAPACITY;

    const size_t first = (start + n <= CAPACITY) ? n : (CAPACITY - start);
    memcpy(dst, &s_buf[start], first * sizeof(TrendPoint));
    if (first < n) {
        memcpy(dst + first, &s_buf[0], (n - first) * sizeof(TrendPoint));
    }

    portEXIT_CRITICAL(&s_mux);
    return n;
}
