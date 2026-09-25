#pragma once
#include <stddef.h>
#include <stdint.h>
#include "shared_state.h"

namespace XcTrackProtocol {
    // NMEA limit includes '$', checksum and CRLF, plus our trailing NUL.
    constexpr size_t LINE_BYTES = 83;
    enum class TxMode : uint8_t { Off, Nmea, Json };
    TxMode txMode(uint16_t cccd);
    size_t sentence(const char* body, char* out, size_t capacity);
    size_t encodeVario(const VarioState& state, uint32_t nowMs, char* out, size_t capacity);

    class GpsLineReceiver {
    public:
        enum Result { Pending, Rmc, Gga };
        Result feed(uint8_t byte);
        const char* line() const { return line_; }
    private:
        char line_[LINE_BYTES]{};
        size_t used_ = 0;
        bool collecting_ = false;
    };
    struct GpsFrames {
        char rmc[LINE_BYTES]{}, gga[LINE_BYTES]{};
        uint32_t rmcMs = 0, ggaMs = 0;
    };
    namespace GpsStore {
        void feed(uint8_t byte, uint32_t nowMs); // single writer: GPS task
        GpsFrames read();
    }

    // One bounded batch, no growing backlog or work in the measurement task.
    // Advance only after the BLE stack accepts a notification fragment.
    class Stream {
    public:
        void reset();
        void prepare(uint32_t nowMs, const VarioState& vario, const GpsState& gps,
                     const GpsFrames& frames);
        const char* data() const { return buffer_ + sent_; }
        size_t chunkSize(uint16_t mtu) const;
        void advance(size_t bytes);
    private:
        void append(const char* line);
        char buffer_[256]{};
        size_t length_ = 0, sent_ = 0;
        uint32_t batchMs_ = 0, varioMs_ = 0, gpsMs_ = 0;
        bool started_ = false, gpsStarted_ = false;
    };
}
