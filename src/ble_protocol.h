#pragma once
#include <stddef.h>
#include <stdint.h>
#include "shared_state.h"

namespace BleProtocol {
    constexpr size_t MAX_COMMAND = 512; // includes terminating NUL
    constexpr size_t TELEMETRY_BYTES = 20;
    constexpr const char* SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
    constexpr const char* RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
    constexpr const char* TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
    constexpr const char* TELEMETRY_UUID = "6e400004-b5a3-f393-e0a9-e50e24dcca9e";
    constexpr const char* GPS_UUID = "6e400005-b5a3-f393-e0a9-e50e24dcca9e";
    constexpr const char* HEALTH_UUID = "6e400006-b5a3-f393-e0a9-e50e24dcca9e";

    // Feed arbitrary GATT chunks; only LF commits a complete command.
    // Overflow or embedded NUL discards the ENTIRE line through its LF.
    class LineReceiver {
    public:
        enum Result { Pending, Complete, Invalid };
        Result feed(uint8_t byte);
        const char* line() const { return buffer_; }
        void reset() { length_ = 0; discard_ = false; }
    private:
        char buffer_[MAX_COMMAND] = {};
        size_t length_ = 0;
        bool discard_ = false;
    };

    void encodeTelemetry(const VarioState& state, uint16_t sequence, uint32_t timeMs,
                         uint8_t out[TELEMETRY_BYTES]);
    void encodeGps(const GpsState& gps, uint8_t out[20]);
    void encodeHealth(const VarioState& state, uint32_t nowMs, uint8_t out[20]);
}
