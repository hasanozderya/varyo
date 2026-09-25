#include "ble_protocol.h"
#include <string.h>
#include <math.h>

BleProtocol::LineReceiver::Result BleProtocol::LineReceiver::feed(uint8_t byte) {
    if (byte == '\n') {
        if (discard_) { reset(); return Invalid; }
        if (length_ && buffer_[length_ - 1] == '\r') --length_;
        buffer_[length_] = '\0';
        const bool nonempty = length_ != 0;
        length_ = 0;
        return nonempty ? Complete : Pending;
    }
    if (discard_) return Pending;
    if (byte == 0 || length_ >= MAX_COMMAND - 1) {
        discard_ = true;
        return Pending;
    }
    buffer_[length_++] = (char)byte;
    return Pending;
}

namespace {
    void put32(uint8_t* out, uint32_t value) {
        for (unsigned i = 0; i < 4; ++i) out[i] = (uint8_t)(value >> (8 * i));
    }
    void put16(uint8_t* out, uint16_t value) { out[0] = (uint8_t)value; out[1] = (uint8_t)(value >> 8); }
    void putFloat(uint8_t* out, float value) {
        static_assert(sizeof(float) == 4, "Protocol needs float32");
        uint32_t bits;
        memcpy(&bits, &value, sizeof(bits));
        put32(out, bits);
    }
}

void BleProtocol::encodeTelemetry(const VarioState& s, uint16_t sequence, uint32_t timeMs,
                                  uint8_t out[TELEMETRY_BYTES]) {
    out[0] = 1; // protocol version
    // Bit 0 remains compatible with old apps: only usable measurements.
    out[1] = (s.baroOk && s.outputReady ? 1 : 0) | (s.imuOk ? 2 : 0) | (s.imuCalibrated ? 4 : 0) |
        (s.imuFusionActive ? 8 : 0) | (s.imuCalibrationStatus == 1 || s.imuCalibrationStatus == 5 ? 16 : 0) |
        32 | ((s.referenceRevision & 1) ? 64 : 0);
    out[2] = (uint8_t)sequence;
    out[3] = (uint8_t)(sequence >> 8);
    (void)timeMs;
    put32(out + 4, s.updatedMs); // measurement time, never a fresh label on stale data
    putFloat(out + 8, s.altitudeM);
    putFloat(out + 12, s.climbRateMps);
    putFloat(out + 16, s.pressurePa);
}

void BleProtocol::encodeGps(const GpsState& g, uint8_t out[20]) {
    memset(out, 0, 20);
    const bool fix = g.fix && isfinite(g.latitude) && isfinite(g.longitude) &&
        fabs(g.latitude) <= 90 && fabs(g.longitude) <= 180;
    const bool motion = fix && g.motionValid && isfinite(g.speedMps) && g.speedMps >= 0 &&
        g.speedMps <= 600 && isfinite(g.courseDeg) && g.courseDeg >= 0 && g.courseDeg < 360;
    const uint32_t age = millis()-g.fixMs;
    const uint8_t ageSteps = (uint8_t)min((uint32_t)15, age/200 + (age%200 != 0));
    out[0] = 1;
    out[1] = (fix ? 1 : 0) | (motion ? 2 : 0) | (ageSteps << 4);
    out[2] = g.satellites;
    out[3] = isfinite(g.hdop) ? (uint8_t)lroundf(constrain(g.hdop, 0.0f, 25.5f)*10) : 255;
    if (fix) {
        put32(out+4, (uint32_t)(int32_t)llround(g.latitude*1e7));
        put32(out+8, (uint32_t)(int32_t)llround(g.longitude*1e7));
    }
    put32(out+12, g.fixMs); // stable fix identity; duplicate notifications aren't new positions
    if (motion) {
        put16(out+16, (uint16_t)lroundf(g.speedMps*100));
        put16(out+18, (uint16_t)constrain((int)lroundf(g.courseDeg*100), 0, 35999));
    }
}

void BleProtocol::encodeHealth(const VarioState& s, uint32_t nowMs, uint8_t out[20]) {
    memset(out, 0, 20);
    out[0] = 1;
    out[1] = 1 | (s.groundStable ? 2 : 0) | (s.outputReady ? 4 : 0); // bit 0: authenticated writes required
    out[2] = !s.sampleSequence || (uint32_t)(nowMs-s.updatedMs) >= 500 ? 1 :
        !s.baroOk ? 2 : !s.outputReady ? 3 : !s.imuOk ? 4 : !s.imuCalibrated ? 5 :
        !s.imuFusionActive ? 6 : 0;
    out[3] = s.flightMode;
    put32(out+4, s.updatedMs);
    put16(out+8, (uint16_t)min((uint32_t)65535, (uint32_t)(nowMs-s.baroSampleMs)));
    put16(out+10, (uint16_t)min((uint32_t)65535, (uint32_t)(nowMs-s.imuSampleMs)));
    put32(out+12, s.referenceRevision);
    out[18] = 2; // firmware major/minor
    out[19] = 1;
}
