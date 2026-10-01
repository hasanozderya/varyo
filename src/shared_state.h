#pragma once
#include <Arduino.h>

// Snapshot of the fused vario state, written at 100Hz by the Core-0
// vario task and read at ~10Hz by the Core-1 UI task. Protected by a
// spinlock (portMUX) since hold times are a handful of float copies —
// far too short to justify a semaphore, and portMUX is the correct
// ESP32 idiom for very short cross-core critical sections.
struct VarioState {

    float altitudeM        = 0.0f;   // Kalman altitude
    float climbRateMps     = 0.0f;   // Kalman vertical speed

    float earthZAccelMps2  = 0.0f;   // gravity-removed vertical acceleration
    float kalmanAccelBias  = 0.0f;   // Kalman's estimated accel bias
    float baroAltitudeM    = 0.0f;   // filtered baro-derived altitude
    float pressurePa       = 0.0f;   // filtered barometer pressure used by altitude calibration

    float pitchDeg         = 0.0f;
    float rollDeg          = 0.0f;

    bool baroOk            = false;
    bool imuOk             = false;
    bool imuCalibrated     = false;
    bool imuFusionActive   = false;
    uint8_t imuCalibrationStatus = 0;
    bool outputReady       = false;
    bool groundStable      = false;
    uint32_t updatedMs     = 0;
    uint32_t sampleSequence = 0;
    uint32_t baroSampleMs  = 0;
    uint32_t imuSampleMs   = 0;
    uint32_t longTicks     = 0;
    uint32_t maxTickUs     = 0;
    uint32_t baroSamples   = 0;
    uint32_t imuSamples    = 0;
    uint32_t sensorRecoveries = 0;
    uint32_t referenceRevision = 0;
    uint32_t stackFreeBytes = 0;
    uint8_t flightMode = 0;
    uint32_t flightSession = 0, flightDurationMs = 0;
};

struct GpsState {
    double latitude = 0, longitude = 0;
    float altitudeM = 0, speedMps = 0, courseDeg = 0, hdop = 99;
    uint32_t fixMs = 0, motionMs = 0, altitudeMs = 0;
    uint32_t utcSeconds = 0, utcSampleMs = 0;
    uint8_t satellites = 0;
    bool fix = false, motionValid = false, altitudeValid = false, utcValid = false;
};

struct BleLinkState {
    bool connected = false, authenticated = false, pairing = false;
    uint32_t passkey = 0, pairingMs = 0;
};

namespace SharedState {
    void init();
    void write(const VarioState& s);
    VarioState read();
    VarioState readFresh(uint32_t nowMs);
    void writeGps(const GpsState& gps);
    GpsState readGps(uint32_t nowMs);
    void writeLink(const BleLinkState& link);
    BleLinkState readLink();
}
