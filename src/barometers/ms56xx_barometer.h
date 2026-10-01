#pragma once
#include <Arduino.h>
#include "config.h"

// Non-blocking MS5607 driver. update() must be called frequently (every
// vario-task tick, ~100Hz) — it advances an internal state machine and
// only touches the bus when a conversion is actually due/ready, so it
// never blocks the caller for the ~9ms ADC conversion time (and never
// holds the I2C mutex across that wait either).
class MS56xxBarometer {
public:
    MS56xxBarometer();
    // Diagnostic snapshot for startup logging — lets the caller tell
    // "chip not on the bus at all" (coeff all 0/0xFFFF, promReadOk=false)
    // apart from "chip responds but PROM is corrupted / wrong device at
    // this address" (promReadOk=true, crcMatch=false).
    struct Diagnostics {
        uint16_t coeff[8];
        uint8_t  crcExpected;
        uint8_t  crcCalculated;
        bool     promReadOk;
        bool     crcMatch;
    };

    bool begin();                 // reset + read/validate PROM (blocking, call once at startup)
    void startRecovery();
    bool serviceRecovery();      // incremental; never waits for reset/conversion
    void update();                // call every task tick
    bool hasNewSample() const { return newSample_; }
    void clearNewSample()     { newSample_ = false; }

    float pressurePa()   const { return pressurePa_; }
    float temperatureC() const { return temperatureC_; }
    uint32_t rawD1() const { return d1_; }
    uint32_t rawD2() const { return d2_; }
    uint32_t filteredD2() const { return (uint32_t)d2Filtered_; }
    uint32_t runtimeBusMisses() const { return runtimeBusMisses_; }
    uint32_t sampleTimeUs() const { return sampleUs_; }

    Diagnostics diagnostics() const;
    static constexpr const char* modelName() {
#if VARIO_BAROMETER_TYPE == BAROMETER_MS5611
        return "MS5611";
#else
        return "MS5607";
#endif
    }

    // Standard barometric formula relative to a QNH (hPa) reference.
    static float pressureToAltitude(float pressurePa, float qnhHpa);

private:
    bool sendCommand(uint8_t cmd, bool nonBlocking = false);
    uint32_t readAdc(bool nonBlocking = false);
    bool readProm(bool nonBlocking = false);
    void compute();

    uint16_t coeff_[8] = {0};
    uint32_t d1_ = 0, d2_ = 0;
    uint32_t pendingD1_ = 0, pendingD2_ = 0;
    uint8_t pendingD1Count_ = 0, pendingD2Count_ = 0;
    uint32_t lastD2Us_ = 0;
    bool initialized_ = false;
    float d2Filtered_ = 0.0f;
    float pressurePa_   = 101325.0f;
    float temperatureC_ = 20.0f;
    bool  newSample_    = false;

    bool promReadOk_ = false;
    bool crcMatch_   = false;

    enum class Phase : uint8_t { Idle, WaitD1, WaitD2 };
    Phase phase_ = Phase::Idle;
    uint32_t convStartUs_ = 0;
    uint16_t pressureSamplesSinceTemp_ = 1000; // force a temp read first
    uint32_t runtimeBusMisses_ = 0;
    uint32_t sampleUs_ = 0, recoveryUs_ = 0;
    uint8_t recoveryStep_ = 0;
    const bool useMs5611Compensation_;
};
