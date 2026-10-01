#pragma once

#include "barometers/barometer.h"
#include "imus/imu.h"
#include <stdint.h>

// Owns the physical sensors and their bounded, non-blocking recovery policy.
class SensorSupervisor {
public:
    void begin();
    void service(uint32_t nowUs, uint32_t nowMs,
                 bool barometerReady, uint32_t lastBarometerUs,
                 bool haveImuSample, uint32_t lastImuUs);

    Barometer& barometer() { return barometer_; }
    Imu& imu() { return imu_; }
    bool barometerOk() const { return barometerOk_; }
    bool imuOk() const { return imuOk_; }
    uint32_t recoveries() const { return recoveries_; }

private:
    Barometer barometer_;
    Imu imu_;
    bool barometerOk_ = false;
    bool imuOk_ = false;
    bool barometerRecovering_ = false;
    bool imuRecovering_ = false;
    uint32_t barometerAttemptMs_ = 0;
    uint32_t imuAttemptMs_ = 0;
    uint32_t busAttemptMs_ = 0;
    uint32_t recoveries_ = 0;
};
