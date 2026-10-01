#include "sensor_supervisor.h"

#include "i2c_bus.h"

void SensorSupervisor::begin() {
    barometerOk_ = barometer_.begin();
    imuOk_ = imu_.begin();
    barometerAttemptMs_ = imuAttemptMs_ = busAttemptMs_ = millis();
}

void SensorSupervisor::service(uint32_t nowUs, uint32_t nowMs,
                               bool barometerReady, uint32_t lastBarometerUs,
                               bool haveImuSample, uint32_t lastImuUs) {
    if ((uint32_t)(nowMs - busAttemptMs_) >= 5000) {
        busAttemptMs_ = nowMs;
        if ((!barometerOk_ || !barometerReady) && (!imuOk_ || !haveImuSample)) {
            I2CBus::recover();
        }
    }

    const bool barometerStale =
        lastBarometerUs != 0 && (uint32_t)(nowUs - lastBarometerUs) >= 2000000;
    if (!barometerRecovering_ && (uint32_t)(nowMs - barometerAttemptMs_) >= 2000 &&
        (!barometerOk_ || !barometerReady || barometerStale)) {
        barometerAttemptMs_ = nowMs;
        barometerRecovering_ = true;
        barometerOk_ = false;
        barometer_.startRecovery();
    }
    if (barometerRecovering_) {
        if (barometer_.serviceRecovery()) {
            barometerOk_ = true;
            barometerRecovering_ = false;
            ++recoveries_;
        } else if ((uint32_t)(nowMs - barometerAttemptMs_) >= 400) {
            barometerRecovering_ = false;
        }
    }

    const bool imuStale = lastImuUs != 0 && (uint32_t)(nowUs - lastImuUs) >= 2000000;
    if (!imuRecovering_ && (uint32_t)(nowMs - imuAttemptMs_) >= 2000 &&
        (!imuOk_ || !haveImuSample || imuStale)) {
        imuAttemptMs_ = nowMs;
        imuRecovering_ = true;
        imuOk_ = false;
        imu_.startRecovery();
    }
    if (imuRecovering_) {
        if (imu_.serviceRecovery()) {
            imuOk_ = true;
            imuRecovering_ = false;
            ++recoveries_;
        } else if ((uint32_t)(nowMs - imuAttemptMs_) >= 400) {
            imuRecovering_ = false;
        }
    }
}
