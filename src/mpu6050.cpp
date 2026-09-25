#include "mpu6050.h"
#include "config.h"
#include "i2c_bus.h"
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include <string.h>
#include <freertos/queue.h>

namespace {
    // One versioned NVS blob keeps all offsets from the same capture.
    struct SavedCalibration {
        uint32_t version;
        float gyroDps[3];
        float accelBiasG;
        float temperatureC;
        float axisScale[3];
        float axisOffset[3];
    };
    // Version 1 did not contain per-axis offsets. Its accelerometer part is
    // intentionally invalid after the affine calibration update, but the
    // already measured gyro zero-rate bias is still useful for proving that
    // the unit is stationary before the replacement calibration is started.
    struct LegacyCalibrationV1 {
        uint32_t version;
        float gyroDps[3];
        float accelBiasG;
        float temperatureC;
        float axisScale[3];
    };
    struct SaveJob { uint32_t ticket; SavedCalibration data; bool success; };
    QueueHandle_t saveJobs = nullptr, saveResults = nullptr;
    constexpr uint32_t CALIBRATION_VERSION = 2;
    constexpr uint32_t CALIBRATION_MS = 3000;
    constexpr int MIN_CALIBRATION_SAMPLES = 150;
    constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    constexpr uint8_t REG_CONFIG     = 0x1A; // DLPF
    constexpr uint8_t REG_GYRO_CFG   = 0x1B;
    constexpr uint8_t REG_ACCEL_CFG  = 0x1C;
    constexpr uint8_t REG_ACCEL_XOUT = 0x3B; // burst start: accel(6) temp(2) gyro(6)
}

void MPU6050::initStorage() {
    saveJobs = xQueueCreate(1, sizeof(SaveJob));
    saveResults = xQueueCreate(1, sizeof(SaveJob));
    configASSERT(saveJobs && saveResults);
}
void MPU6050::serviceStorage() {
    SaveJob job{};
    if (!saveJobs || xQueueReceive(saveJobs, &job, 0) != pdTRUE) return;
    Preferences prefs;
    job.success = prefs.begin("imu_cal", false) &&
        prefs.putBytes("offsets", &job.data, sizeof(job.data)) == sizeof(job.data);
    prefs.end();
    xQueueOverwrite(saveResults, &job);
}
void MPU6050::rejectCalibration() {
    ++saveTicket_;
    calibrationStatus_ = CalibrationStatus::Rejected;
}

bool MPU6050::writeReg(uint8_t reg, uint8_t val) {
    I2CLockGuard lock(0);
    if (!lock.ok()) return false;
    Wire.beginTransmission(I2CAddr::MPU6050);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool MPU6050::readBurst14(int16_t raw[7], bool nonBlocking) {
    I2CLockGuard lock(nonBlocking ? 0 : Timing::I2C_MUTEX_TIMEOUT);
    if (!lock.ok()) return false;
    Wire.beginTransmission(I2CAddr::MPU6050);
    Wire.write(REG_ACCEL_XOUT);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)I2CAddr::MPU6050, (uint8_t)14) != 14) return false;
    for (int i = 0; i < 7; i++) {
        uint8_t hi = Wire.read(), lo = Wire.read();
        raw[i] = (int16_t)(((uint16_t)hi << 8) | lo);
    }
    return true;
}

bool MPU6050::begin() {
    if (!I2CBus::ping(I2CAddr::MPU6050, 0)) return false;
    loadCalibration();
    startRecovery();
    for (unsigned i = 0; i < 40; ++i) {
        if (serviceRecovery()) return true;
        delay(5);
    }
    return false;
}

bool MPU6050::readReg(uint8_t reg, uint8_t& value) {
    I2CLockGuard lock(0);
    if (!lock.ok()) return false;
    Wire.beginTransmission(I2CAddr::MPU6050);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom((uint8_t)I2CAddr::MPU6050, (uint8_t)1) != 1) return false;
    value = Wire.read();
    return true;
}

void MPU6050::startRecovery() {
    recoveryStep_ = 0;
    if (calibrating()) rejectCalibration();
}

bool MPU6050::serviceRecovery() {
    if (recoveryStep_ == 0) {
        if (!writeReg(REG_PWR_MGMT_1, 0x80)) return false;
        recoveryMs_ = millis(); recoveryStep_ = 1;
        return false;
    }
    if (recoveryStep_ == 1) {
        if ((uint32_t)(millis() - recoveryMs_) < 100) return false;
        if (!writeReg(REG_PWR_MGMT_1, 0x01)) return false;
        recoveryMs_ = millis(); recoveryStep_ = 2;
        return false;
    }
    if (recoveryStep_ == 2) {
        if ((uint32_t)(millis() - recoveryMs_) < 50) return false;
        uint8_t id;
        if (!readReg(0x75, id) || id != 0x68) return false;
        if (!writeReg(REG_CONFIG, 0x03) || !writeReg(0x19, 9) ||
            !writeReg(REG_GYRO_CFG, 0x08) || !writeReg(REG_ACCEL_CFG, 0x10)) return false;
        uint8_t dlpf, divider, gyro, accel;
        if (!readReg(REG_CONFIG, dlpf) || !readReg(0x19, divider) ||
            !readReg(REG_GYRO_CFG, gyro) || !readReg(REG_ACCEL_CFG, accel) ||
            dlpf != 0x03 || divider != 9 || gyro != 0x08 || accel != 0x10) return false;
        recoveryStep_ = 3;
    }
    return recoveryStep_ == 3;
}

void MPU6050::loadCalibration() {
    calibrationValid_ = false;
    memset(gyroBiasDps_, 0, sizeof(gyroBiasDps_));
    accelBiasG_ = 0;
    accelScalarBiasAccepted_ = false;
    SavedCalibration saved{};
    LegacyCalibrationV1 legacy{};
    Preferences prefs;
    if (!prefs.begin("imu_cal", true)) return;
    const size_t storedSize = prefs.getBytesLength("offsets");
    const bool loaded = storedSize == sizeof(saved) &&
        prefs.getBytes("offsets", &saved, sizeof(saved)) == sizeof(saved);
    const bool loadedLegacy = storedSize == sizeof(legacy) &&
        prefs.getBytes("offsets", &legacy, sizeof(legacy)) == sizeof(legacy);
    prefs.end();
    if (loadedLegacy && legacy.version == 1) {
        for (float bias : legacy.gyroDps)
            if (!isfinite(bias) || fabsf(bias) > 20) return;
        memcpy(gyroBiasDps_, legacy.gyroDps, sizeof(gyroBiasDps_));
        return;
    }
    if (!loaded || saved.version != CALIBRATION_VERSION ||
        !isfinite(saved.accelBiasG) || fabsf(saved.accelBiasG) >= 0.015f ||
        !isfinite(saved.temperatureC) || saved.temperatureC < -40 || saved.temperatureC > 85 ||
        saved.axisScale[0] != ImuCalibration::ACCEL_SCALE_X ||
        saved.axisScale[1] != ImuCalibration::ACCEL_SCALE_Y ||
        saved.axisScale[2] != ImuCalibration::ACCEL_SCALE_Z ||
        saved.axisOffset[0] != ImuCalibration::ACCEL_OFFSET_X ||
        saved.axisOffset[1] != ImuCalibration::ACCEL_OFFSET_Y ||
        saved.axisOffset[2] != ImuCalibration::ACCEL_OFFSET_Z) return;
    for (float bias : saved.gyroDps)
        if (!isfinite(bias) || fabsf(bias) > 20) return;
    memcpy(gyroBiasDps_, saved.gyroDps, sizeof(gyroBiasDps_));
    accelBiasG_ = saved.accelBiasG;
    accelScalarBiasAccepted_ = true;
    calibrationTemperatureC_ = saved.temperatureC;
    calibrationValid_ = true;
}

bool MPU6050::calibrationTemperatureOk() const {
    // A stored zero-rate offset is temperature dependent. Fall back to baro
    // outside this conservative operating window; do not recalibrate in flight.
    return calibrationValid_ && isfinite(temperatureC_) &&
        fabsf(temperatureC_ - calibrationTemperatureC_) <= 10.0f;
}

void MPU6050::startGroundCalibration() {
    if (calibrating()) return;
    memset(calSum_, 0, sizeof(calSum_));
    memset(calSumSq_, 0, sizeof(calSumSq_));
    calibSamples_ = 0;
    calibrationStartMs_ = millis();
    calibrationStatus_ = CalibrationStatus::Collecting;
}

void MPU6050::collectCalibration(const int16_t raw[7]) {
    if (calibrationStatus_ != CalibrationStatus::Collecting) return;
    const float values[7] = {
        raw[4] * GYRO_SCALE, raw[5] * GYRO_SCALE, raw[6] * GYRO_SCALE,
        (raw[0] * ACCEL_SCALE - ImuCalibration::ACCEL_OFFSET_X) * ImuCalibration::ACCEL_SCALE_X,
        (raw[1] * ACCEL_SCALE - ImuCalibration::ACCEL_OFFSET_Y) * ImuCalibration::ACCEL_SCALE_Y,
        (raw[2] * ACCEL_SCALE - ImuCalibration::ACCEL_OFFSET_Z) * ImuCalibration::ACCEL_SCALE_Z,
        temperatureC_
    };
    for (int i = 0; i < 7; ++i) {
        calSum_[i] += values[i];
        calSumSq_[i] += (double)values[i] * values[i];
    }
    ++calibSamples_;
}

void MPU6050::serviceGroundCalibration() {
    SaveJob result{};
    if (saveResults && xQueueReceive(saveResults, &result, 0) == pdTRUE &&
        result.ticket == saveTicket_ && calibrationStatus_ == CalibrationStatus::Saving) {
        if (result.success) {
            memcpy(gyroBiasDps_, result.data.gyroDps, sizeof(gyroBiasDps_));
            accelBiasG_ = result.data.accelBiasG;
            calibrationTemperatureC_ = result.data.temperatureC;
            accelScalarBiasAccepted_ = calibrationValid_ = true;
            calibrationStatus_ = CalibrationStatus::Saved;
        } else calibrationStatus_ = CalibrationStatus::SaveFailed;
    }
    if (calibrationStatus_ != CalibrationStatus::Collecting ||
        (uint32_t)(millis()-calibrationStartMs_) < CALIBRATION_MS) return;
    calibrationStatus_ = CalibrationStatus::Rejected;
    if (calibSamples_ < MIN_CALIBRATION_SAMPLES) return;
    double mean[7], variance[7];
    for (int i = 0; i < 7; ++i) {
        mean[i] = calSum_[i] / calibSamples_;
        variance[i] = fmax(0.0, calSumSq_[i] / calibSamples_ - mean[i] * mean[i]);
    }
    // These checks reject vibration/rotation, but a steady turn cannot be
    // distinguished from bias here. Therefore this is a ground-only command.
    for (int i = 0; i < 3; ++i) {
        if (fabs(mean[i]) > 20 || variance[i] > 0.25 || variance[i + 3] > 0.000064) return;
    }
    accelCalibMeanG_ = (float)sqrt(mean[3]*mean[3] + mean[4]*mean[4] + mean[5]*mean[5]);
    accelCalibStdG_ = (float)sqrt(variance[3] + variance[4] + variance[5]);
    if (accelCalibMeanG_ < 0.85f || accelCalibMeanG_ > 1.15f ||
        mean[6] < -40 || mean[6] > 85) return;
    const float residual = accelCalibMeanG_ - 1.0f;
    if (fabsf(residual) >= 0.015f) return;
    SavedCalibration saved{CALIBRATION_VERSION,
        {(float)mean[0], (float)mean[1], (float)mean[2]},
        residual, (float)mean[6],
        {ImuCalibration::ACCEL_SCALE_X, ImuCalibration::ACCEL_SCALE_Y, ImuCalibration::ACCEL_SCALE_Z},
        {ImuCalibration::ACCEL_OFFSET_X, ImuCalibration::ACCEL_OFFSET_Y, ImuCalibration::ACCEL_OFFSET_Z}};
    const SaveJob job{++saveTicket_, saved, false};
    calibrationStatus_ = saveJobs && xQueueSend(saveJobs, &job, 0) == pdTRUE
        ? CalibrationStatus::Saving : CalibrationStatus::SaveFailed;
}

bool MPU6050::read(float& axG, float& ayG, float& azG,
                    float& gxRad, float& gyRad, float& gzRad) {
    uint8_t ready;
    if (!readReg(0x3A, ready)) { ++runtimeReadMisses_; return false; }
    if (!(ready & 1)) return false;
    int16_t raw[7];
    if (!readBurst14(raw, true)) {
        runtimeReadMisses_++;
        return false;
    }
    temperatureC_ = raw[3] / 340.0f + 36.53f;
    gyroSaturated_ = abs((int)raw[4]) >= 32000 || abs((int)raw[5]) >= 32000 ||
                     abs((int)raw[6]) >= 32000;
    accelSaturated_ = abs(raw[0]) >= ACCEL_SATURATION_LSB ||
                       abs(raw[1]) >= ACCEL_SATURATION_LSB ||
                       abs(raw[2]) >= ACCEL_SATURATION_LSB;
    if (accelSaturated_ || gyroSaturated_) {
        if (calibrating()) rejectCalibration();
    } else collectCalibration(raw);
    axG = (raw[0] * ACCEL_SCALE - ImuCalibration::ACCEL_OFFSET_X) * ImuCalibration::ACCEL_SCALE_X;
    ayG = (raw[1] * ACCEL_SCALE - ImuCalibration::ACCEL_OFFSET_Y) * ImuCalibration::ACCEL_SCALE_Y;
    azG = (raw[2] * ACCEL_SCALE - ImuCalibration::ACCEL_OFFSET_Z) * ImuCalibration::ACCEL_SCALE_Z;
    gxRad = (raw[4] * GYRO_SCALE - gyroBiasDps_[0]) * DEG_TO_RAD;
    gyRad = (raw[5] * GYRO_SCALE - gyroBiasDps_[1]) * DEG_TO_RAD;
    gzRad = (raw[6] * GYRO_SCALE - gyroBiasDps_[2]) * DEG_TO_RAD;
    return true;
}
