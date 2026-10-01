#include "imu.h"
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include <freertos/queue.h>

namespace {
struct SavedCalibration {
    uint32_t version, sensorType;
    float gyroDps[3], accelBiasG, temperatureC;
};
struct SaveJob { uint32_t ticket; SavedCalibration data; bool success; };
QueueHandle_t saveJobs = nullptr, saveResults = nullptr;
constexpr uint32_t CALIBRATION_VERSION = 4;
constexpr uint32_t CALIBRATION_MS = 3000;
constexpr int MIN_CALIBRATION_SAMPLES = 150;
void calibrationKey(uint32_t sensorType, char key[12]) {
    snprintf(key, 12, "sensor_%lu", (unsigned long)sensorType);
}
}

void Imu::initStorage() {
    saveJobs = xQueueCreate(1, sizeof(SaveJob));
    saveResults = xQueueCreate(1, sizeof(SaveJob));
    configASSERT(saveJobs && saveResults);
}

void Imu::serviceStorage() {
    SaveJob job{};
    if (!saveJobs || xQueueReceive(saveJobs, &job, 0) != pdTRUE) return;
    Preferences prefs;
    char key[12]; calibrationKey(job.data.sensorType, key);
    job.success = prefs.begin("imu_cal", false) &&
        prefs.putBytes(key, &job.data, sizeof(job.data)) == sizeof(job.data);
    prefs.end();
    xQueueOverwrite(saveResults, &job);
}

bool Imu::begin() {
    loadCalibration();
    return driver_.begin();
}

void Imu::startRecovery() {
    if (calibrating()) rejectCalibration();
    driver_.startRecovery();
}

bool Imu::serviceRecovery() { return driver_.serviceRecovery(); }

void Imu::rejectCalibration() {
    ++saveTicket_;
    calibrationStatus_ = CalibrationStatus::Rejected;
}

void Imu::loadCalibration() {
    calibrationValid_ = false;
    memset(gyroBiasDps_, 0, sizeof(gyroBiasDps_));
    accelBiasG_ = 0;
    accelScalarBiasAccepted_ = false;
    SavedCalibration saved{};
    Preferences prefs;
    if (!prefs.begin("imu_cal", true)) return;
    char key[12]; calibrationKey(VARIO_IMU_TYPE, key);
    const size_t size = prefs.getBytesLength(key);
    const bool loaded = size == sizeof(saved) &&
        prefs.getBytes(key, &saved, sizeof(saved)) == sizeof(saved);
    prefs.end();
    if (!loaded || saved.version != CALIBRATION_VERSION ||
        saved.sensorType != VARIO_IMU_TYPE ||
        !isfinite(saved.accelBiasG) || fabsf(saved.accelBiasG) >= 0.15f ||
        !isfinite(saved.temperatureC) || saved.temperatureC < -40 || saved.temperatureC > 85) return;
    for (float bias : saved.gyroDps)
        if (!isfinite(bias) || fabsf(bias) > 20) return;
    memcpy(gyroBiasDps_, saved.gyroDps, sizeof(gyroBiasDps_));
    accelBiasG_ = saved.accelBiasG;
    accelScalarBiasAccepted_ = true;
    calibrationTemperatureC_ = saved.temperatureC;
    calibrationValid_ = true;
}

bool Imu::calibrationTemperatureOk() const {
    return calibrationValid_ && isfinite(temperatureC_) &&
        fabsf(temperatureC_ - calibrationTemperatureC_) <= 10.0f;
}

void Imu::startGroundCalibration() {
    if (calibrating()) return;
    memset(calSum_, 0, sizeof(calSum_));
    memset(calSumSq_, 0, sizeof(calSumSq_));
    calibSamples_ = 0;
    calibrationStartMs_ = millis();
    calibrationStatus_ = CalibrationStatus::Collecting;
}

void Imu::collectCalibration(const ImuHardwareSample& sample) {
    if (calibrationStatus_ != CalibrationStatus::Collecting) return;
    const float values[7] = {
        sample.gyroDps[0], sample.gyroDps[1], sample.gyroDps[2],
        sample.accelG[0], sample.accelG[1], sample.accelG[2], sample.temperatureC
    };
    for (int i = 0; i < 7; ++i) {
        calSum_[i] += values[i];
        calSumSq_[i] += (double)values[i] * values[i];
    }
    ++calibSamples_;
}

void Imu::serviceGroundCalibration() {
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
        (uint32_t)(millis() - calibrationStartMs_) < CALIBRATION_MS) return;
    calibrationStatus_ = CalibrationStatus::Rejected;
    if (calibSamples_ < MIN_CALIBRATION_SAMPLES) return;
    double mean[7], variance[7];
    for (int i = 0; i < 7; ++i) {
        mean[i] = calSum_[i] / calibSamples_;
        variance[i] = fmax(0.0, calSumSq_[i] / calibSamples_ - mean[i] * mean[i]);
    }
    for (int i = 0; i < 3; ++i)
        if (fabs(mean[i]) > 20 || variance[i] > 0.25 || variance[i + 3] > 0.000064) return;
    accelCalibMeanG_ = (float)sqrt(mean[3] * mean[3] + mean[4] * mean[4] + mean[5] * mean[5]);
    accelCalibStdG_ = (float)sqrt(variance[3] + variance[4] + variance[5]);
    if (accelCalibMeanG_ < 0.85f || accelCalibMeanG_ > 1.15f || mean[6] < -40 || mean[6] > 85) return;
    const float residual = accelCalibMeanG_ - 1.0f;
    if (fabsf(residual) >= 0.15f) return;
    SavedCalibration saved{CALIBRATION_VERSION, VARIO_IMU_TYPE,
        {(float)mean[0], (float)mean[1], (float)mean[2]}, residual, (float)mean[6]};
    const SaveJob job{++saveTicket_, saved, false};
    calibrationStatus_ = saveJobs && xQueueSend(saveJobs, &job, 0) == pdTRUE
        ? CalibrationStatus::Saving : CalibrationStatus::SaveFailed;
}

bool Imu::read(float& axG, float& ayG, float& azG,
               float& gxRad, float& gyRad, float& gzRad) {
    ImuHardwareSample sample{};
    if (!driver_.read(sample)) { ++runtimeReadMisses_; return false; }
    temperatureC_ = sample.temperatureC;
    accelSaturated_ = sample.accelSaturated;
    gyroSaturated_ = sample.gyroSaturated;
    if (accelSaturated_ || gyroSaturated_) {
        if (calibrating()) rejectCalibration();
    } else collectCalibration(sample);
    axG = sample.accelG[0]; ayG = sample.accelG[1]; azG = sample.accelG[2];
    gxRad = (sample.gyroDps[0] - gyroBiasDps_[0]) * DEG_TO_RAD;
    gyRad = (sample.gyroDps[1] - gyroBiasDps_[1]) * DEG_TO_RAD;
    gzRad = (sample.gyroDps[2] - gyroBiasDps_[2]) * DEG_TO_RAD;
    return true;
}
