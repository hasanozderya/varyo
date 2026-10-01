#pragma once
#include <Arduino.h>
#include "config.h"
#if VARIO_IMU_TYPE == IMU_MPU6050
#include "mpu6050.h"
using SelectedImuDriver = Mpu6050Driver;
#elif VARIO_IMU_TYPE == IMU_LSM6DS3 || \
      VARIO_IMU_TYPE == IMU_LSM6DS3TR || \
      VARIO_IMU_TYPE == IMU_LSM6DSOX
#include "lsm6ds_family.h"
using SelectedImuDriver = Lsm6dsDriver;
#elif VARIO_IMU_TYPE == IMU_BMI160
#include "bmi160.h"
using SelectedImuDriver = Bmi160Driver;
#elif VARIO_IMU_TYPE == IMU_BMI270
#include "bmi270.h"
using SelectedImuDriver = Bmi270Driver;
#elif VARIO_IMU_TYPE == IMU_BNO055
#include "bno055.h"
using SelectedImuDriver = Bno055Driver;
#else
#error Unsupported VARIO_IMU_TYPE
#endif

class Imu {
public:
    bool begin();
    void startRecovery();
    bool serviceRecovery();
    static void initStorage();
    static void serviceStorage();
    bool read(float& axG, float& ayG, float& azG,
              float& gxRad, float& gyRad, float& gzRad);
    static const char* modelName() { return SelectedImuDriver::modelName(); }
    bool accelSaturated() const { return accelSaturated_; }
    bool gyroSaturated() const { return gyroSaturated_; }

    enum class CalibrationStatus : uint8_t { Idle, Collecting, Saved, Rejected, SaveFailed, Saving };
    void startGroundCalibration();
    void serviceGroundCalibration();
    bool calibrated() const { return calibrationValid_; }
    bool calibrating() const { return calibrationStatus_ == CalibrationStatus::Collecting || calibrationStatus_ == CalibrationStatus::Saving; }
    void rejectCalibration();
    CalibrationStatus calibrationStatus() const { return calibrationStatus_; }
    bool calibrationTemperatureOk() const;
    float accelBiasG() const { return accelBiasG_; }
    float accelCalibMeanG() const { return accelCalibMeanG_; }
    float accelCalibStdG() const { return accelCalibStdG_; }
    bool accelScalarBiasAccepted() const { return accelScalarBiasAccepted_; }
    float gyroBiasDps(int axis) const { return gyroBiasDps_[axis]; }
    int calibrationSamples() const { return calibSamples_; }
    uint32_t runtimeReadMisses() const { return runtimeReadMisses_; }

private:
    void loadCalibration();
    void collectCalibration(const ImuHardwareSample& sample);
    SelectedImuDriver driver_;
    bool calibrationValid_ = false;
    CalibrationStatus calibrationStatus_ = CalibrationStatus::Idle;
    uint32_t calibrationStartMs_ = 0;
    double calSum_[7] = {}, calSumSq_[7] = {};
    float temperatureC_ = 0.0f, calibrationTemperatureC_ = 0.0f;
    float gyroBiasDps_[3] = {0, 0, 0};
    float accelBiasG_ = 0.0f;
    float accelCalibMeanG_ = 0.0f, accelCalibStdG_ = 0.0f;
    bool accelScalarBiasAccepted_ = false;
    int calibSamples_ = 0;
    uint32_t runtimeReadMisses_ = 0, saveTicket_ = 0;
    bool accelSaturated_ = false, gyroSaturated_ = false;
};
