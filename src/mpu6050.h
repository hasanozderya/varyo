#pragma once
#include <Arduino.h>

// Configuration and saved ground calibration are loaded at boot; movement
// during begin() is never used to learn an offset. Ground calibration is
// explicitly requested and sampled incrementally by the owning vario task.
class MPU6050 {
public:
    bool begin();                 // wake, configure, load saved calibration
    void startRecovery();
    bool serviceRecovery();
    static void initStorage();
    static void serviceStorage();
    bool read(float& axG, float& ayG, float& azG,
              float& gxRad, float& gyRad, float& gzRad);
    // True if the most recent read() saw a raw accelerometer axis pinned at
    // (or beyond) the ADC's usable full-scale — e.g. from a sharp hand
    // shake. That sample's magnitude is not trustworthy for anything that
    // integrates acceleration (Kalman predict); attitude tilt-correction
    // already discounts it separately via normTrust/directionTrust.
    bool accelSaturated() const { return accelSaturated_; }
    bool gyroSaturated() const { return gyroSaturated_; }

    enum class CalibrationStatus : uint8_t { Idle, Collecting, Saved, Rejected, SaveFailed, Saving };
    void startGroundCalibration();
    void serviceGroundCalibration(); // call each tick, including missed reads
    bool calibrated() const { return calibrationValid_; }
    bool calibrating() const { return calibrationStatus_ == CalibrationStatus::Collecting || calibrationStatus_ == CalibrationStatus::Saving; }
    void rejectCalibration();
    CalibrationStatus calibrationStatus() const { return calibrationStatus_; }
    bool calibrationTemperatureOk() const;

    // Optional small norm correction learned only by ground calibration.
    float accelBiasG() const { return accelBiasG_; }
    float accelCalibMeanG() const { return accelCalibMeanG_; }
    float accelCalibStdG() const { return accelCalibStdG_; }
    bool  accelScalarBiasAccepted() const { return accelScalarBiasAccepted_; }
    float gyroBiasDps(int axis) const { return gyroBiasDps_[axis]; }
    int   calibrationSamples() const { return calibSamples_; }
    uint32_t runtimeReadMisses() const { return runtimeReadMisses_; }

private:
    bool writeReg(uint8_t reg, uint8_t val);
    bool readReg(uint8_t reg, uint8_t& value);
    bool readBurst14(int16_t raw[7], bool nonBlocking = false);
    void loadCalibration();
    void collectCalibration(const int16_t raw[7]);

    bool calibrationValid_ = false;
    CalibrationStatus calibrationStatus_ = CalibrationStatus::Idle;
    uint32_t calibrationStartMs_ = 0;
    double calSum_[7] = {}, calSumSq_[7] = {};
    float temperatureC_ = 0.0f, calibrationTemperatureC_ = 0.0f;

    float gyroBiasDps_[3] = {0, 0, 0};
    float accelBiasG_     = 0.0f;
    float accelCalibMeanG_ = 0.0f;
    float accelCalibStdG_  = 0.0f;
    bool  accelScalarBiasAccepted_ = false;
    int   calibSamples_   = 0;
    uint32_t runtimeReadMisses_ = 0;   // skipped/failed runtime reads
    uint32_t recoveryMs_ = 0;
    uint8_t recoveryStep_ = 0;
    uint32_t saveTicket_ = 0;
    bool accelSaturated_ = false;
    bool gyroSaturated_ = false;
    // +-8g range: a hand-held shake easily exceeds +-4g peaks and clips the
    // ADC, which used to inject a bogus, asymmetric "acceleration" into the
    // Kalman predict step and permanently mis-train its accel-bias state.
    static constexpr float ACCEL_SCALE = 1.0f / 4096.0f; // +-8g range
    static constexpr float GYRO_SCALE  = 1.0f / 65.5f;   // +-500 dps range
    // Raw int16 LSBs; full scale is +-32768 but real chips saturate a
    // little before that. 32000 gives margin while still catching clipping.
    static constexpr int16_t ACCEL_SATURATION_LSB = 32000;
};
