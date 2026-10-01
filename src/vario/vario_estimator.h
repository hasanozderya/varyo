#pragma once

#include "barometers/barometer.h"
#include "flight_state.h"
#include "imus/imu.h"
#include "sensor_fusion.h"
#include "shared_state.h"
#include "tunables.h"
#include <math.h>

struct VarioEstimate {
    bool outputReady = false;
    bool barometerHealthy = false;
    bool imuHealthy = false;
    bool fusionActive = false;
    bool lastImuSampleOk = false;
    float altitudeM = 0.0f;
    float barometerAltitudeM = 0.0f;
    float pressurePa = 0.0f;
    float outputVarioMps = 0.0f;
    float flightDetectionVarioMps = 0.0f;
    float barometerVarioMps = 0.0f;
    float kalmanAltitudeM = 0.0f;
    float kalmanVarioMps = 0.0f;
    float earthZAccelMps2 = 0.0f;
    float pitchDeg = 0.0f;
    float rollDeg = 0.0f;
    float kalmanAccelBias = 0.0f;
    float effectiveBarometerVariance = 0.0f;
};

// Pure estimation pipeline: pressure conditioning, attitude, Kalman fusion,
// fallback regression and final output filtering.
class VarioEstimator {
public:
    VarioEstimate update(Barometer& barometer, Imu& imu,
                         bool barometerOk, bool imuOk,
                         uint32_t nowUs, float dt, const Tunables& tunables);
    void calibrationStarted();
    bool updateGroundStability(uint32_t nowMs, bool measurementValid,
                               float outputVarioMps, const GpsState& gps);

    bool barometerReady() const { return barometerPressureReady_; }
    bool haveImuSample() const { return haveImuSample_; }
    uint32_t lastBarometerUs() const { return lastBarometerUs_; }
    uint32_t lastImuUs() const { return lastImuUs_; }
    uint32_t barometerSamples() const { return barometerSamples_; }
    uint32_t imuSamples() const { return imuSamples_; }
    uint32_t referenceRevision() const { return referenceRevision_; }
    float lastAxG() const { return lastAxG_; }
    float lastAyG() const { return lastAyG_; }
    float lastAzG() const { return lastAzG_; }

private:
    void applyQnh(float qnhHpa);

    AttitudeFilter attitude_;
    AltitudeKalman kalman_;
    BaroVarioRegression barometerVario_;
    VarioOutputFilter outputFilter_;
    GroundStability ground_;

    bool barometerPressureReady_ = false;
    bool haveImuSample_ = false;
    bool imuSettling_ = false;
    bool fusionActive_ = false;
    bool imuReferenceUsable_ = false;
    uint32_t imuSettlingStartUs_ = 0;
    uint32_t lastBarometerUs_ = 0;
    uint32_t lastImuUs_ = 0;
    uint32_t barometerSamples_ = 0;
    uint32_t imuSamples_ = 0;
    uint32_t referenceRevision_ = 0;
    float filteredPressurePa_ = 0.0f;
    float barometerAltitudeM_ = 0.0f;
    float fusionBlend_ = 0.0f;
    float lastAppliedQnh_ = NAN;
    float lastAxG_ = 0.0f;
    float lastAyG_ = 0.0f;
    float lastAzG_ = 0.0f;
};
