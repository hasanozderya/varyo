#pragma once
#include <Arduino.h>

// Output smoothing only: no deadband, learned zero or stationary assumption.
// Estimator/flight detection continue using the un-smoothed measurement.
class VarioOutputFilter {
public:
    void reset() { ready_ = false; }
    void rebase(float scale);
    float update(float value, float dt, float tau);
private:
    float value_ = 0;
    bool ready_ = false;
};

// Six-axis attitude: body-to-world quaternion, with bounded gravity correction.
class AttitudeFilter {
public:
    void update(float axG, float ayG, float azG,
                float gxRad, float gyRad, float gzRad, float dt, float alpha);

    float pitchRad() const;
    float rollRad() const;
    bool verticalReferenceReliable() const { return initialized_ && reliable_; }

    // World-frame vertical acceleration with gravity removed, in m/s^2
    // (positive = accelerating upward).
    float earthZAccelMps2(float axG, float ayG, float azG, float accelBiasG) const;

private:
    void rotate(float x, float y, float z, float dt);
    float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;
    bool  initialized_ = false;
    bool reliable_ = false;
    float recoveryQuietS_ = 0;
};

// Independent barometer-only reference variometer.
// It estimates dh/dt with least-squares linear regression over a sliding
// time window using the REAL sample timestamps. It is deliberately
// independent of the MPU6050/Kalman path, so it can be used both as a
// stable fallback and as a diagnostic reference.
class BaroVarioRegression {
public:
    void reset();
    void rebase(float scale, float offset);

    void addSample(float altitudeM,
                   uint32_t timeUs,
                   uint32_t windowUs,
                   uint8_t minSamples);

    bool  ready() const { return ready_; }
    float climbRateMps() const { return climbRateMps_; }
    uint16_t samplesUsed() const { return samplesUsed_; }
    uint32_t spanUs() const { return spanUs_; }

private:
    // 3 seconds at the 100 Hz acquisition rate needs 301 samples, not 128.
    // Keep margin for jitter; these buffers live in static storage, not task stack.
    static constexpr uint16_t MAX_SAMPLES = 512;

    float    altitude_[MAX_SAMPLES] = {};
    uint32_t timeUs_[MAX_SAMPLES]   = {};
    uint16_t head_ = 0;
    uint16_t count_ = 0;

    bool     ready_ = false;
    float    climbRateMps_ = 0.0f;
    uint16_t samplesUsed_ = 0;
    uint32_t spanUs_ = 0;
};

// Stage 2: 3-state Kalman filter (altitude, climb-rate, accel-bias)
// fusing the IMU's high-rate Earth-Z acceleration with the barometer's
// low-rate, drift-free absolute altitude.
class AltitudeKalman {
public:
    void reset();
    void seed(float altitudeM, float climbRateMps, float baroVar);
    void rebase(float scale, float offset);

    void predict(float earthZAccelMps2, float dt, float accelVar, float accelBiasVar, float kAdapt);
    void correct(float baroAltitudeM, float baroVar);

    bool  ready()        const { return ready_; }
    float altitudeM()    const { return altitude_; }
    float climbRateMps() const { return climbRate_; }
    float kalmanGain(float baroVar) const { return p00_ / (p00_ + baroVar); }
    float accelBiasMps2() const { return accelBias_; }

private:
    void initializeCovariance(float baroVar);

    bool  ready_     = false;
    float altitude_  = 0.0f;
    float climbRate_ = 0.0f;
    float accelBias_ = 0.0f;

    float p00_ = 1, p01_ = 0, p02_ = 0;
    float p10_ = 0, p11_ = 0.25f, p12_ = 0;
    float p20_ = 0, p21_ = 0, p22_ = 0.01f;
};
