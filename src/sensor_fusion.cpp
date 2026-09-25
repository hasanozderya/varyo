#include "sensor_fusion.h"
#include "config.h"
#include <math.h>

namespace {
    constexpr float G_MPS2 = 9.80665f;

}

void VarioOutputFilter::rebase(float scale) {
    if (isfinite(scale) && scale > 0) value_ *= scale;
}

float VarioOutputFilter::update(float value, float dt, float tau) {
    if (!isfinite(value)) { reset(); return 0; }
    if (!ready_ || !isfinite(dt) || dt <= 0 || dt > 0.2f || !isfinite(tau) || tau <= 0) {
        value_ = value;
        ready_ = true;
    } else value_ += (1.0f - expf(-dt/tau)) * (value-value_);
    return value_;
}

void AttitudeFilter::rotate(float x, float y, float z, float dt) {
    const float rate = sqrtf(x*x + y*y + z*z);
    const float half = 0.5f * rate * dt;
    const float k = rate > 1e-6f ? sinf(half) / rate : 0.5f * dt;
    const float w = cosf(half), a = x*k, b = y*k, c = z*k;
    const float n0 = q0_*w - q1_*a - q2_*b - q3_*c;
    const float n1 = q0_*a + q1_*w + q2_*c - q3_*b;
    const float n2 = q0_*b - q1_*c + q2_*w + q3_*a;
    const float n3 = q0_*c + q1_*b - q2_*a + q3_*w;
    const float norm = sqrtf(n0*n0 + n1*n1 + n2*n2 + n3*n3);
    if (!isfinite(norm) || norm < 0.1f) { initialized_ = false; return; }
    q0_ = n0/norm; q1_ = n1/norm; q2_ = n2/norm; q3_ = n3/norm;
}

void AttitudeFilter::update(float axG, float ayG, float azG,
                            float gxRad, float gyRad, float gzRad, float dt, float alpha) {
    reliable_ = false;
    if (!isfinite(dt) || dt <= 0.0f || dt > 0.2f || !isfinite(alpha) ||
        !isfinite(gxRad) || !isfinite(gyRad) || !isfinite(gzRad)) {
        recoveryQuietS_ = 0;
        return;
    }
    const float accelNorm = sqrtf(axG * axG + ayG * ayG + azG * azG);
    const bool accelValid = isfinite(accelNorm) && accelNorm >= 0.20f;
    if (!initialized_) {
        if (!accelValid || fabsf(accelNorm-1.0f) > 0.25f) return;
        const float p = 0.5f * atan2f(-axG, sqrtf(ayG*ayG + azG*azG));
        const float r = 0.5f * atan2f(ayG, azG);
        q0_ = cosf(r)*cosf(p); q1_ = sinf(r)*cosf(p);
        q2_ = cosf(r)*sinf(p); q3_ = -sinf(r)*sinf(p);
        initialized_ = true;
        reliable_ = true;
        return;
    }
    rotate(gxRad, gyRad, gzRad, dt);
    if (!initialized_ || !accelValid) { recoveryQuietS_ = 0; return; }
    const float vx = 2.0f*(q1_*q3_ - q0_*q2_);
    const float vy = 2.0f*(q0_*q1_ + q2_*q3_);
    const float vz = 1.0f - 2.0f*(q1_*q1_ + q2_*q2_);
    const float ax = axG/accelNorm, ay = ayG/accelNorm, az = azG/accelNorm;
    // Norm alone cannot detect every coordinated turn. Also limit correction
    // when measured and predicted gravity disagree strongly; never learn a
    // gyro offset from this motion. Barometer remains the drift reference.
    const float dot = constrain(ax*vx + ay*vy + az*vz, -1.0f, 1.0f);
    reliable_ = dot >= 0.8660254f;
    float directionTrust = constrain((dot - 0.8660254f) / 0.0999004f, 0.0f, 1.0f);
    // Recovery of attitude only, never a zero-speed or gyro-bias calibration.
    // Persistent near-1g / low angular rate may occur in moving flight too;
    // the output uses the barometer while the gravity reference is unreliable.
    const float rate = sqrtf(gxRad*gxRad + gyRad*gyRad + gzRad*gzRad);
    recoveryQuietS_ = fabsf(accelNorm-1.0f) < 0.08f && rate < 0.15f
        ? fminf(recoveryQuietS_ + dt, 1.0f) : 0.0f;
    const bool recovering = recoveryQuietS_ >= 0.5f && dot < 0.9659258f;
    if (recovering) directionTrust = fmaxf(directionTrust, 0.5f);
    const float normTrust = constrain(1.0f - fabsf(accelNorm - 1.0f) / 0.25f, 0.0f, 1.0f);
    alpha = constrain(alpha, 0.0f, 0.9999f);
    const float correctionAlpha = recovering ? fminf(alpha, 0.98f) : alpha;
    const float gain = (1.0f - powf(correctionAlpha, dt * Timing::VARIO_TASK_HZ)) * normTrust * directionTrust / dt;
    float ex = ay*vz - az*vy, ey = az*vx - ax*vz, ez = ax*vy - ay*vx;
    if (recovering) {
        float cross = sqrtf(ex*ex + ey*ey + ez*ez);
        if (cross < 1e-5f && dot < 0) {
            // At exactly 180 degrees the cross product is zero: choose an
            // axis perpendicular to measured gravity to escape the deadlock.
            if (fabsf(ax) < 0.9f) { ex = 0; ey = az; ez = -ay; }
            else { ex = -az; ey = 0; ez = ax; }
            cross = sqrtf(ex*ex + ey*ey + ez*ez);
        }
        if (cross > 1e-6f) {
            const float scale = acosf(dot)/cross;
            ex *= scale; ey *= scale; ez *= scale;
        }
    }
    rotate(gain*ex, gain*ey, gain*ez, dt);
}

float AttitudeFilter::pitchRad() const {
    return asinf(constrain(2.0f*(q0_*q2_ - q3_*q1_), -1.0f, 1.0f));
}

float AttitudeFilter::rollRad() const {
    return atan2f(2.0f*(q0_*q1_ + q2_*q3_), 1.0f - 2.0f*(q1_*q1_ + q2_*q2_));
}

float AttitudeFilter::earthZAccelMps2(float axG, float ayG, float azG,
                                      float accelBiasG) const {
    if (!initialized_) return 0.0f;

    const float gxBody = 2.0f*(q1_*q3_ - q0_*q2_);
    const float gyBody = 2.0f*(q0_*q1_ + q2_*q3_);
    const float gzBody = 1.0f - 2.0f*(q1_*q1_ + q2_*q2_);

    const float alongGravityG = axG * gxBody + ayG * gyBody + azG * gzBody;
    const float earthZg = alongGravityG - 1.0f - accelBiasG;
    return constrain(earthZg * G_MPS2, -40.0f, 40.0f);
}

void BaroVarioRegression::rebase(float scale, float offset) {
    if (!isfinite(scale) || scale <= 0 || !isfinite(offset)) return;
    for (uint16_t i = 0; i < count_; ++i) altitude_[i] = scale*altitude_[i] + offset;
    climbRateMps_ *= scale;
}

void BaroVarioRegression::reset() {
    head_ = 0;
    count_ = 0;
    ready_ = false;
    climbRateMps_ = 0.0f;
    samplesUsed_ = 0;
    spanUs_ = 0;
}

void BaroVarioRegression::addSample(float altitudeM,
                                    uint32_t timeUs,
                                    uint32_t windowUs,
                                    uint8_t minSamples) {
    if (!isfinite(altitudeM)) return;

    altitude_[head_] = altitudeM;
    timeUs_[head_] = timeUs;
    head_ = (uint16_t)((head_ + 1) % MAX_SAMPLES);
    if (count_ < MAX_SAMPLES) count_++;

    double sumT = 0.0;
    double sumH = 0.0;
    double sumTT = 0.0;
    double sumTH = 0.0;
    uint16_t n = 0;
    uint32_t oldestAgeUs = 0;

    for (uint16_t j = 0; j < count_; ++j) {
        const uint16_t idx = (uint16_t)((head_ + MAX_SAMPLES - 1 - j) % MAX_SAMPLES);
        const uint32_t ageUs = timeUs - timeUs_[idx];
        if (ageUs > windowUs) break;

        const double t = -(double)ageUs * 1e-6;
        const double h = altitude_[idx];

        sumT  += t;
        sumH  += h;
        sumTT += t * t;
        sumTH += t * h;
        oldestAgeUs = ageUs;
        n++;
    }

    samplesUsed_ = n;
    spanUs_ = oldestAgeUs;
    if (n < minSamples || oldestAgeUs < 250000UL) {
        ready_ = false;
        climbRateMps_ = 0.0f;
        return;
    }

    const double denom = (double)n * sumTT - sumT * sumT;
    if (fabs(denom) < 1e-12) {
        ready_ = false;
        return;
    }

    const double slope = ((double)n * sumTH - sumT * sumH) / denom;
    climbRateMps_ = constrain((float)slope, -20.0f, 20.0f);
    ready_ = true;
}

void AltitudeKalman::initializeCovariance(float baroVar) {
    const float r = fmaxf(baroVar, 1e-4f);

    // Keep velocity covariance bounded. Moving-start velocity is seeded
    // from barometer regression before this estimator drives the output.
    p00_ = r;
    p01_ = 0.0f;
    p02_ = 0.0f;

    p10_ = 0.0f;
    p11_ = 0.25f;
    p12_ = 0.0f;

    p20_ = 0.0f;
    p21_ = 0.0f;
    p22_ = 0.01f;
}

void AltitudeKalman::reset() {
    ready_ = false;
    altitude_ = climbRate_ = accelBias_ = 0.0f;
    initializeCovariance(0.05f);
}

void AltitudeKalman::seed(float altitudeM, float climbRateMps, float baroVar) {
    reset();
    if (!isfinite(altitudeM) || !isfinite(climbRateMps)) return;
    correct(altitudeM, baroVar);
    climbRate_ = climbRateMps;
}

void AltitudeKalman::rebase(float scale, float offset) {
    if (!ready_ || !isfinite(scale) || scale <= 0 || !isfinite(offset)) return;
    altitude_ = scale*altitude_ + offset;
    climbRate_ *= scale;
    const float s2 = scale*scale;
    p00_ *= s2; p01_ *= s2; p10_ *= s2; p11_ *= s2;
    p02_ *= scale; p20_ *= scale; p12_ *= scale; p21_ *= scale;
}

void AltitudeKalman::predict(float earthZAccelMps2, float dt,
                             float accelVar, float accelBiasVar, float kAdapt) {
    if (!ready_) return;
    if (!isfinite(dt) || dt <= 0.0f || dt > 0.2f || !isfinite(earthZAccelMps2) ||
        !isfinite(accelVar) || accelVar <= 0 || accelVar > 1000 ||
        !isfinite(accelBiasVar) || accelBiasVar < 0 || accelBiasVar > 10 ||
        !isfinite(kAdapt) || kAdapt < 0 || kAdapt > 10) return;

    const float a = earthZAccelMps2 - accelBias_;
    const float accel_ext = fabsf(a); 
    const float dynamicBiasVar = accelBiasVar / (1.0f + accel_ext);
    const float dynamicAccelVar = accelVar + (kAdapt * accel_ext * accel_ext);

    altitude_  += climbRate_ * dt + 0.5f * a * dt * dt;
    climbRate_ += a * dt;
    climbRate_ = constrain(climbRate_, -25.0f, 25.0f);

    const float d = dt;
    const float e = -0.5f * dt * dt;
    const float f = -dt;

    const float r00 = p00_ + d * p10_ + e * p20_;
    const float r01 = p01_ + d * p11_ + e * p21_;
    const float r02 = p02_ + d * p12_ + e * p22_;
    const float r10 = p10_ + f * p20_;
    const float r11 = p11_ + f * p21_;
    const float r12 = p12_ + f * p22_;
    const float r20 = p20_;
    const float r21 = p21_;
    const float r22 = p22_;

    p00_ = r00 + d * r01 + e * r02;
    p01_ = r01 + f * r02;
    p02_ = r02;

    p10_ = r10 + d * r11 + e * r12;
    p11_ = r11 + f * r12;
    p12_ = r12;

    p20_ = r20 + d * r21 + e * r22;
    p21_ = r21 + f * r22;
    p22_ = r22;

    const float dt2 = dt * dt;
    
    p00_ += 0.25f * dt2 * dt2 * dynamicAccelVar;
    p01_ += 0.5f  * dt2 * dt  * dynamicAccelVar;
    p10_ += 0.5f  * dt2 * dt  * dynamicAccelVar;
    p11_ += dt2 * dynamicAccelVar;
    p22_ += dynamicBiasVar * dt;

    p10_ = p01_ = 0.5f * (p01_ + p10_);
    p20_ = p02_ = 0.5f * (p02_ + p20_);
    p21_ = p12_ = 0.5f * (p12_ + p21_);
}

void AltitudeKalman::correct(float baroAltitudeM, float baroVar) {
    if (!isfinite(baroAltitudeM) || !isfinite(baroVar) || baroVar <= 0) return;
    baroVar = fmaxf(baroVar, 1e-4f);

    if (!ready_) {
        altitude_ = baroAltitudeM;
        climbRate_ = 0.0f;
        accelBias_ = 0.0f;
        initializeCovariance(baroVar);
        ready_ = true;
        return;
    }

    const float innovation = baroAltitudeM - altitude_;
    const float nominalS = p00_ + baroVar;
    // Robust measurement weighting, not a zero-velocity reset. An isolated
    // pressure spike is weak evidence, even if it is >15 m. Inflate R for
    // >3-sigma innovations and use that SAME R in the covariance update.
    // Clipping only the residual while shrinking P as if fully trusted
    // makes the filter overconfident after a run of bad measurements.
    const float requiredS = innovation * innovation / 9.0f;
    if (!isfinite(nominalS) || !isfinite(requiredS)) return;
    if (requiredS > nominalS) baroVar = fmaxf(baroVar, requiredS - p00_);
    const float S = p00_ + baroVar;

    const float k0 = p00_ / S;
    const float k1 = p10_ / S;
    const float k2 = p20_ / S;

    altitude_  += k0 * innovation;
    climbRate_ += k1 * innovation;
    climbRate_ = constrain(climbRate_, -25.0f, 25.0f);
    accelBias_ += k2 * innovation;
    accelBias_ = constrain(accelBias_, -VarioTuning::KF_BIAS_LIMIT_MPS2,
                           VarioTuning::KF_BIAS_LIMIT_MPS2);

    // Joseph form: (I-KH) P (I-KH)' + K R K'. Retain nonnegative
    // variances without subtracting nearly equal floats on each update.
    const float a0 = 1.0f - k0;
    const float p00 = a0*a0*p00_ + k0*k0*baroVar;
    const float p01 = a0*(p01_ - k1*p00_) + k0*k1*baroVar;
    const float p02 = a0*(p02_ - k2*p00_) + k0*k2*baroVar;
    const float p11 = p11_ - 2*k1*p01_ + k1*k1*(p00_ + baroVar);
    const float p12 = p12_ - k1*p02_ - k2*p01_ + k1*k2*(p00_ + baroVar);
    const float p22 = p22_ - 2*k2*p02_ + k2*k2*(p00_ + baroVar);
    p00_ = fmaxf(p00, 0.0f); p01_ = p10_ = p01; p02_ = p20_ = p02;
    p11_ = fmaxf(p11, 0.0f); p12_ = p21_ = p12; p22_ = fmaxf(p22, 0.0f);

    p10_ = p01_ = 0.5f * (p01_ + p10_);
    p20_ = p02_ = 0.5f * (p02_ + p20_);
    p21_ = p12_ = 0.5f * (p12_ + p21_);
}
