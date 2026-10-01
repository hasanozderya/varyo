#include "vario_estimator.h"

#include "config.h"
#include "debug_log.h"
#include "i2c_bus.h"
#include "shared_state.h"
#include "trend_buffer.h"
#include <math.h>

namespace {
constexpr uint32_t BAROMETER_TIMEOUT_US = 500000;
constexpr uint32_t IMU_TIMEOUT_US = 200000;
}

void VarioEstimator::calibrationStarted() {
    imuSettling_ = false;
    fusionActive_ = false;
    outputFilter_.reset();
}

bool VarioEstimator::updateGroundStability(uint32_t nowMs, bool measurementValid,
                                           float outputVarioMps,
                                           const GpsState& gps) {
    return ground_.update(nowMs, measurementValid, outputVarioMps,
                          gps.motionValid, gps.speedMps);
}

void VarioEstimator::applyQnh(float qnhHpa) {
    if (!isfinite(lastAppliedQnh_)) {
        lastAppliedQnh_ = qnhHpa;
        return;
    }
    if (fabsf(qnhHpa - lastAppliedQnh_) <= 0.001f) return;

    const float scale = powf(lastAppliedQnh_ / qnhHpa, 0.190295f);
    const float offset = 44330.0f * (1.0f - scale);
    kalman_.rebase(scale, offset);
    barometerVario_.rebase(scale, offset);
    outputFilter_.rebase(scale);
    barometerAltitudeM_ = scale * barometerAltitudeM_ + offset;
    lastAppliedQnh_ = qnhHpa;
    ++referenceRevision_;
    TrendBuffer::clear();
}

VarioEstimate VarioEstimator::update(Barometer& barometer, Imu& imu,
                                     bool barometerOk, bool imuOk,
                                     uint32_t nowUs, float dt,
                                     const Tunables& tunables) {
    if (dt > 0.2f) {
        fusionActive_ = false;
        imuSettling_ = false;
    }

    applyQnh(tunables.fusion.qnhHpa);
    const float effectiveBarometerVariance =
        fmaxf(tunables.fusion.kfBaroVar, VarioTuning::KF_BARO_VAR_FLOOR);

    if (barometerPressureReady_ &&
        (uint32_t)(nowUs - lastBarometerUs_) >= BAROMETER_TIMEOUT_US) {
        barometerPressureReady_ = false;
        kalman_.reset();
        barometerVario_.reset();
        fusionActive_ = false;
    }

    if (barometerOk) barometer.update();
    if (barometer.hasNewSample()) {
        barometer.clearNewSample();
        ++barometerSamples_;
        const uint32_t sampleUs = barometer.sampleTimeUs();
        const float rawPressurePa = barometer.pressurePa();

        if (!barometerPressureReady_ || !isfinite(filteredPressurePa_)) {
            filteredPressurePa_ = rawPressurePa;
            lastBarometerUs_ = sampleUs;
            barometerPressureReady_ = true;
        } else {
            const float barometerDt = (sampleUs - lastBarometerUs_) * 1.0e-6f;
            lastBarometerUs_ = sampleUs;
            if (barometerDt <= 0.0f || barometerDt > 1.0f) {
                filteredPressurePa_ = rawPressurePa;
            } else {
                const float alpha = 1.0f -
                    expf(-barometerDt / VarioTuning::BARO_PRESSURE_LPF_TAU_S);
                filteredPressurePa_ += alpha * (rawPressurePa - filteredPressurePa_);
            }
        }

        barometerAltitudeM_ = Barometer::pressureToAltitude(
            filteredPressurePa_, tunables.fusion.qnhHpa);
        barometerVario_.addSample(
            barometerAltitudeM_, sampleUs,
            VarioTuning::BARO_REGRESSION_WINDOW_MS * 1000UL,
            VarioTuning::BARO_REGRESSION_MIN_SAMPLES);
        kalman_.correct(barometerAltitudeM_, effectiveBarometerVariance);
    }

    float earthZ = 0.0f;
    bool lastImuSampleOk = false;
    float accelInput = kalman_.accelBiasMps2();
    if (imuOk) {
        float ax, ay, az, gx, gy, gz;
        if (imu.read(ax, ay, az, gx, gy, gz)) {
            const uint32_t imuUs = micros();
            if (!haveImuSample_ || (uint32_t)(imuUs - lastImuUs_) >= IMU_TIMEOUT_US) {
                attitude_ = AttitudeFilter{};
                imuSettling_ = false;
                fusionActive_ = false;
            }
            const float imuDt = haveImuSample_ ? (imuUs - lastImuUs_) * 1e-6f : 0.01f;
            lastImuUs_ = imuUs;
            ++imuSamples_;
            haveImuSample_ = true;
            lastImuSampleOk = true;
            lastAxG_ = ax;
            lastAyG_ = ay;
            lastAzG_ = az;

            if (imu.gyroSaturated()) {
                attitude_ = AttitudeFilter{};
            } else {
                attitude_.update(ax, ay, az, gx, gy, gz, imuDt,
                                 tunables.fusion.compFilterAlpha);
            }

            const float norm = sqrtf(ax * ax + ay * ay + az * az);
            const float gyroRate = sqrtf(gx * gx + gy * gy + gz * gz);
            imuReferenceUsable_ = !imu.accelSaturated() && !imu.gyroSaturated() &&
                isfinite(norm) && norm >= 0.2f && norm < 3.0f &&
                attitude_.verticalReferenceReliable();
            if (imuReferenceUsable_) {
                earthZ = attitude_.earthZAccelMps2(ax, ay, az, imu.accelBiasG());
                accelInput = earthZ;
            }
            ground_.observeImu(millis(), imuReferenceUsable_ &&
                fabsf(norm - 1.0f) < 0.025f && gyroRate < 3.0f * DEG_TO_RAD);
        }
    }

    I2CBus::sensorCycleComplete();
    kalman_.predict(accelInput, dt, tunables.fusion.kfAccelVar,
                    tunables.fusion.kfAccelBiasVar,
                    tunables.fusion.kAdaptFactor);

    const uint32_t healthUs = micros();
    const bool barometerHealthy = barometerOk && barometerPressureReady_ &&
        (uint32_t)(healthUs - lastBarometerUs_) < BAROMETER_TIMEOUT_US;
    const bool imuHealthy = imuOk && haveImuSample_ &&
        (uint32_t)(healthUs - lastImuUs_) < IMU_TIMEOUT_US;

    const bool wasCalibrating = imu.calibrating();
    imu.serviceGroundCalibration();
    if (wasCalibrating && !imu.calibrating()) {
        attitude_ = AttitudeFilter{};
        imuSettling_ = false;
        fusionActive_ = false;
        DebugLog::logf(">> IMU yer kalibrasyonu sonucu=%u, ornek=%d\n",
                       (unsigned)imu.calibrationStatus(), imu.calibrationSamples());
    }

    const bool imuUsable = imuHealthy && imuReferenceUsable_ && imu.calibrated() &&
        imu.calibrationTemperatureOk() && !imu.calibrating();
    if (!imuUsable) {
        imuSettling_ = false;
    } else if (!imuSettling_) {
        imuSettling_ = true;
        imuSettlingStartUs_ = healthUs;
    }

    const bool useFusion = !VarioTuning::USE_BARO_REGRESSION_OUTPUT && imuUsable &&
        imuSettling_ &&
        (fusionActive_ || (uint32_t)(healthUs - imuSettlingStartUs_) >= 2000000UL) &&
        barometerHealthy && barometerVario_.ready();
    if (useFusion && !fusionActive_) {
        kalman_.seed(barometerAltitudeM_, barometerVario_.climbRateMps(),
                     effectiveBarometerVariance);
        fusionBlend_ = 0.0f;
    } else if (useFusion) {
        fusionBlend_ = fminf(fusionBlend_ + fminf(dt, 0.05f), 1.0f);
    } else {
        fusionBlend_ = 0.0f;
    }
    if (fusionActive_ != useFusion) outputFilter_.reset();
    fusionActive_ = useFusion;

    const float kalmanVario = kalman_.ready() ? kalman_.climbRateMps() : 0.0f;
    bool outputReady = barometerVario_.ready();
    float outputVario = outputReady ? barometerVario_.climbRateMps() : 0.0f;
    if (fusionActive_) outputVario += fusionBlend_ * (kalmanVario - outputVario);
    outputReady = outputReady && barometerHealthy && isfinite(outputVario);
    if (!outputReady) outputVario = 0.0f;
    const float flightDetectionVario = outputVario;
    if (outputReady) {
        outputVario = outputFilter_.update(
            outputVario, dt, VarioTuning::OUTPUT_LPF_TAU_S);
    } else {
        outputFilter_.reset();
    }

    VarioEstimate result;
    result.outputReady = outputReady;
    result.barometerHealthy = barometerHealthy;
    result.imuHealthy = imuHealthy;
    result.fusionActive = fusionActive_;
    result.lastImuSampleOk = lastImuSampleOk;
    result.altitudeM = fusionActive_
        ? barometerAltitudeM_ + fusionBlend_ * (kalman_.altitudeM() - barometerAltitudeM_)
        : barometerAltitudeM_;
    result.barometerAltitudeM = barometerAltitudeM_;
    result.pressurePa = barometerPressureReady_
        ? filteredPressurePa_ : barometer.pressurePa();
    result.outputVarioMps = outputVario;
    result.flightDetectionVarioMps = flightDetectionVario;
    result.barometerVarioMps = barometerVario_.ready()
        ? barometerVario_.climbRateMps() : 0.0f;
    result.kalmanAltitudeM = kalman_.ready()
        ? kalman_.altitudeM() : barometerAltitudeM_;
    result.kalmanVarioMps = kalmanVario;
    result.earthZAccelMps2 = earthZ;
    result.pitchDeg = attitude_.pitchRad() * RAD_TO_DEG;
    result.rollDeg = attitude_.rollRad() * RAD_TO_DEG;
    result.kalmanAccelBias = kalman_.accelBiasMps2();
    result.effectiveBarometerVariance = effectiveBarometerVariance;
    return result;
}
