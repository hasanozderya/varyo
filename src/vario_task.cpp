#include "vario_task.h"
#include "config.h"
#include "shared_state.h"
#include "tunables.h"
#include "debug_log.h"
#include "ms5607.h"
#include "mpu6050.h"
#include "sensor_fusion.h"
#include "buzzer.h"
#include "boot_sync.h"
#include "trend_buffer.h"
#include "i2c_bus.h"
#include <esp_task_wdt.h>
#include "flight_state.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <atomic>

namespace {
    std::atomic<bool> groundCalibrationRequested{false};
}

bool requestGroundImuCalibration() {
    return !groundCalibrationRequested.exchange(true);
}

void varioTaskFunc(void* /*pvParameters*/) {

    vTaskDelay(pdMS_TO_TICKS(500));

    static MS5607 baro;
    static MPU6050 imu;
    static AttitudeFilter attitude;
    static AltitudeKalman kalman;
    static BaroVarioRegression baroVario;
    static VarioOutputFilter outputFilter;
    static Buzzer buzzer;
    static FlightStateMachine flight;
    static GroundStability ground;

    bool baroOk = baro.begin();
    bool imuOk  = imu.begin();   // only configure and load previously saved offsets
    buzzer.begin();

    DebugLog::log("\n=== Vario Task baslangic teshisi ===\n");
    {
        MS5607::Diagnostics d = baro.diagnostics();
        DebugLog::logf("MS5607: begin()=%s  PROM_okundu=%s  CRC=%s (beklenen=0x%X hesaplanan=0x%X)\n",
                        baroOk ? "OK" : "HATA",
                        d.promReadOk ? "evet" : "hayir (cip yanit vermiyor)",
                        d.crcMatch ? "eslesti" : "ESLESMEDI",
                        d.crcExpected, d.crcCalculated);
        DebugLog::logf("  Katsayilar: C1=%u C2=%u C3=%u C4=%u C5=%u C6=%u\n",
                        d.coeff[1], d.coeff[2], d.coeff[3], d.coeff[4], d.coeff[5], d.coeff[6]);
        if (!baroOk) {
            DebugLog::log("  -> baro basarisiz oldugu icin vario hazir olmayacak.\n");
        }
    }

    DebugLog::logf("MPU6050: begin()=%s  kayitli_yer_kalibrasyonu=%s\n",
                    imuOk ? "OK" : "HATA", imu.calibrated() ? "VAR" : "YOK -> baro vario");
    DebugLog::logf("Aktif cikis: %s\n",
                    VarioTuning::USE_BARO_REGRESSION_OUTPUT
                        ? "BARO linear-regression (IMU+Kalman paralel teshis)"
                        : "IMU+Kalman");
    DebugLog::logf("Diag buzzer mute: %s\n", VarioDebug::MUTE_BUZZER_DURING_DIAG ? "EVET" : "HAYIR");
    DebugLog::log("=====================================\n\n");

    const bool trendOk = TrendBuffer::init();
    DebugLog::logf("Trend buffer: %s, capacity=%u, memory=%s\n",
                   trendOk ? "OK" : "HATA",
                   (unsigned)TrendBuffer::capacity(),
                   TrendBuffer::usingPsram() ? "PSRAM" : "internal RAM");

    BootSync::markSensorsReady();
    if (esp_task_wdt_status(nullptr) == ESP_ERR_INVALID_STATE) {
        const esp_task_wdt_config_t watchdog{3000, 0, true};
        ESP_ERROR_CHECK(esp_task_wdt_init(&watchdog));
    }
    ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));

    const TickType_t period = pdMS_TO_TICKS(1000 / Timing::VARIO_TASK_HZ);
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t lastMicros = micros();
    uint32_t sampleSequence = 0, imuSampleCount = 0, referenceRevision = 0;
    uint32_t maxTickUs = 0, recoveries = 0;
    uint32_t baroAttemptMs = millis(), imuAttemptMs = millis(), busAttemptMs = millis();
    bool baroRecovering = false, imuRecovering = false;
    uint32_t lastCsvUs = 0;
    uint32_t baroSampleCount = 0;
    uint32_t longTickCount = 0;
    uint32_t lastTrendSampleMs = 0;
    uint32_t lastImuUs = 0;
    bool haveImuSample = false;
    bool imuSettling = false;
    uint32_t imuSettlingStartUs = 0;
    bool fusionActive = false;
    bool imuReferenceUsable = false;
    float fusionBlend = 0.0f;
    constexpr uint32_t BARO_TIMEOUT_US = 500000;
    constexpr uint32_t IMU_TIMEOUT_US = 200000;

    static float baroAltitude = 0.0f;
    static float filteredPressurePa = 0.0f;
    static uint32_t lastBaroFilterUs = 0;
    static bool baroPressureFilterReady = false;
    float effectiveBaroVar = VarioTuning::KF_BARO_VAR_FLOOR;
    static float earthZ = 0.0f;
    static float lastAxG = 0.0f, lastAyG = 0.0f, lastAzG = 0.0f;
    static bool  lastImuSampleOk = false;
    static float lastAppliedQnh = NAN;

    for (;;) {
        const uint32_t now = micros();
        float dt = (now - lastMicros) / 1000000.0f;
        lastMicros = now;
        if (dt > 0.015f) longTickCount++;
        maxTickUs = max(maxTickUs, (uint32_t)(dt*1000000.0f));
        if (dt > 0.2f) { fusionActive = false; imuSettling = false; }
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        const uint32_t recoveryMs = millis();
        if ((uint32_t)(recoveryMs - busAttemptMs) >= 5000) {
            busAttemptMs = recoveryMs;
            if ((!baroOk || !baroPressureFilterReady) && (!imuOk || !haveImuSample)) I2CBus::recover();
        }
        if (!baroRecovering && (uint32_t)(recoveryMs - baroAttemptMs) >= 2000 &&
            (!baroOk || !baroPressureFilterReady || (uint32_t)(now-lastBaroFilterUs) >= 2000000)) {
            baroAttemptMs = recoveryMs; baroRecovering = true; baroOk = false;
            baro.startRecovery();
        }
        if (baroRecovering) {
            if (baro.serviceRecovery()) { baroOk = true; baroRecovering = false; ++recoveries; }
            else if ((uint32_t)(recoveryMs-baroAttemptMs) >= 400) baroRecovering = false;
        }
        if (!imuRecovering && (uint32_t)(recoveryMs - imuAttemptMs) >= 2000 &&
            (!imuOk || !haveImuSample || (uint32_t)(now-lastImuUs) >= 2000000)) {
            imuAttemptMs = recoveryMs; imuRecovering = true; imuOk = false;
            haveImuSample = false; imu.startRecovery();
        }
        if (imuRecovering) {
            if (imu.serviceRecovery()) { imuOk = true; imuRecovering = false; ++recoveries; }
            else if ((uint32_t)(recoveryMs-imuAttemptMs) >= 400) imuRecovering = false;
        }

        Tunables tun = TunablesStore::read();
        if (groundCalibrationRequested.exchange(false)) {
            if (imuOk && !imu.calibrating() && flight.mode() != FlightMode::Flying &&
                SharedState::readFresh(millis()).groundStable) {
                imu.startGroundCalibration();
                imuSettling = false;
                fusionActive = false;
                DebugLog::log(">> IMU ground calibration started (3 seconds)\n");
            } else if (!imu.calibrating()) imu.rejectCalibration();
        }

        effectiveBaroVar = fmaxf(tun.fusion.kfBaroVar, VarioTuning::KF_BARO_VAR_FLOOR);

        // The barometric QNH change is affine in altitude. Transform both
        // estimators, including history/covariance, without inserting v=0.
        if (!isfinite(lastAppliedQnh)) lastAppliedQnh = tun.fusion.qnhHpa;
        else if (fabsf(tun.fusion.qnhHpa-lastAppliedQnh) > 0.001f) {
            const float scale = powf(lastAppliedQnh/tun.fusion.qnhHpa, 0.190295f);
            const float offset = 44330.0f*(1.0f-scale);
            kalman.rebase(scale, offset);
            baroVario.rebase(scale, offset);
            outputFilter.rebase(scale);
            baroAltitude = scale*baroAltitude + offset;
            lastAppliedQnh = tun.fusion.qnhHpa;
            ++referenceRevision;
            TrendBuffer::clear();
        }

        // -------------------------------------------------------------
        // Barometer
        // -------------------------------------------------------------
        // Expire the estimator before accepting a returning sample, so a
        // reconnection starts with zero velocity rather than a stale state.
        if (baroPressureFilterReady && (uint32_t)(now - lastBaroFilterUs) >= BARO_TIMEOUT_US) {
            baroPressureFilterReady = false;
            kalman.reset();
            baroVario.reset();
            fusionActive = false;
        }
        if (baroOk) baro.update();
        if (baro.hasNewSample()) {
            baro.clearNewSample();
            baroSampleCount++;

            const uint32_t sampleUs = baro.sampleTimeUs();
            const float rawPressurePa = baro.pressurePa();

            // Pressure low-pass (exact discrete-time form of a 1st-order RC).
            // Use the real barometer sample interval because OLED/I2C sharing
            // can make the MS5607 sample spacing slightly non-uniform.
            if (!baroPressureFilterReady || !isfinite(filteredPressurePa)) {
                filteredPressurePa = rawPressurePa;
                lastBaroFilterUs = sampleUs;
                baroPressureFilterReady = true;
            } else {
                float baroDt = (sampleUs - lastBaroFilterUs) * 1.0e-6f;
                lastBaroFilterUs = sampleUs;

                // A very long gap means the old state is no longer useful.
                if (baroDt <= 0.0f || baroDt > 1.0f) {
                    filteredPressurePa = rawPressurePa;
                } else {
                    const float tau = VarioTuning::BARO_PRESSURE_LPF_TAU_S;
                    const float alpha = 1.0f - expf(-baroDt / tau);
                    filteredPressurePa += alpha * (rawPressurePa - filteredPressurePa);
                }
            }

            baroAltitude = MS5607::pressureToAltitude(
                filteredPressurePa, tun.fusion.qnhHpa);

            baroVario.addSample(
                baroAltitude,
                sampleUs,
                VarioTuning::BARO_REGRESSION_WINDOW_MS * 1000UL,
                VarioTuning::BARO_REGRESSION_MIN_SAMPLES);

            // Keep the existing IMU+Kalman estimator alive in parallel so
            // the PC diagnostic logger can compare both paths.
            kalman.correct(baroAltitude, effectiveBaroVar);
        }

        // -------------------------------------------------------------
        // IMU / attitude / vertical acceleration
        // -------------------------------------------------------------
        earthZ = 0.0f;
        lastImuSampleOk = false;
        // If the IMU cannot be read because OLED currently owns the shared
        // I2C bus, propagate the Kalman state with zero NET acceleration
        // instead of stalling this real-time task or injecting -bias.
        float accelInput = kalman.accelBiasMps2();
        if (imuOk) {
            float ax, ay, az, gx, gy, gz;
            if (imu.read(ax, ay, az, gx, gy, gz)) {
                const uint32_t imuUs = micros();
                if (!haveImuSample || (uint32_t)(imuUs - lastImuUs) >= IMU_TIMEOUT_US) {
                    attitude = AttitudeFilter{};
                    imuSettling = false;
                    fusionActive = false;
                }
                const float imuDt = haveImuSample ? (imuUs-lastImuUs)*1e-6f : 0.01f;
                lastImuUs = imuUs;
                ++imuSampleCount;
                haveImuSample = true;
                lastImuSampleOk = true;
                lastAxG = ax; lastAyG = ay; lastAzG = az;
                if (imu.gyroSaturated()) attitude = AttitudeFilter{};
                else attitude.update(
                    ax, ay, az,
                    gx, gy, gz,
                    imuDt,
                    tun.fusion.compFilterAlpha
                );

                const float norm = sqrtf(ax*ax + ay*ay + az*az);
                const float gyroRate = sqrtf(gx*gx + gy*gy + gz*gz);
                // A fast hand shake can clip the ADC or otherwise produce a
                // multi-g reading that is not a real, integrable vertical
                // acceleration. Feeding that straight into the Kalman
                // predict step used to push accelBias_ toward its limit and
                // leave it stuck there (small process noise -> slow to
                // recover). Coast on the current bias estimate instead of
                // trusting this one sample; the attitude filter's own
                // tilt-correction already down-weights it separately.
                const bool verticalReferenceUsable = !imu.accelSaturated() && !imu.gyroSaturated() &&
                    isfinite(norm) && norm >= 0.2f && norm < 3.0f &&
                    attitude.verticalReferenceReliable();
                imuReferenceUsable = verticalReferenceUsable;
                if (verticalReferenceUsable) {
                    earthZ = attitude.earthZAccelMps2(
                        ax, ay, az,
                        imu.accelBiasG()
                    );
                    accelInput = earthZ;
                } else {
                    earthZ = 0.0f;
                    accelInput = kalman.accelBiasMps2();
                }
                ground.observeImu(millis(), verticalReferenceUsable &&
                    fabsf(norm-1.0f) < 0.025f && gyroRate < 3.0f*DEG_TO_RAD);
            }
        }

        I2CBus::sensorCycleComplete();

        kalman.predict(
            accelInput,
            dt,
            tun.fusion.kfAccelVar,
            tun.fusion.kfAccelBiasVar,
            tun.fusion.kAdaptFactor
        );

        const uint32_t healthUs = micros();
        const bool baroHealthy = baroOk && baroPressureFilterReady &&
            (uint32_t)(healthUs - lastBaroFilterUs) < BARO_TIMEOUT_US;
        const bool imuHealthy = imuOk && haveImuSample &&
            (uint32_t)(healthUs - lastImuUs) < IMU_TIMEOUT_US;
        const bool wasCalibrating = imu.calibrating();
        imu.serviceGroundCalibration();
        if (wasCalibrating && !imu.calibrating()) {
            attitude = AttitudeFilter{};
            imuSettling = false;
            fusionActive = false;
            DebugLog::logf(">> IMU yer kalibrasyonu sonucu=%u, ornek=%d\n",
                (unsigned)imu.calibrationStatus(), imu.calibrationSamples());
        }

        // Never infer a zero-rate offset from motion during boot. A stored
        // calibration enables fusion only after the attitude has had time
        // to settle. Until then, actual pressure change supplies the vario.
        const bool imuUsable = imuHealthy && imuReferenceUsable && imu.calibrated() &&
            imu.calibrationTemperatureOk() && !imu.calibrating();
        if (!imuUsable) imuSettling = false;
        else if (!imuSettling) {
            imuSettling = true;
            imuSettlingStartUs = healthUs;
        }
        const bool useFusion = !VarioTuning::USE_BARO_REGRESSION_OUTPUT && imuUsable &&
            imuSettling && (fusionActive || (uint32_t)(healthUs - imuSettlingStartUs) >= 2000000UL) &&
            baroHealthy && baroVario.ready();
        if (useFusion && !fusionActive) {
            // Opening while already climbing/sinking must not start at v=0.
            kalman.seed(baroAltitude, baroVario.climbRateMps(), effectiveBaroVar);
            fusionBlend = 0.0f;
        }
        else if (useFusion) fusionBlend = fminf(fusionBlend + fminf(dt, 0.05f), 1.0f);
        else fusionBlend = 0.0f;
        if (fusionActive != useFusion) outputFilter.reset();
        fusionActive = useFusion;
        const float kalmanVario = kalman.ready() ? kalman.climbRateMps() : 0.0f;
        bool outputReady = baroVario.ready();
        float outputVario = outputReady ? baroVario.climbRateMps() : 0.0f;
        if (fusionActive)
            outputVario += fusionBlend * (kalmanVario - outputVario);

        // -------------------------------------------------------------
        // Publish / audio
        // -------------------------------------------------------------
        outputReady = outputReady && baroHealthy && isfinite(outputVario);
        if (!outputReady) outputVario = 0.0f;
        // Preserve the estimator value for flight-state transitions, but use
        // the same low-pass result seen by BLE/display/audio when deciding
        // whether the unit has been quiet long enough for ground calibration.
        // This prevents short MS5607 pressure-noise cycles from repeatedly
        // resetting the eight-second timer. A sustained climb/sink still
        // passes through the LPF and therefore remains ineligible.
        const float flightDetectionVario = outputVario;
        if (outputReady) outputVario = outputFilter.update(outputVario, dt,
            VarioTuning::OUTPUT_LPF_TAU_S);
        else outputFilter.reset();
        const uint32_t publishMs = millis();
        const GpsState gps = SharedState::readGps(publishMs);
        VarioState s;
        s.outputReady = outputReady;
        s.groundStable = ground.update(publishMs, outputReady && imuHealthy,
            outputVario, gps.motionValid, gps.speedMps);
        const int flightRequest = FlightControl::takeRequest();
        if (flightRequest > 0 && outputReady) flight.start(publishMs);
        if (flightRequest < 0) flight.stop(publishMs);
        flight.update(publishMs, outputReady, flightDetectionVario,
            s.groundStable, gps.motionValid, gps.speedMps);
        s.flightMode = (uint8_t)flight.mode();
        s.flightSession = flight.session();
        s.flightDurationMs = flight.durationMs(publishMs);
        if (flight.mode() == FlightMode::Flying) s.groundStable = false;
        s.updatedMs = publishMs;
        s.sampleSequence = ++sampleSequence;
        if (sampleSequence == 0) s.sampleSequence = ++sampleSequence;
        // Ages are converted relative to the same now; micros wraps much sooner than millis.
        s.baroSampleMs = publishMs - (uint32_t)(micros()-lastBaroFilterUs)/1000;
        s.imuSampleMs = publishMs - (uint32_t)(micros()-lastImuUs)/1000;
        s.longTicks = longTickCount;
        s.maxTickUs = maxTickUs;
        s.baroSamples = baroSampleCount;
        s.imuSamples = imuSampleCount;
        s.sensorRecoveries = recoveries;
        s.referenceRevision = referenceRevision;
        s.stackFreeBytes = uxTaskGetStackHighWaterMark(nullptr);
        s.altitudeM       = fusionActive
            ? baroAltitude + fusionBlend * (kalman.altitudeM() - baroAltitude) : baroAltitude;
        s.climbRateMps    = outputVario;
        s.earthZAccelMps2 = earthZ;
        s.baroAltitudeM   = baroAltitude;
        s.pressurePa      = baroPressureFilterReady ? filteredPressurePa : baro.pressurePa();
        s.kalmanAccelBias = kalman.accelBiasMps2();
        s.pitchDeg        = attitude.pitchRad() * RAD_TO_DEG;
        s.rollDeg         = attitude.rollRad() * RAD_TO_DEG;
        s.baroOk          = baroHealthy;
        s.imuOk           = imuHealthy;
        s.imuCalibrated   = imu.calibrated();
        s.imuFusionActive = fusionActive;
        s.imuCalibrationStatus = (uint8_t)imu.calibrationStatus();
        SharedState::write(s);

        // 1 Hz trend history, stored only in volatile RAM/PSRAM.
        const uint32_t trendNowMs = millis();
        if (outputReady && (uint32_t)(trendNowMs - lastTrendSampleMs) >= 1000UL) {
            lastTrendSampleMs = trendNowMs;
            TrendBuffer::add(trendNowMs, s.altitudeM, s.climbRateMps, s.pressurePa);
        }

        if (VarioDebug::MUTE_BUZZER_DURING_DIAG) {
            buzzer.update(0.0f, tun.audio);
        } else {
            buzzer.update(outputVario, tun.audio, outputReady);
        }

        // -------------------------------------------------------------
        // Non-blocking serial diagnostic stream.
        // Ordinary log lines may share the port; the PC tool only parses
        // records beginning with "@VARIO,".
        //
        // @VARIO:
        // t_us,dt_s,baro_count,P_pa,temp_C,baro_alt,baro_vario,
        // kalman_alt,kalman_vario,output_vario,earthZ,pitch,roll,kf_bias,
        // rawD1,rawD2,filtD2,effective_baro_var,imu_read_miss,baro_bus_miss,long_tick_count,
        // imu_ok,ax_g,ay_g,az_g,accel_bias_g
        // -------------------------------------------------------------
        if (VarioDebug::SERIAL_CSV) {
            const uint32_t csvPeriodUs = 1000000UL / VarioDebug::SERIAL_CSV_HZ;
            if ((uint32_t)(now - lastCsvUs) >= csvPeriodUs) {
                lastCsvUs = now;

                char line[256];
                const int n = snprintf(
                    line, sizeof(line),
                    "@VARIO,%lu,%.6f,%lu,%.1f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.2f,%.2f,%.4f,%lu,%lu,%lu,%.3f,%lu,%lu,%lu,%u,%.5f,%.5f,%.5f,%.5f\n",
                    (unsigned long)now,
                    dt,
                    (unsigned long)baroSampleCount,
                    baro.pressurePa(),
                    baro.temperatureC(),
                    baroAltitude,
                    baroVario.ready() ? baroVario.climbRateMps() : 0.0f,
                    kalman.ready() ? kalman.altitudeM() : baroAltitude,
                    kalmanVario,
                    outputVario,
                    earthZ,
                    s.pitchDeg,
                    s.rollDeg,
                    kalman.accelBiasMps2(),
                    (unsigned long)baro.rawD1(),
                    (unsigned long)baro.rawD2(),
                    (unsigned long)baro.filteredD2(),
                    effectiveBaroVar,
                    (unsigned long)imu.runtimeReadMisses(),
                    (unsigned long)baro.runtimeBusMisses(),
                    (unsigned long)longTickCount,
                    lastImuSampleOk ? 1u : 0u,
                    lastAxG, lastAyG, lastAzG, imu.accelBiasG()
                );

                if (n > 0 && n < (int)sizeof(line) &&
                    Serial && Serial.availableForWrite() >= n) {
                    Serial.write((const uint8_t*)line, (size_t)n);
                }
            }
        }

        // Human-readable diagnostics are formatted by the BLE/UI core.
        // Recover the schedule after an overrun instead of burst-reading sensors.
        if ((TickType_t)(xTaskGetTickCount()-lastWake) >= period) lastWake = xTaskGetTickCount();
        vTaskDelayUntil(&lastWake, period);
    }
}
