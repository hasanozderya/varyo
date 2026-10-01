#include "vario_task.h"

#include "boot_sync.h"
#include "config.h"
#include "debug_log.h"
#include "shared_state.h"
#include "trend_buffer.h"
#include "tunables.h"
#include "vario/flight_session_controller.h"
#include "vario/sensor_supervisor.h"
#include "vario/vario_estimator.h"
#include <atomic>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

namespace {
std::atomic<bool> groundCalibrationRequested{false};

void logStartupDiagnostics(SensorSupervisor& sensors) {
    Barometer& barometer = sensors.barometer();
    Imu& imu = sensors.imu();
    DebugLog::log("\n=== Vario Task baslangic teshisi ===\n");
    const auto diagnostics = barometer.diagnostics();
#if VARIO_BAROMETER_TYPE == BAROMETER_MS5607 || \
    VARIO_BAROMETER_TYPE == BAROMETER_MS5611
    DebugLog::logf(
        "%s: begin()=%s  PROM_okundu=%s  CRC=%s (beklenen=0x%X hesaplanan=0x%X)\n",
        Barometer::modelName(), sensors.barometerOk() ? "OK" : "HATA",
        diagnostics.promReadOk ? "evet" : "hayir (cip yanit vermiyor)",
        diagnostics.crcMatch ? "eslesti" : "ESLESMEDI",
        diagnostics.crcExpected, diagnostics.crcCalculated);
    DebugLog::logf("  Katsayilar: C1=%u C2=%u C3=%u C4=%u C5=%u C6=%u\n",
                   diagnostics.coeff[1], diagnostics.coeff[2], diagnostics.coeff[3],
                   diagnostics.coeff[4], diagnostics.coeff[5], diagnostics.coeff[6]);
#else
    DebugLog::logf("%s: begin()=%s  chipId=0x%02X  kalibrasyon=%s\n",
                   Barometer::modelName(), sensors.barometerOk() ? "OK" : "HATA",
                   diagnostics.chipId,
                   diagnostics.calibrationValid ? "OK" : "HATA");
#endif
    if (!sensors.barometerOk()) {
        DebugLog::log("  -> baro basarisiz oldugu icin vario hazir olmayacak.\n");
    }

    DebugLog::logf("%s: begin()=%s  kayitli_yer_kalibrasyonu=%s\n",
                   Imu::modelName(), sensors.imuOk() ? "OK" : "HATA",
                   imu.calibrated() ? "VAR" : "YOK -> baro vario");
    DebugLog::logf("Aktif cikis: %s\n",
                   VarioTuning::USE_BARO_REGRESSION_OUTPUT
                       ? "BARO linear-regression (IMU+Kalman paralel teshis)"
                       : "IMU+Kalman");
    const Tunables active = TunablesStore::read();
    DebugLog::logf("Fusion: alpha=%.4f accelVar=%.6f biasVar=%.8f baroVar=%.6f adapt=%.4f\n",
                   active.fusion.compFilterAlpha, active.fusion.kfAccelVar,
                   active.fusion.kfAccelBiasVar, active.fusion.kfBaroVar,
                   active.fusion.kAdaptFactor);
    DebugLog::logf("Filter: pressureTau=%.3fs outputTau=%.3fs regression=%lums\n",
                   VarioTuning::BARO_PRESSURE_LPF_TAU_S,
                   VarioTuning::OUTPUT_LPF_TAU_S,
                   (unsigned long)VarioTuning::BARO_REGRESSION_WINDOW_MS);
    DebugLog::logf("Diag buzzer mute: %s\n",
                   VarioDebug::MUTE_BUZZER_DURING_DIAG ? "EVET" : "HAYIR");
    DebugLog::log("=====================================\n\n");
}
} // namespace

bool requestGroundImuCalibration() {
    return !groundCalibrationRequested.exchange(true);
}

void varioTaskFunc(void* /*pvParameters*/) {
    vTaskDelay(pdMS_TO_TICKS(500));

    static SensorSupervisor sensors;
    static VarioEstimator estimator;
    static FlightSessionController flightController;

    sensors.begin();
    flightController.begin();
    logStartupDiagnostics(sensors);

    const bool trendOk = TrendBuffer::init();
    DebugLog::logf("Trend buffer: %s, capacity=%u, memory=%s\n",
                   trendOk ? "OK" : "HATA", (unsigned)TrendBuffer::capacity(),
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
    uint32_t sampleSequence = 0;
    uint32_t maxTickUs = 0;
    uint32_t longTickCount = 0;
    uint32_t lastTrendSampleMs = 0;
    uint32_t lastCsvUs = 0;

    for (;;) {
        const uint32_t nowUs = micros();
        const float dt = (nowUs - lastMicros) * 1.0e-6f;
        lastMicros = nowUs;
        if (dt > 0.015f) ++longTickCount;
        maxTickUs = max(maxTickUs, (uint32_t)(dt * 1000000.0f));
        ESP_ERROR_CHECK(esp_task_wdt_reset());

        const uint32_t nowMs = millis();
        sensors.service(nowUs, nowMs,
                        estimator.barometerReady(), estimator.lastBarometerUs(),
                        estimator.haveImuSample(), estimator.lastImuUs());

        Tunables tunables = TunablesStore::read();
        Imu& imu = sensors.imu();
        if (groundCalibrationRequested.exchange(false)) {
            const VarioState previous = SharedState::readFresh(nowMs);
            if (sensors.imuOk() && !imu.calibrating() &&
                flightController.mode() != FlightMode::Flying && previous.groundStable) {
                imu.startGroundCalibration();
                estimator.calibrationStarted();
                DebugLog::log(">> IMU ground calibration started (3 seconds)\n");
            } else if (!imu.calibrating()) {
                imu.rejectCalibration();
            }
        }

        Barometer& barometer = sensors.barometer();
        const VarioEstimate estimate = estimator.update(
            barometer, imu, sensors.barometerOk(), sensors.imuOk(),
            nowUs, dt, tunables);

        const uint32_t publishMs = millis();
        const GpsState gps = SharedState::readGps(publishMs);
        bool groundStable = estimator.updateGroundStability(
            publishMs, estimate.outputReady && estimate.imuHealthy,
            estimate.outputVarioMps, gps);
        const FlightSessionSnapshot flight = flightController.update(
            publishMs, estimate.outputReady, estimate.flightDetectionVarioMps,
            groundStable, gps, estimate.altitudeM,
            estimate.outputVarioMps, tunables);
        if (flight.mode == FlightMode::Flying) groundStable = false;

        VarioState state;
        state.outputReady = estimate.outputReady;
        state.groundStable = groundStable;
        state.flightMode = (uint8_t)flight.mode;
        state.flightSession = flight.session;
        state.flightDurationMs = flight.durationMs;
        state.updatedMs = publishMs;
        state.sampleSequence = ++sampleSequence;
        if (sampleSequence == 0) state.sampleSequence = ++sampleSequence;
        state.baroSampleMs = estimator.lastBarometerUs() == 0 ? 0 :
            publishMs - (uint32_t)(micros() - estimator.lastBarometerUs()) / 1000;
        state.imuSampleMs = estimator.lastImuUs() == 0 ? 0 :
            publishMs - (uint32_t)(micros() - estimator.lastImuUs()) / 1000;
        state.longTicks = longTickCount;
        state.maxTickUs = maxTickUs;
        state.baroSamples = estimator.barometerSamples();
        state.imuSamples = estimator.imuSamples();
        state.sensorRecoveries = sensors.recoveries();
        state.referenceRevision = estimator.referenceRevision();
        state.stackFreeBytes = uxTaskGetStackHighWaterMark(nullptr);
        state.altitudeM = estimate.altitudeM;
        state.climbRateMps = estimate.outputVarioMps;
        state.earthZAccelMps2 = estimate.earthZAccelMps2;
        state.baroAltitudeM = estimate.barometerAltitudeM;
        state.pressurePa = estimate.pressurePa;
        state.kalmanAccelBias = estimate.kalmanAccelBias;
        state.pitchDeg = estimate.pitchDeg;
        state.rollDeg = estimate.rollDeg;
        state.baroOk = estimate.barometerHealthy;
        state.imuOk = estimate.imuHealthy;
        state.imuCalibrated = imu.calibrated();
        state.imuFusionActive = estimate.fusionActive;
        state.imuCalibrationStatus = (uint8_t)imu.calibrationStatus();
        SharedState::write(state);

        if (estimate.outputReady &&
            (uint32_t)(publishMs - lastTrendSampleMs) >= 1000UL) {
            lastTrendSampleMs = publishMs;
            TrendBuffer::add(publishMs, state.altitudeM,
                             state.climbRateMps, state.pressurePa);
        }

        if (VarioDebug::SERIAL_CSV) {
            const uint32_t csvPeriodUs = 1000000UL / VarioDebug::SERIAL_CSV_HZ;
            if ((uint32_t)(nowUs - lastCsvUs) >= csvPeriodUs) {
                lastCsvUs = nowUs;
                char line[256];
                const int length = snprintf(
                    line, sizeof(line),
                    "@VARIO,%lu,%.6f,%lu,%.1f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.2f,%.2f,%.4f,%lu,%lu,%lu,%.3f,%lu,%lu,%lu,%u,%.5f,%.5f,%.5f,%.5f\n",
                    (unsigned long)nowUs, dt,
                    (unsigned long)estimator.barometerSamples(),
                    barometer.pressurePa(), barometer.temperatureC(),
                    estimate.barometerAltitudeM, estimate.barometerVarioMps,
                    estimate.kalmanAltitudeM, estimate.kalmanVarioMps,
                    estimate.outputVarioMps, estimate.earthZAccelMps2,
                    estimate.pitchDeg, estimate.rollDeg,
                    estimate.kalmanAccelBias,
                    (unsigned long)barometer.rawD1(),
                    (unsigned long)barometer.rawD2(),
                    (unsigned long)barometer.filteredD2(),
                    estimate.effectiveBarometerVariance,
                    (unsigned long)imu.runtimeReadMisses(),
                    (unsigned long)barometer.runtimeBusMisses(),
                    (unsigned long)longTickCount,
                    estimate.lastImuSampleOk ? 1u : 0u,
                    estimator.lastAxG(), estimator.lastAyG(), estimator.lastAzG(),
                    imu.accelBiasG());
                if (length > 0 && length < (int)sizeof(line) && Serial &&
                    Serial.availableForWrite() >= length) {
                    Serial.write((const uint8_t*)line, (size_t)length);
                }
            }
        }

        if ((TickType_t)(xTaskGetTickCount() - lastWake) >= period) {
            lastWake = xTaskGetTickCount();
        }
        vTaskDelayUntil(&lastWake, period);
    }
}
