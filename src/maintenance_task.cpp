#include "maintenance_task.h"
#include "imus/imu.h"
#include "shared_state.h"
#include "debug_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void maintenanceTaskFunc(void*) {
    uint32_t lastLogMs = 0;
    uint32_t lastBiasWarningMs = 0;
    for (;;) {
        Imu::serviceStorage();
        const uint32_t nowMs = millis();
        if ((uint32_t)(nowMs-lastLogMs) >= 1000) {
            lastLogMs = nowMs;
            const VarioState s = SharedState::readFresh(nowMs);
            DebugLog::logf("[vario] ready=%u fusion=%u ground=%u v=%.2f alt=%.1f az=%.3f long=%lu maxUs=%lu recover=%lu\n",
                s.outputReady, s.imuFusionActive, s.groundStable,
                s.climbRateMps, s.altitudeM,
                s.earthZAccelMps2, (unsigned long)s.longTicks,
                (unsigned long)s.maxTickUs, (unsigned long)s.sensorRecoveries);
            if (s.imuFusionActive && fabsf(s.kalmanAccelBias) >= 0.45f &&
                (lastBiasWarningMs == 0 || (uint32_t)(nowMs-lastBiasWarningMs) >= 10000)) {
                lastBiasWarningMs = nowMs;
                DebugLog::logf("[vario-warning] accel bias near limit: %.3f m/s2; inspect calibration, mounting and vibration\n",
                               s.kalmanAccelBias);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
