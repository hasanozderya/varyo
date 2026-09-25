#include "maintenance_task.h"
#include "mpu6050.h"
#include "shared_state.h"
#include "debug_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void maintenanceTaskFunc(void*) {
    uint32_t lastLogMs = 0;
    for (;;) {
        MPU6050::serviceStorage();
        const uint32_t nowMs = millis();
        if ((uint32_t)(nowMs-lastLogMs) >= 1000) {
            lastLogMs = nowMs;
            const VarioState s = SharedState::readFresh(nowMs);
            DebugLog::logf("[vario] ready=%u fusion=%u ground=%u v=%.2f alt=%.1f az=%.3f long=%lu maxUs=%lu recover=%lu\n",
                s.outputReady, s.imuFusionActive, s.groundStable,
                s.climbRateMps, s.altitudeM,
                s.earthZAccelMps2, (unsigned long)s.longTicks,
                (unsigned long)s.maxTickUs, (unsigned long)s.sensorRecoveries);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
