#include "gps_ui_task.h"
#include "config.h"
#include "shared_state.h"
#include "display.h"
#include "xctrack_protocol.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <time.h>
#include <math.h>

void uiTaskFunc(void* /*pvParameters*/) {
    static HardwareSerial gpsSerial(0); // ESP32-S3 UART0, pins GPIO 43/44
    gpsSerial.begin(Timing::GPS_BAUD, SERIAL_8N1, Pins::GPS_RX, Pins::GPS_TX);
    static TinyGPSPlus gps;

    static Display display;
    display.begin();

    const TickType_t period = pdMS_TO_TICKS(1000 / Timing::UI_TASK_HZ);
    TickType_t lastWake = xTaskGetTickCount();
    const uint32_t redrawEveryNTicks = Timing::UI_TASK_HZ / Timing::DISPLAY_REFRESH_HZ;
    uint32_t tickCount = 0;
    GpsState state;

    for (;;) {
        // Drain everything currently buffered — at 9600 baud a byte
        // arrives roughly every ~1ms, so this must not wait for a full
        // NMEA sentence before yielding back to vTaskDelayUntil.
        while (gpsSerial.available()) {
            const uint8_t byte = (uint8_t)gpsSerial.read();
            gps.encode(byte);
            XcTrackProtocol::GpsStore::feed(byte, millis());
        }

        const uint32_t nowMs = millis();
        if (gps.location.isUpdated()) {
            state.latitude = gps.location.lat(); state.longitude = gps.location.lng();
            state.fixMs = nowMs - gps.location.age();
        }
        state.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 99;
        state.satellites = gps.satellites.isValid() ? min((uint32_t)99, gps.satellites.value()) : 0;
        state.fix = gps.location.isValid() && gps.location.age() < 3000 &&
            gps.hdop.age() < 3000 && gps.satellites.age() < 3000 &&
            state.hdop > 0 && state.hdop <= 5.0f && state.satellites >= 4;
        if (gps.speed.isUpdated()) {
            state.speedMps = gps.speed.mps();
            state.courseDeg = gps.course.isValid() ? gps.course.deg() : 0;
            state.motionMs = nowMs - gps.speed.age();
        }
        state.motionValid = state.fix && gps.speed.isValid() && gps.speed.age() < 3000 &&
            gps.course.isValid() && gps.course.age() < 3000;
        if (gps.altitude.isUpdated()) {
            state.altitudeM = gps.altitude.meters();
            state.altitudeMs = nowMs - gps.altitude.age();
        }
        state.altitudeValid = state.fix && gps.altitude.isValid() && gps.altitude.age() < 3000;
        if (gps.time.isUpdated() && gps.time.isValid() && gps.date.isValid() && gps.date.age() < 3000) {
            tm utc{};
            utc.tm_year = gps.date.year()-1900; utc.tm_mon = gps.date.month()-1;
            utc.tm_mday = gps.date.day(); utc.tm_hour = gps.time.hour();
            utc.tm_min = gps.time.minute(); utc.tm_sec = gps.time.second();
            // ESP32's default timezone is UTC; set explicitly once in setup.
            const time_t epoch = mktime(&utc);
            if (epoch >= 1704067200LL && epoch <= 4102444799LL) {
                state.utcSeconds = (uint32_t)epoch;
                state.utcSampleMs = nowMs-gps.time.age();
                state.utcValid = true;
            }
        }
        SharedState::writeGps(state);
        display.service();
        if (++tickCount >= redrawEveryNTicks) {
            tickCount = 0;
            const VarioState s = SharedState::readFresh(nowMs);
            const GpsState fresh = SharedState::readGps(nowMs);
            display.render(s, fresh.latitude, fresh.longitude,
                fresh.altitudeM, fresh.satellites, fresh.fix && fresh.altitudeValid);
        }

        vTaskDelayUntil(&lastWake, period);
    }
}
