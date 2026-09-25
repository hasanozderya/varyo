#include "display.h"
#include "config.h"
#include "shared_state.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <string.h>
#include <math.h>

namespace {
    Adafruit_ILI9341 tft(Pins::DISPLAY_CS, Pins::DISPLAY_DC, Pins::DISPLAY_RST);
}

bool Display::begin() {
    pinMode(Pins::DISPLAY_BL, OUTPUT);
    digitalWrite(Pins::DISPLAY_BL, HIGH);
    SPI.begin(Pins::SPI_SCK, Pins::SPI_MISO, Pins::SPI_MOSI, Pins::DISPLAY_CS);
    tft.begin(40000000);
    tft.setRotation(1);
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    available_ = true;
    firstFrame_ = true;
    memset(lastHeader_, 0, sizeof(lastHeader_));
    memset(lastClimb_, 0, sizeof(lastClimb_));
    memset(lastAlt_, 0, sizeof(lastAlt_));
    memset(lastGps_, 0, sizeof(lastGps_));
    memset(lastSpeed_, 0, sizeof(lastSpeed_));
    memset(lastCourse_, 0, sizeof(lastCourse_));
    memset(lastBt_, 0, sizeof(lastBt_));
    memset(lastStatus_, 0, sizeof(lastStatus_));
    return true;
}

void Display::render(const VarioState& s, double /*gpsLat*/, double /*gpsLng*/,
                    double gpsAltM, uint8_t satellites, bool gpsFix) {
    if (!available_) return;

    char buf[96];
    const BleLinkState link = SharedState::readLink();
    const GpsState gps = SharedState::readGps(millis());

    const uint16_t panelDark = ILI9341_DARKGREY;
    const uint16_t panelMid = 0x39E7;
    const uint16_t accent = ILI9341_CYAN;
    const uint16_t accent2 = ILI9341_GREENYELLOW;
    const uint16_t accent3 = ILI9341_YELLOW;
    const uint16_t accent4 = ILI9341_ORANGE;

    if (firstFrame_) {
        tft.fillScreen(ILI9341_BLACK);

        tft.fillRoundRect(8, 8, 304, 32, 8, panelDark);
        tft.drawRoundRect(8, 8, 304, 32, 8, ILI9341_DARKGREY);

        tft.fillRoundRect(10, 48, 300, 118, 10, panelMid);
        tft.drawRoundRect(10, 48, 300, 118, 10, ILI9341_DARKGREY);

        tft.fillRoundRect(10, 178, 142, 26, 8, panelDark);
        tft.drawRoundRect(10, 178, 142, 26, 8, ILI9341_DARKGREY);
        tft.fillRoundRect(168, 178, 142, 26, 8, panelDark);
        tft.drawRoundRect(168, 178, 142, 26, 8, ILI9341_DARKGREY);

        tft.fillRoundRect(10, 212, 142, 26, 8, panelDark);
        tft.drawRoundRect(10, 212, 142, 26, 8, ILI9341_DARKGREY);
        tft.fillRoundRect(168, 212, 142, 26, 8, panelDark);
        tft.drawRoundRect(168, 212, 142, 26, 8, ILI9341_DARKGREY);

        tft.fillRoundRect(10, 240, 300, 30, 8, panelDark);
        tft.drawRoundRect(10, 240, 300, 30, 8, ILI9341_DARKGREY);

        firstFrame_ = false;
    }

    tft.setTextColor(accent, panelDark);
    tft.setTextSize(2);
    snprintf(buf, sizeof(buf), "%s  SAT:%u",
             !s.baroOk ? "SENSOR!" : !s.outputReady ? "WAIT" : s.imuFusionActive ? "FUSION" : "BARO",
             satellites);
    if (strcmp(buf, lastHeader_) != 0) {
        tft.fillRect(16, 15, 280, 18, panelDark);
        tft.setCursor(16, 15);
        tft.print(buf);
        strncpy(lastHeader_, buf, sizeof(lastHeader_) - 1);
        lastHeader_[sizeof(lastHeader_) - 1] = '\0';
    }

    tft.setTextColor(ILI9341_WHITE, panelMid);
    tft.setTextSize(4);
    if (s.outputReady) snprintf(buf, sizeof(buf), "%+.1f", s.climbRateMps);
    else snprintf(buf, sizeof(buf), "--");
    if (strcmp(buf, lastClimb_) != 0) {
        tft.fillRect(22, 74, 180, 42, panelMid);
        tft.setCursor(22, 74);
        tft.print(buf);
        strncpy(lastClimb_, buf, sizeof(lastClimb_) - 1);
        lastClimb_[sizeof(lastClimb_) - 1] = '\0';
    }
    tft.setTextColor(ILI9341_LIGHTGREY, panelMid);
    tft.setTextSize(2);
    tft.fillRect(236, 92, 60, 20, panelMid);
    tft.setCursor(236, 92);
    tft.print("m/s");

    tft.setTextColor(accent2, panelDark);
    tft.setTextSize(2);
    if (s.baroOk) snprintf(buf, sizeof(buf), "BARO ALT %.0f", s.altitudeM);
    else snprintf(buf, sizeof(buf), "BARO ALT --");
    if (strcmp(buf, lastAlt_) != 0) {
        tft.fillRect(18, 182, 120, 18, panelDark);
        tft.setCursor(18, 182);
        tft.print(buf);
        strncpy(lastAlt_, buf, sizeof(lastAlt_) - 1);
        lastAlt_[sizeof(lastAlt_) - 1] = '\0';
    }

    tft.setTextColor(accent3, panelDark);
    if (gpsFix) snprintf(buf, sizeof(buf), "GPS ALT %.0f", gpsAltM);
    else snprintf(buf, sizeof(buf), "GPS ALT --");
    if (strcmp(buf, lastGps_) != 0) {
        tft.fillRect(176, 182, 120, 18, panelDark);
        tft.setCursor(176, 182);
        tft.print(buf);
        strncpy(lastGps_, buf, sizeof(lastGps_) - 1);
        lastGps_[sizeof(lastGps_) - 1] = '\0';
    }

    tft.setTextColor(ILI9341_LIGHTGREY, panelDark);
    if (gpsFix && isfinite(gps.speedMps)) snprintf(buf, sizeof(buf), "SPEED %.1f", gps.speedMps);
    else snprintf(buf, sizeof(buf), "SPEED --");
    if (strcmp(buf, lastSpeed_) != 0) {
        tft.fillRect(18, 216, 120, 18, panelDark);
        tft.setCursor(18, 216);
        tft.print(buf);
        strncpy(lastSpeed_, buf, sizeof(lastSpeed_) - 1);
        lastSpeed_[sizeof(lastSpeed_) - 1] = '\0';
    }

    if (gpsFix && isfinite(gps.courseDeg)) snprintf(buf, sizeof(buf), "COURSE %.0f", gps.courseDeg);
    else snprintf(buf, sizeof(buf), "COURSE --");
    if (strcmp(buf, lastCourse_) != 0) {
        tft.fillRect(176, 216, 120, 18, panelDark);
        tft.setCursor(176, 216);
        tft.print(buf);
        strncpy(lastCourse_, buf, sizeof(lastCourse_) - 1);
        lastCourse_[sizeof(lastCourse_) - 1] = '\0';
    }

    tft.setTextColor(accent4, panelDark);
    tft.setTextSize(2);
    if (link.connected) snprintf(buf, sizeof(buf), "BT ON");
    else snprintf(buf, sizeof(buf), "BT OFF");
    if (strcmp(buf, lastBt_) != 0) {
        tft.fillRect(20, 247, 120, 22, panelDark);
        tft.setCursor(20, 247);
        tft.print(buf);
        strncpy(lastBt_, buf, sizeof(lastBt_) - 1);
        lastBt_[sizeof(lastBt_) - 1] = '\0';
    }

    tft.setTextColor(ILI9341_LIGHTGREY, panelDark);
    tft.setTextSize(2);
    snprintf(buf, sizeof(buf), "P:%0.0f R:%0.0f  MOD:%u  FIX:%s",
             isfinite(s.pitchDeg) ? s.pitchDeg : 0.0f,
             isfinite(s.rollDeg) ? s.rollDeg : 0.0f,
             (unsigned)s.flightMode,
             gpsFix ? "YES" : "NO");
    if (strcmp(buf, lastStatus_) != 0) {
        tft.fillRect(150, 247, 150, 20, panelDark);
        tft.setCursor(150, 247);
        tft.print(buf);
        strncpy(lastStatus_, buf, sizeof(lastStatus_) - 1);
        lastStatus_[sizeof(lastStatus_) - 1] = '\0';
    }

    nextChunk_ = 0;
}

void Display::service() {
    if (!available_) return;
    nextChunk_ = 0;
}
