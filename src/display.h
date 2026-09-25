#pragma once
#include "shared_state.h"
#include <stdint.h>

class Display {
public:
    bool begin();
    void render(const VarioState& state, double gpsLat, double gpsLng,
                double gpsAltM, uint8_t satellites, bool gpsFix);
    void service();
private:
    bool available_ = false;
    uint8_t nextChunk_ = 16;
    bool firstFrame_ = true;
    char lastHeader_[24] = {0};
    char lastClimb_[16] = {0};
    char lastAlt_[20] = {0};
    char lastGps_[20] = {0};
    char lastSpeed_[20] = {0};
    char lastCourse_[20] = {0};
    char lastBt_[12] = {0};
    char lastStatus_[40] = {0};
};
