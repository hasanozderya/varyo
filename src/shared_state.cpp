#include "shared_state.h"

namespace {
    portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
    VarioState   s_state;
    GpsState s_gps;
    BleLinkState s_link;
}

void SharedState::init() {
    portENTER_CRITICAL(&s_mux);
    s_state = VarioState{};
    s_gps = GpsState{};
    s_link = BleLinkState{};
    portEXIT_CRITICAL(&s_mux);
}

void SharedState::writeLink(const BleLinkState& link) {
    portENTER_CRITICAL(&s_mux);
    s_link = link;
    portEXIT_CRITICAL(&s_mux);
}

BleLinkState SharedState::readLink() {
    portENTER_CRITICAL(&s_mux);
    const BleLinkState link = s_link;
    portEXIT_CRITICAL(&s_mux);
    return link;
}

VarioState SharedState::readFresh(uint32_t nowMs) {
    VarioState s = read();
    if (!s.sampleSequence || (uint32_t)(nowMs - s.updatedMs) >= 500) {
        s.outputReady = s.baroOk = s.imuOk = s.imuFusionActive = s.groundStable = false;
    }
    if ((uint32_t)(nowMs - s.baroSampleMs) >= 500) s.outputReady = s.baroOk = false;
    if ((uint32_t)(nowMs - s.imuSampleMs) >= 200) s.imuOk = s.imuFusionActive = s.groundStable = false;
    return s;
}

void SharedState::writeGps(const GpsState& gps) {
    portENTER_CRITICAL(&s_mux);
    s_gps = gps;
    portEXIT_CRITICAL(&s_mux);
}

GpsState SharedState::readGps(uint32_t nowMs) {
    GpsState gps;
    portENTER_CRITICAL(&s_mux);
    gps = s_gps;
    portEXIT_CRITICAL(&s_mux);
    gps.fix = gps.fix && (uint32_t)(nowMs - gps.fixMs) < 3000;
    gps.motionValid = gps.fix && gps.motionValid && (uint32_t)(nowMs - gps.motionMs) < 3000;
    gps.altitudeValid = gps.fix && gps.altitudeValid && (uint32_t)(nowMs - gps.altitudeMs) < 3000;
    gps.utcValid = gps.utcValid && (uint32_t)(nowMs - gps.utcSampleMs) < 3000;
    return gps;
}

void SharedState::write(const VarioState& s) {
    portENTER_CRITICAL(&s_mux);
    s_state = s;
    portEXIT_CRITICAL(&s_mux);
}

VarioState SharedState::read() {
    VarioState copy;
    portENTER_CRITICAL(&s_mux);
    copy = s_state;
    portEXIT_CRITICAL(&s_mux);
    return copy;
}
