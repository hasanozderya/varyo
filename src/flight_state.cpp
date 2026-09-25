#include "flight_state.h"
#include <math.h>
#include <atomic>

namespace { std::atomic<int> request{0}; }
void FlightControl::requestStart() { request = 1; }
void FlightControl::requestStop() { request = -1; }
int FlightControl::takeRequest() { return request.exchange(0); }

bool GroundStability::update(uint32_t nowMs, bool valid, float v,
                             bool gpsValid, float speed) {
    const bool quiet = valid && isfinite(v) && fabsf(v) < 0.2f &&
        haveImu_ && stationary_ && (uint32_t)(nowMs-imuMs_) < 100 &&
        (!gpsValid || (isfinite(speed) && speed >= 0 && speed < 1.5f));
    if (!quiet) quiet_ = false;
    else if (!quiet_) { quiet_ = true; quietSinceMs_ = nowMs; }
    return quiet_ && (uint32_t)(nowMs-quietSinceMs_) >= 8000;
}

void FlightStateMachine::start(uint32_t nowMs) {
    if (mode_ == FlightMode::Flying) return;
    mode_ = FlightMode::Flying; startMs_ = nowMs; durationMs_ = 0;
    ++session_; if (!session_) ++session_;
    moving_ = landing_ = manualStop_ = false;
}
void FlightStateMachine::stop(uint32_t nowMs) {
    if (mode_ != FlightMode::Flying) return;
    durationMs_ = nowMs-startMs_; mode_ = FlightMode::Landed;
    moving_ = landing_ = false; manualStop_ = true;
}
void FlightStateMachine::update(uint32_t nowMs, bool valid, float v,
                               bool ground, bool gpsValid, float speed) {
    if (!valid || !isfinite(v)) { moving_ = landing_ = false; return; }
    gpsValid = gpsValid && isfinite(speed) && speed >= 0;
    if (mode_ == FlightMode::Flying) {
        // Loss of GPS must not look like landing in straight, quiet flight.
        const bool landed = ground && gpsValid && speed < 1.5f;
        if (!landed) landing_ = false;
        else if (!landing_) { landing_ = true; landingSince_ = nowMs; }
        else if ((uint32_t)(nowMs-landingSince_) >= 60000) stop(nowMs);
        return;
    }
    if (ground) {
        manualStop_ = false;
        if (mode_ == FlightMode::Unknown) mode_ = FlightMode::Ground;
    }
    const bool moving = !manualStop_ &&
        ((gpsValid && speed >= 5.0f) || (!gpsValid && fabsf(v) >= 1.0f));
    if (!moving) moving_ = false;
    else if (!moving_) { moving_ = true; movingSince_ = nowMs; }
    else if ((uint32_t)(nowMs-movingSince_) >= (gpsValid ? 3000u : 8000u)) start(nowMs);
}
