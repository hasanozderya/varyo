#pragma once

#include "altitude_alert.h"
#include "buzzer.h"
#include "flight_state.h"
#include "gps_audio_state.h"
#include "shared_state.h"
#include "tunables.h"

struct FlightSessionSnapshot {
    FlightMode mode = FlightMode::Unknown;
    uint32_t session = 0;
    uint32_t durationMs = 0;
};

// Keeps flight transitions, spoken notifications and vario tone arbitration
// together. It does not read sensors or publish global state.
class FlightSessionController {
public:
    void begin();
    FlightSessionSnapshot update(uint32_t nowMs, bool outputReady,
                                 float flightDetectionVario, bool groundStable,
                                 const GpsState& gps, float altitudeM,
                                 float outputVario, const Tunables& tunables);
    FlightMode mode() const { return flight_.mode(); }

private:
    FlightStateMachine flight_;
    AltitudeAlertState altitudeAlert_;
    GpsAudioState gpsAudio_;
    Buzzer buzzer_;
    FlightMode audioFlightMode_ = FlightMode::Unknown;
};
