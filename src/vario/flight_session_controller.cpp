#include "flight_session_controller.h"

#include "config.h"

void FlightSessionController::begin() {
    buzzer_.begin();
    audioFlightMode_ = flight_.mode();
}

FlightSessionSnapshot FlightSessionController::update(
    uint32_t nowMs, bool outputReady, float flightDetectionVario,
    bool groundStable, const GpsState& gps, float altitudeM,
    float outputVario, const Tunables& tunables) {
    const int flightRequest = FlightControl::takeRequest();
    if (flightRequest > 0 && outputReady) flight_.start(nowMs);
    if (flightRequest < 0) flight_.stop(nowMs);
    flight_.update(nowMs, outputReady, flightDetectionVario, groundStable,
                   gps.motionValid, gps.speedMps);

    const FlightMode currentMode = flight_.mode();
    bool flightVoiceTriggered = false;
    if (currentMode != audioFlightMode_) {
        if (currentMode == FlightMode::Flying) {
            flightVoiceTriggered = true;
            if (!VarioDebug::MUTE_BUZZER_DURING_DIAG) {
                buzzer_.playEvent(Buzzer::Event::FlightStarted);
            }
        } else if (audioFlightMode_ == FlightMode::Flying &&
                   currentMode == FlightMode::Landed) {
            flightVoiceTriggered = true;
            if (!VarioDebug::MUTE_BUZZER_DURING_DIAG) {
                buzzer_.playEvent(Buzzer::Event::FlightEnded);
            }
        }
        audioFlightMode_ = currentMode;
    }

    const GpsAudioState::Event gpsEvent = gpsAudio_.update(nowMs, gps.fix);
    bool notificationPlayed = flightVoiceTriggered;
    if (!notificationPlayed && !VarioDebug::MUTE_BUZZER_DURING_DIAG) {
        if (gpsEvent == GpsAudioState::Event::Acquired) {
            buzzer_.playEvent(Buzzer::Event::GpsAcquired);
            notificationPlayed = true;
        } else if (gpsEvent == GpsAudioState::Event::Lost) {
            buzzer_.playEvent(Buzzer::Event::GpsLost);
            notificationPlayed = true;
        }
    }
    if (!notificationPlayed &&
        altitudeAlert_.update(tunables.audio.altitudeAlertEnabled,
                              currentMode == FlightMode::Flying, outputReady,
                              altitudeM, tunables.audio.altitudeLimitM,
                              tunables.audio.altitudeWarningMarginM) &&
        !VarioDebug::MUTE_BUZZER_DURING_DIAG) {
        buzzer_.playEvent(Buzzer::Event::AltitudeLimitApproaching);
    }

    if (VarioDebug::MUTE_BUZZER_DURING_DIAG) {
        buzzer_.update(0.0f, tunables.audio);
    } else {
        buzzer_.update(outputVario, tunables.audio, outputReady);
    }

    return {currentMode, flight_.session(), flight_.durationMs(nowMs)};
}
