#pragma once
#include <Arduino.h>
#include "tunables.h"
class Buzzer {
public:
    enum class Event { FlightStarted, FlightEnded, GpsAcquired, GpsLost, AltitudeLimitApproaching };
    void begin();
    void playEvent(Event event);
    void update(float climbRateMps, const AudioParams& params, bool valid = true);
private:
    enum class Mode { Quiet, Climb, Sink, Alarm, Weak, Fault };
    void tone(uint32_t frequency, int volume);
    Mode mode_ = Mode::Quiet;
    uint32_t modeSinceMs_ = 0, nextEventUs_ = 0;
    const int16_t* pendingVoice_ = nullptr;
    size_t pendingVoiceSamples_ = 0;
    bool toneOn_ = false;
};
