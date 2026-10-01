#include "buzzer.h"
#include "config.h"
#include "audio_output.h"
#include "generated/voice_prompts.h"
#include <math.h>

void Buzzer::begin() {
    pendingVoice_ = nullptr;
    pendingVoiceSamples_ = 0;
    AudioOutput::stopClip();
    tone(0, 0);
}
void Buzzer::playEvent(Event event) {
    AudioOutput::stopClip();
    switch (event) {
        case Event::FlightStarted:
            pendingVoice_ = VoicePrompts::FLIGHT_STARTED;
            pendingVoiceSamples_ = VoicePrompts::FLIGHT_STARTED_COUNT;
            break;
        case Event::FlightEnded:
            pendingVoice_ = VoicePrompts::FLIGHT_ENDED;
            pendingVoiceSamples_ = VoicePrompts::FLIGHT_ENDED_COUNT;
            break;
        case Event::GpsAcquired:
            pendingVoice_ = VoicePrompts::GPS_ACQUIRED;
            pendingVoiceSamples_ = VoicePrompts::GPS_ACQUIRED_COUNT;
            break;
        case Event::GpsLost:
            pendingVoice_ = VoicePrompts::GPS_LOST;
            pendingVoiceSamples_ = VoicePrompts::GPS_LOST_COUNT;
            break;
        case Event::AltitudeLimitApproaching:
            pendingVoice_ = VoicePrompts::ALTITUDE_LIMIT_APPROACHING;
            pendingVoiceSamples_ = VoicePrompts::ALTITUDE_LIMIT_APPROACHING_COUNT;
            break;
    }
}
void Buzzer::tone(uint32_t frequency, int volume) {
    if (volume == 0) frequency = 0;
    AudioOutput::setTone(frequency, volume);
}
void Buzzer::update(float v, const AudioParams& p, bool valid) {
    const uint32_t now = micros(), ms = millis();
    const float h = p.hysteresisMps;
    Mode next = Mode::Quiet;
    if (!valid || !isfinite(v)) next = Mode::Fault;
    else if (p.strongSinkMps < 0 && v < p.strongSinkMps + (mode_ == Mode::Alarm ? h : 0)) next = Mode::Alarm;
    else if (v < p.sinkAlarmMps + (mode_ == Mode::Sink ? h : 0)) next = Mode::Sink;
    else if (v > fmaxf(0, p.climbDeadbandMps - (mode_ == Mode::Climb ? h : 0))) next = Mode::Climb;
    else if (p.weakLift && v > -0.3f) next = Mode::Weak;
    if (next != mode_) {
        mode_ = next; modeSinceMs_ = ms; toneOn_ = false; nextEventUs_ = now;
        if (mode_ == Mode::Fault || mode_ == Mode::Alarm) {
            pendingVoice_ = nullptr;
            pendingVoiceSamples_ = 0;
            AudioOutput::stopClip();
        }
    }
    if (mode_ == Mode::Fault) {
        if (pendingVoice_) {
            pendingVoice_ = nullptr;
            pendingVoiceSamples_ = 0;
            AudioOutput::stopClip();
        }
        // A short double chirp after 3 seconds, then every 10 seconds.
        const uint32_t elapsed = ms - modeSinceMs_;
        const uint32_t phase = elapsed >= 3000 ? (elapsed-3000)%10000 : 1000;
        tone(phase < 120 ? 900 : phase >= 240 && phase < 360 ? 450 : 0, p.volume);
        return;
    }
    if (mode_ == Mode::Alarm) {
        if (pendingVoice_) {
            pendingVoice_ = nullptr;
            pendingVoiceSamples_ = 0;
            AudioOutput::stopClip();
        }
        tone(((ms-modeSinceMs_)/200)%2 ? 450 : 850, p.volume);
        return;
    }
    if (pendingVoice_) {
        AudioOutput::playClip(pendingVoice_, pendingVoiceSamples_, p.volume);
        pendingVoice_ = nullptr;
        pendingVoiceSamples_ = 0;
    }
    if (mode_ == Mode::Quiet) { tone(0, p.volume); return; }
    if (mode_ == Mode::Sink) {
        const float t = constrain((p.sinkAlarmMps-v)/fmaxf(1.0f, fabsf(p.sinkAlarmMps)), 0.0f, 1.0f);
        const int minimumSinkHz = min(p.sinkToneHz, 350);
        tone((uint32_t)(p.sinkToneHz-t*(p.sinkToneHz-minimumSinkHz)), p.volume);
        return;
    }
    const float t = constrain((v-p.climbDeadbandMps)/(p.climbMaxMps-p.climbDeadbandMps), 0.0f, 1.0f);
    const uint32_t frequency = mode_ == Mode::Weak ? p.toneMinHz : (uint32_t)(p.toneMinHz+t*(p.toneMaxHz-p.toneMinHz));
    const uint32_t periodUs = mode_ == Mode::Weak ? 1200000 : (uint32_t)(p.periodMaxMs-t*(p.periodMaxMs-p.periodMinMs))*1000;
    const uint32_t onUs = mode_ == Mode::Weak ? 65000 : (uint32_t)(periodUs*0.45f);
    if ((int32_t)(now-nextEventUs_) >= 0) {
        toneOn_ = !toneOn_;
        nextEventUs_ = now + (toneOn_ ? onUs : periodUs-onUs);
    }
    tone(toneOn_ ? frequency : 0, p.volume);
}
