#pragma once
#include <stdint.h>

class GpsAudioState {
public:
    enum class Event { None, Acquired, Lost };

    Event update(uint32_t nowMs, bool fix) {
        if (fix != candidateFix_) {
            candidateFix_ = fix;
            candidateSinceMs_ = nowMs;
        }
        const uint32_t delayMs = fix ? 2000u : 5000u;
        if ((uint32_t)(nowMs-candidateSinceMs_) < delayMs) return Event::None;
        if (fix && !announcedFix_) {
            announcedFix_ = true;
            return Event::Acquired;
        }
        if (!fix && announcedFix_) {
            announcedFix_ = false;
            return Event::Lost;
        }
        return Event::None;
    }

private:
    bool candidateFix_ = false;
    bool announcedFix_ = false;
    uint32_t candidateSinceMs_ = 0;
};
