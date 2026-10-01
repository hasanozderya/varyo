#pragma once
#include <math.h>

// One-shot approach warning with hysteresis. It is intentionally driven by
// the same calibrated barometric altitude shown to the pilot.
class AltitudeAlertState {
public:
    bool update(bool enabled, bool flying, bool altitudeValid, float altitudeM,
                float limitM, float warningMarginM) {
        if (!enabled || !flying || !altitudeValid || !isfinite(altitudeM) ||
            !isfinite(limitM) || !isfinite(warningMarginM)) {
            if (!enabled || !flying) armed_ = true;
            return false;
        }
        const float threshold = limitM - warningMarginM;
        if (altitudeM < threshold - 50.0f) armed_ = true;
        if (!armed_ || altitudeM < threshold) return false;
        armed_ = false;
        return true;
    }
private:
    bool armed_ = true;
};
