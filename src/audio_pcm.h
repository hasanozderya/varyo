#pragma once
#include <stdint.h>
#include <stddef.h>

struct AudioToneRequest {
    uint32_t frequency = 0;
    int volume = 0;
    uint32_t updatedMs = 0;
    uint32_t effectiveFrequency(uint32_t nowMs) const {
        return (uint32_t)(nowMs-updatedMs) < 200 && volume > 0 &&
            frequency >= 150 && frequency <= 5000 ? frequency : 0;
    }
};

// Pure PCM generator, also exercised by host tests. Identical L/R samples
// support either channel-selection setting on common MAX98357A modules.
class AudioPcm {
public:
    static constexpr uint32_t SAMPLE_RATE = 16000;
    static constexpr int MAX_AMPLITUDE = 26213; // 0.8 * signed 16-bit full scale
    static constexpr int RAMP_SAMPLES = 80;     // 5 ms fade, no DC-biased PWM
    AudioPcm();
    void render(int16_t* stereo, size_t frames, uint32_t frequency, int volume);
private:
    float sine_[257];
    float phase_ = 0, amplitude_ = 0;
    uint32_t frequency_ = 700;
};
