#include "audio_pcm.h"
#include <math.h>

AudioPcm::AudioPcm() {
    for (int i=0; i<=256; ++i) sine_[i] = sinf(6.28318530718f*i/256.0f);
}

void AudioPcm::render(int16_t* stereo, size_t frames, uint32_t frequency, int volume) {
    const bool audible = frequency >= 150 && frequency <= 5000 && volume > 0;
    if (audible) frequency_ = frequency;
    if (volume > 100) volume = 100;
    const float target = audible ? MAX_AMPLITUDE * (volume/100.0f) : 0;
    const float step = (float)MAX_AMPLITUDE / RAMP_SAMPLES;
    const float phaseStep = 256.0f * frequency_ / SAMPLE_RATE;
    for (size_t i=0; i<frames; ++i) {
        if (amplitude_ < target) amplitude_ = fminf(amplitude_+step, target);
        else if (amplitude_ > target) amplitude_ = fmaxf(amplitude_-step, target);
        const unsigned index = (unsigned)phase_;
        const float wave = sine_[index] + (sine_[index+1]-sine_[index])*(phase_-index);
        const int16_t sample = (int16_t)lroundf(wave*amplitude_);
        stereo[2*i] = stereo[2*i+1] = sample;
        phase_ += phaseStep;
        if (phase_ >= 256) phase_ -= 256;
    }
}
