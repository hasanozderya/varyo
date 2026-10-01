#pragma once
#include <stddef.h>
#include <stdint.h>

namespace AudioOutput {
    // Called once from setup. Allocates a dedicated Core-1 audio task.
    bool begin();
    // Nonblocking mailbox; refresh every sensor tick, including steady tones.
    void setTone(uint32_t frequency, int volume);
    // Flash-resident mono PCM, played once with priority over the tone mailbox.
    void playClip(const int16_t* samples, size_t sampleCount, int volume);
    void stopClip();
}
