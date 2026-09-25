#pragma once
#include <stdint.h>

namespace AudioOutput {
    // Called once from setup. Allocates a dedicated Core-1 audio task.
    bool begin();
    // Nonblocking mailbox; refresh every sensor tick, including steady tones.
    void setTone(uint32_t frequency, int volume);
}
