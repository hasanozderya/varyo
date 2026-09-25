#pragma once
#include <Arduino.h>

// Cross-core-safe rolling text log. Core 0 (and anywhere else) calls
// log()/logf() to append; the Core-1 web task serves the accumulated
// buffer as plain text at /log. Every write is also mirrored to Serial
// in case that ever becomes usable again — this does not replace it,
// only adds a second, more reliable channel.
namespace DebugLog {
    void init();
    void log(const char* text);
    void logf(const char* fmt, ...);
    String snapshot();   // thread-safe copy of the current buffer content

    // Returns whatever text has been appended since the last call
    // (thread-safe). Meant for a single periodic low-priority caller
    // (the web task) to mirror new log lines to Serial — this is the
    // ONLY place Serial is touched, keeping the real-time vario task
    // completely immune to a stuck/blocked USB-CDC port. If some lines
    // were evicted from the ring buffer before being pulled, they are
    // silently skipped (debug-only, acceptable loss).
    String pullNewForSerial();
}
