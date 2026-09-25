#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <algorithm>
#include <assert.h>
using std::min;
using std::max;
#define PI 3.14159265358979323846
#define DEG_TO_RAD (PI/180.0)
#define RAD_TO_DEG (180.0/PI)
#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define configASSERT(x) assert(x)
extern uint64_t fakeUs;
extern uint32_t fakeToneHz, fakeDuty, toneChanges;
inline uint32_t micros() { return (uint32_t)fakeUs; }
inline uint32_t millis() { return (uint32_t)(fakeUs/1000); }
inline bool ledcAttach(int, uint32_t, int) { return true; }
inline uint32_t ledcChangeFrequency(int, uint32_t hz, int) { ++toneChanges; fakeToneHz = hz; return hz; }
inline bool ledcWrite(int, uint32_t duty) { fakeDuty = duty; return true; }
