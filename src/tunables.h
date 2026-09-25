#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>

// Runtime-adjustable tuning parameters. config.h's VarioTuning/AudioTuning
// namespaces remain the compile-time DEFAULTS used to seed this store at
// startup; from then on, the web task can override individual fields
// live (e.g. while bench-testing) without a reflash. Protected by a
// spinlock since this is a small POD struct copy — read once per vario
// tick on Core 0, written occasionally from the Core-1 web task.
struct FusionParams {
    float compFilterAlpha;
    float kfAccelVar;
    float kfAccelBiasVar;
    float kfBaroVar;
    float qnhHpa;
    float kAdaptFactor;
};

struct AudioParams {
    float climbDeadbandMps;
    float climbMaxMps;
    float sinkAlarmMps;
    int   toneMinHz;
    int   toneMaxHz;
    int   sinkToneHz;
    float strongSinkMps = -5.0f;
    float hysteresisMps = 0.05f;
    int volume = 100;
    int periodMinMs = 120;
    int periodMaxMs = 700;
    bool weakLift = false;
};

struct Tunables {
    FusionParams fusion;
    AudioParams  audio;
};

namespace TunablesStore {
    bool validFusion(const FusionParams& f);
    bool validAudio(const AudioParams& a);
    bool storageReady();
    void init();                              // seed with config.h defaults
    Tunables read();
    bool writeFusion(const FusionParams& f);
    bool writeAudio(const AudioParams& a);
}
