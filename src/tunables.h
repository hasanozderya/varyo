#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>

// Runtime-adjustable tuning parameters. tuning_defaults.h values
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
    bool altitudeAlertEnabled = false;
    float altitudeLimitM = 2500.0f;
    float altitudeWarningMarginM = 100.0f;
};

struct Tunables {
    FusionParams fusion;
    AudioParams  audio;
};

namespace TunablesStore {
    bool validFusion(const FusionParams& f);
    bool validAudio(const AudioParams& a);
    bool storageReady();
    void init();                              // seed with compile-time defaults
    Tunables read();
    bool write(const Tunables& settings);
}
