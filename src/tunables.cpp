#include "tunables.h"
#include "config.h"
#include <Preferences.h>
#include <freertos/semphr.h>
#include <math.h>
#include <stddef.h>

bool TunablesStore::validFusion(const FusionParams& f) {
    return isfinite(f.compFilterAlpha) && f.compFilterAlpha >= 0 && f.compFilterAlpha < 1 &&
        isfinite(f.kfAccelVar) && f.kfAccelVar > 0 && f.kfAccelVar <= 1000 &&
        isfinite(f.kfAccelBiasVar) && f.kfAccelBiasVar >= 0 && f.kfAccelBiasVar <= 10 &&
        isfinite(f.kfBaroVar) && f.kfBaroVar > 0 && f.kfBaroVar <= 1000 &&
        isfinite(f.qnhHpa) && f.qnhHpa >= 800 && f.qnhHpa <= 1100 &&
        isfinite(f.kAdaptFactor) && f.kAdaptFactor >= 0 && f.kAdaptFactor <= 10;
}
bool TunablesStore::validAudio(const AudioParams& a) {
    return isfinite(a.climbDeadbandMps) && a.climbDeadbandMps >= 0 &&
        isfinite(a.climbMaxMps) && a.climbMaxMps > a.climbDeadbandMps && a.climbMaxMps <= 20 &&
        isfinite(a.sinkAlarmMps) && a.sinkAlarmMps < 0 && a.sinkAlarmMps >= -20 &&
        a.toneMinHz >= 150 && a.toneMaxHz >= a.toneMinHz && a.toneMaxHz <= 5000 &&
        a.sinkToneHz >= 150 && a.sinkToneHz <= 5000 &&
        isfinite(a.strongSinkMps) && (a.strongSinkMps == 0 ||
            (a.strongSinkMps < a.sinkAlarmMps && a.strongSinkMps >= -25)) &&
        isfinite(a.hysteresisMps) && a.hysteresisMps >= 0 && a.hysteresisMps <= 0.3f &&
        a.volume >= 0 && a.volume <= 100 && a.periodMinMs >= 60 &&
        a.periodMaxMs >= a.periodMinMs && a.periodMaxMs <= 2000 &&
        isfinite(a.altitudeLimitM) && a.altitudeLimitM >= -500 && a.altitudeLimitM <= 9000 &&
        isfinite(a.altitudeWarningMarginM) && a.altitudeWarningMarginM >= 20 &&
        a.altitudeWarningMarginM <= 1000;
}
namespace {
    void upgradeLegacyTones(AudioParams& a) {
        if (a.toneMinHz == 600 && a.toneMaxHz == 1800 && a.sinkToneHz == 300) {
            a.toneMinHz = AudioTuning::TONE_MIN_HZ;
            a.toneMaxHz = AudioTuning::TONE_MAX_HZ;
            a.sinkToneHz = AudioTuning::SINK_TONE_HZ;
        }
    }
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    SemaphoreHandle_t writer = nullptr;
    Preferences prefs;
    bool available = false;
    Tunables current;
    struct Saved { Tunables value; uint32_t checksum; };
    static_assert(sizeof(Saved) == 84, "settings on-device layout changed");
    struct LegacyAudioParams {
        float climbDeadbandMps, climbMaxMps, sinkAlarmMps;
        int toneMinHz, toneMaxHz, sinkToneHz;
        float strongSinkMps, hysteresisMps;
        int volume, periodMinMs, periodMaxMs;
        bool weakLift;
    };
    struct LegacyTunables { FusionParams fusion; LegacyAudioParams audio; };
    struct LegacySaved { LegacyTunables value; uint32_t checksum; };
    static_assert(sizeof(LegacySaved) == 76, "legacy settings layout changed");
    template <typename Record> uint32_t checksum(const Record& saved) {
        uint32_t h = 2166136261u;
        const auto* p = reinterpret_cast<const uint8_t*>(&saved);
        for (size_t i = 0; i < offsetof(Record, checksum); ++i) h = (h ^ p[i]) * 16777619u;
        return h;
    }
    bool persist(const Tunables& value) {
        Saved saved{};
        saved.value = value;
        saved.checksum = checksum(saved);
        return available && prefs.putBytes("settings", &saved, sizeof(saved)) == sizeof(saved);
    }
    void publish(const Tunables& value) {
        portENTER_CRITICAL(&mux);
        current = value;
        portEXIT_CRITICAL(&mux);
    }
}
void TunablesStore::init() {
    writer = xSemaphoreCreateMutex();
    configASSERT(writer != nullptr);
    available = prefs.begin("vario", false);
    current.fusion = {VarioTuning::COMP_FILTER_ALPHA, VarioTuning::KF_ACCEL_VAR,
        VarioTuning::KF_ACCEL_BIAS_VAR, VarioTuning::KF_BARO_VAR, 1013.25f, VarioTuning::KF_ADAPT_FACTOR};
    current.audio = {VarioTuning::CLIMB_DEADBAND_MPS, VarioTuning::CLIMB_MAX_MPS,
        VarioTuning::SINK_ALARM_MPS, AudioTuning::TONE_MIN_HZ, AudioTuning::TONE_MAX_HZ,
        AudioTuning::SINK_TONE_HZ};
    if (!available) return;
    Saved saved{};
    bool loaded = false;
    bool migrated = false;
    if (prefs.getBytesLength("settings") == sizeof(saved) &&
        prefs.getBytes("settings", &saved, sizeof(saved)) == sizeof(saved) &&
        saved.checksum == checksum(saved) && validFusion(saved.value.fusion) &&
        validAudio(saved.value.audio)) {
        current = saved.value;
        loaded = true;
    } else if (prefs.getBytesLength("settings") == sizeof(LegacySaved)) {
        LegacySaved legacy{};
        if (prefs.getBytes("settings", &legacy, sizeof(legacy)) == sizeof(legacy) &&
            legacy.checksum == checksum(legacy)) {
            Tunables upgraded = current;
            upgraded.fusion = legacy.value.fusion;
            upgraded.audio.climbDeadbandMps = legacy.value.audio.climbDeadbandMps;
            upgraded.audio.climbMaxMps = legacy.value.audio.climbMaxMps;
            upgraded.audio.sinkAlarmMps = legacy.value.audio.sinkAlarmMps;
            upgraded.audio.toneMinHz = legacy.value.audio.toneMinHz;
            upgraded.audio.toneMaxHz = legacy.value.audio.toneMaxHz;
            upgraded.audio.sinkToneHz = legacy.value.audio.sinkToneHz;
            upgraded.audio.strongSinkMps = legacy.value.audio.strongSinkMps;
            upgraded.audio.hysteresisMps = legacy.value.audio.hysteresisMps;
            upgraded.audio.volume = legacy.value.audio.volume;
            upgraded.audio.periodMinMs = legacy.value.audio.periodMinMs;
            upgraded.audio.periodMaxMs = legacy.value.audio.periodMaxMs;
            upgraded.audio.weakLift = legacy.value.audio.weakLift;
            if (validFusion(upgraded.fusion) && validAudio(upgraded.audio)) {
                current = upgraded;
                loaded = migrated = true;
            }
        }
    }
    upgradeLegacyTones(current.audio);
    if (!loaded) {
        // This namespace owns only runtime settings. Replace every obsolete or
        // corrupt layout with one canonical record; IMU calibration lives in
        // its separate "imu_cal" namespace.
        prefs.clear();
        persist(current);
    } else if (migrated) persist(current);
}
Tunables TunablesStore::read() {
    portENTER_CRITICAL(&mux);
    const Tunables result = current;
    portEXIT_CRITICAL(&mux);
    return result;
}
bool TunablesStore::storageReady() { return available; }
bool TunablesStore::write(const Tunables& settings) {
    if (!validFusion(settings.fusion) || !validAudio(settings.audio) || !writer ||
        xSemaphoreTake(writer, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
    const bool ok = persist(settings);
    if (ok) publish(settings);
    xSemaphoreGive(writer);
    return ok;
}
