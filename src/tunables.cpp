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
        a.periodMaxMs >= a.periodMinMs && a.periodMaxMs <= 2000;
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
    struct Saved { uint32_t version; Tunables value; uint32_t checksum; };
    // Before kAdaptFactor was added, settings_v2 stored only five fusion
    // floats. Never reinterpret that blob with the new struct layout.
    struct FusionV2 { float alpha, accelVar, biasVar, baroVar, qnh; };
    struct ValueV2 { FusionV2 fusion; AudioParams audio; };
    struct SavedV2 { uint32_t version; ValueV2 value; uint32_t checksum; };
    static_assert(sizeof(SavedV2) == 76, "settings_v2 on-device layout changed");
    static_assert(sizeof(Saved) == 80, "settings_v3 on-device layout changed");
    template <typename Record> uint32_t checksum(const Record& saved) {
        uint32_t h = 2166136261u;
        const auto* p = reinterpret_cast<const uint8_t*>(&saved);
        for (size_t i = 0; i < offsetof(Record, checksum); ++i) h = (h ^ p[i]) * 16777619u;
        return h;
    }
    bool persist(const Tunables& value) {
        Saved saved{};
        saved.version = 3;
        saved.value = value;
        saved.checksum = checksum(saved);
        return available && prefs.putBytes("settings_v3", &saved, sizeof(saved)) == sizeof(saved);
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
    if (prefs.getBytesLength("settings_v3") != 0) {
        if (prefs.getBytesLength("settings_v3") == sizeof(saved) &&
            prefs.getBytes("settings_v3", &saved, sizeof(saved)) == sizeof(saved) &&
            saved.version == 3 && saved.checksum == checksum(saved) &&
            validFusion(saved.value.fusion) && validAudio(saved.value.audio)) {
            current = saved.value;
        }
        // A corrupt current record must not resurrect an older calibration.
        upgradeLegacyTones(current.audio);
        return;
    }
    // Also accept the interim kAdapt firmware's larger, still-version-2 blob.
    if (prefs.getBytesLength("settings_v2") == sizeof(saved) &&
        prefs.getBytes("settings_v2", &saved, sizeof(saved)) == sizeof(saved) &&
        saved.version == 2 && saved.checksum == checksum(saved) &&
        validFusion(saved.value.fusion) && validAudio(saved.value.audio)) {
        current = saved.value;
        upgradeLegacyTones(current.audio);
        return;
    }
    SavedV2 old{};
    if (prefs.getBytesLength("settings_v2") == sizeof(old) &&
        prefs.getBytes("settings_v2", &old, sizeof(old)) == sizeof(old) &&
        old.version == 2 && old.checksum == checksum(old)) {
        const auto& f = old.value.fusion;
        Tunables migrated = current;
        migrated.fusion = {f.alpha, f.accelVar, f.biasVar, f.baroVar, f.qnh,
            VarioTuning::KF_ADAPT_FACTOR};
        migrated.audio = old.value.audio;
        if (validFusion(migrated.fusion) && validAudio(migrated.audio)) {
            current = migrated;
            upgradeLegacyTones(current.audio);
            return;
        }
    }
    // Read old keys without overwriting them. Migration is committed on edit.
    Tunables legacy = current;
    auto& f = legacy.fusion;
    f.compFilterAlpha = prefs.getFloat("f_alpha", f.compFilterAlpha);
    f.kfAccelVar = prefs.getFloat("f_kfA", f.kfAccelVar);
    f.kfAccelBiasVar = prefs.getFloat("f_kfAB", f.kfAccelBiasVar);
    f.kfBaroVar = prefs.getFloat("f_kfBaro", f.kfBaroVar);
    f.qnhHpa = prefs.getFloat("f_qnh", f.qnhHpa);
    f.kAdaptFactor = prefs.getFloat("f_kAdapt", f.kAdaptFactor);
    if (validFusion(f)) current.fusion = f;
    auto& a = legacy.audio;
    a.climbDeadbandMps = prefs.getFloat("a_dead", a.climbDeadbandMps);
    a.climbMaxMps = prefs.getFloat("a_cmax", a.climbMaxMps);
    a.sinkAlarmMps = prefs.getFloat("a_sink", a.sinkAlarmMps);
    a.toneMinHz = prefs.getInt("a_tmin", a.toneMinHz);
    a.toneMaxHz = prefs.getInt("a_tmax", a.toneMaxHz);
    a.sinkToneHz = prefs.getInt("a_stone", a.sinkToneHz);
    if (a.strongSinkMps >= a.sinkAlarmMps) a.strongSinkMps = 0;
    if (validAudio(a)) current.audio = a;
    upgradeLegacyTones(current.audio);
}
Tunables TunablesStore::read() {
    portENTER_CRITICAL(&mux);
    const Tunables result = current;
    portEXIT_CRITICAL(&mux);
    return result;
}
bool TunablesStore::storageReady() { return available; }
bool TunablesStore::writeFusion(const FusionParams& f) {
    if (!validFusion(f) || !writer || xSemaphoreTake(writer, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
    Tunables next = read(); next.fusion = f;
    const bool ok = persist(next);
    if (ok) publish(next);
    xSemaphoreGive(writer);
    return ok;
}
bool TunablesStore::writeAudio(const AudioParams& a) {
    if (!validAudio(a) || !writer || xSemaphoreTake(writer, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
    Tunables next = read(); next.audio = a;
    const bool ok = persist(next);
    if (ok) publish(next);
    xSemaphoreGive(writer);
    return ok;
}
