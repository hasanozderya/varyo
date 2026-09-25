#include "config.h"
#if !VARIO_USE_WIFI
#include "ble_commands.h"
#include "tunables.h"
#include "shared_state.h"
#include "vario_task.h"
#include "trend_buffer.h"
#include "debug_log.h"
#include "firmware_info.h"
#include <esp_system.h>
#include "flight_state.h"
#include <cJSON.h>
#include <math.h>
#include <string.h>
#include <memory>

namespace {
    bool shallowJson(const char* text) {
        unsigned depth = 0;
        bool quoted = false, escaped = false;
        for (const char* p = text; *p; ++p) {
            if (quoted) {
                if (escaped) escaped = false;
                else if (*p == '\\') escaped = true;
                else if (*p == '"') quoted = false;
            } else if (*p == '"') quoted = true;
            else if (*p == '{' || *p == '[') { if (++depth > 4) return false; }
            else if (*p == '}' || *p == ']') { if (!depth) return false; --depth; }
        }
        return !quoted && depth == 0;
    }
    using Json = std::unique_ptr<cJSON, decltype(&cJSON_Delete)>;
    Json own(cJSON* p) { return Json(p, cJSON_Delete); }
    String print(cJSON* object) {
        char* text = cJSON_PrintUnformatted(object);
        if (!text) return "{\"id\":null,\"ok\":false,\"error\":\"out_of_memory\"}";
        String result(text);
        cJSON_free(text);
        return result;
    }
    bool number(cJSON* object, const char* key, double& value) {
        cJSON* item = cJSON_GetObjectItemCaseSensitive(object, key);
        if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble)) return false;
        value = item->valuedouble;
        return true;
    }
    bool integer(cJSON* object, const char* key, double& value, double min, double max) {
        return number(object, key, value) && value >= min && value <= max && floor(value) == value;
    }
    bool allowed(cJSON* object, const char* keys) {
        if (!cJSON_IsObject(object)) return false;
        for (cJSON* p = object->child; p; p = p->next) {
            if (!p->string || strlen(p->string) > 40 || strchr(p->string, '|')) return false;
            char token[44];
            snprintf(token, sizeof(token), "|%s|", p->string);
            if (!strstr(keys, token)) return false;
            for (cJSON* q = p->next; q; q = q->next)
                if (q->string && strcmp(p->string, q->string) == 0) return false;
        }
        return true;
    }
    bool patchFloat(cJSON* params, const char* key, float& value) {
        if (!cJSON_HasObjectItem(params, key)) return true;
        double n;
        if (!number(params, key, n) || !isfinite((float)n)) return false;
        value = (float)n;
        return true;
    }
    bool patchTone(cJSON* params, const char* key, int& value) {
        if (!cJSON_HasObjectItem(params, key)) return true;
        double n;
        if (!integer(params, key, n, 150, 5000)) return false;
        value = (int)n;
        return true;
    }
    bool patchInt(cJSON* params, const char* key, int& value, int lo, int hi) {
        if (!cJSON_HasObjectItem(params, key)) return true;
        double n;
        if (!integer(params,key,n,lo,hi)) return false;
        value = (int)n; return true;
    }
    bool patchBool(cJSON* params, const char* key, bool& value) {
        if (!cJSON_HasObjectItem(params,key)) return true;
        const cJSON* item = cJSON_GetObjectItemCaseSensitive(params,key);
        if (!cJSON_IsBool(item)) return false;
        value = cJSON_IsTrue(item); return true;
    }
    void settings(cJSON* out) {
        const Tunables t = TunablesStore::read();
        cJSON* f = cJSON_AddObjectToObject(out, "fusion");
        cJSON_AddNumberToObject(f, "alpha", t.fusion.compFilterAlpha);
        cJSON_AddNumberToObject(f, "kfA", t.fusion.kfAccelVar);
        cJSON_AddNumberToObject(f, "kfAB", t.fusion.kfAccelBiasVar);
        cJSON_AddNumberToObject(f, "kfBaro", t.fusion.kfBaroVar);
        cJSON_AddNumberToObject(f, "qnh", t.fusion.qnhHpa);
        cJSON* a = cJSON_AddObjectToObject(out, "audio");
        cJSON_AddNumberToObject(a, "deadband", t.audio.climbDeadbandMps);
        cJSON_AddNumberToObject(a, "climbMax", t.audio.climbMaxMps);
        cJSON_AddNumberToObject(a, "sinkAlarm", t.audio.sinkAlarmMps);
        cJSON_AddNumberToObject(a, "toneMin", t.audio.toneMinHz);
        cJSON_AddNumberToObject(a, "toneMax", t.audio.toneMaxHz);
        cJSON_AddNumberToObject(a, "sinkTone", t.audio.sinkToneHz);
        cJSON_AddNumberToObject(a, "strongSink", t.audio.strongSinkMps);
        cJSON_AddNumberToObject(a, "hysteresis", t.audio.hysteresisMps);
        cJSON_AddNumberToObject(a, "volume", t.audio.volume);
        cJSON_AddNumberToObject(a, "periodMin", t.audio.periodMinMs);
        cJSON_AddNumberToObject(a, "periodMax", t.audio.periodMaxMs);
        cJSON_AddBoolToObject(a, "weakLift", t.audio.weakLift);
    }
}

String executeBleCommand(const char* jsonLine) {
    auto root = own(shallowJson(jsonLine) ? cJSON_ParseWithOpts(jsonLine, nullptr, true) : nullptr);
    auto reply = own(cJSON_CreateObject());
    if (!reply) return "{\"id\":null,\"ok\":false,\"error\":\"out_of_memory\"}";
    double id = 0;
    const bool hasId = root && integer(root.get(), "id", id, 1, 2147483647);
    if (hasId) cJSON_AddNumberToObject(reply.get(), "id", id);
    else cJSON_AddNullToObject(reply.get(), "id");
    auto fail = [&](const char* error) {
        cJSON_AddBoolToObject(reply.get(), "ok", false);
        cJSON_AddStringToObject(reply.get(), "error", error);
        return print(reply.get());
    };
    if (!hasId || !allowed(root.get(), "|id||cmd||params|")) return fail("invalid_request");
    cJSON* command = cJSON_GetObjectItemCaseSensitive(root.get(), "cmd");
    if (!cJSON_IsString(command)) return fail("invalid_command");
    auto empty = own(cJSON_CreateObject());
    cJSON* params = cJSON_GetObjectItemCaseSensitive(root.get(), "params");
    if (!params) params = empty.get();
    if (!cJSON_IsObject(params)) return fail("invalid_params");
    auto result = own(cJSON_CreateObject());
    if (!result) return fail("out_of_memory");
    const char* cmd = command->valuestring;

    if (!strcmp(cmd, "status")) {
        if (!allowed(params, "")) return fail("unknown_param");
        const VarioState s = SharedState::readFresh(millis());
        cJSON_AddNumberToObject(result.get(), "protocol", 1);
        cJSON_AddBoolToObject(result.get(), "xcTrackCompatible", true);
        cJSON_AddStringToObject(result.get(), "firmware", FirmwareInfo::VERSION);
        cJSON_AddStringToObject(result.get(), "build", FirmwareInfo::BUILD);
        cJSON_AddNumberToObject(result.get(), "measurementMs", s.updatedMs);
        cJSON_AddNumberToObject(result.get(), "measurementSequence", s.sampleSequence);
        cJSON_AddBoolToObject(result.get(), "outputReady", s.outputReady);
        cJSON_AddBoolToObject(result.get(), "groundStable", s.groundStable);
        cJSON_AddNumberToObject(result.get(), "flightMode", s.flightMode);
        cJSON_AddNumberToObject(result.get(), "flightSession", s.flightSession);
        cJSON_AddNumberToObject(result.get(), "flightDurationMs", s.flightDurationMs);
        cJSON_AddBoolToObject(result.get(), "settingsStorageOk", TunablesStore::storageReady());
        cJSON_AddNumberToObject(result.get(), "referenceRevision", s.referenceRevision);
        cJSON_AddNumberToObject(result.get(), "baroAgeMs", (uint32_t)(millis()-s.baroSampleMs));
        cJSON_AddNumberToObject(result.get(), "imuAgeMs", (uint32_t)(millis()-s.imuSampleMs));
        cJSON_AddNumberToObject(result.get(), "longTicks", s.longTicks);
        cJSON_AddNumberToObject(result.get(), "maxTickUs", s.maxTickUs);
        cJSON_AddNumberToObject(result.get(), "baroSamples", s.baroSamples);
        cJSON_AddNumberToObject(result.get(), "imuSamples", s.imuSamples);
        cJSON_AddNumberToObject(result.get(), "sensorRecoveries", s.sensorRecoveries);
        cJSON_AddNumberToObject(result.get(), "stackFreeBytes", s.stackFreeBytes);
        cJSON_AddNumberToObject(result.get(), "heapFreeBytes", ESP.getFreeHeap());
        cJSON_AddNumberToObject(result.get(), "heapMinimumBytes", ESP.getMinFreeHeap());
        cJSON_AddNumberToObject(result.get(), "resetReason", (int)esp_reset_reason());
        const GpsState gps = SharedState::readGps(millis());
        cJSON* g = cJSON_AddObjectToObject(result.get(), "gps");
        cJSON_AddBoolToObject(g,"fix",gps.fix);
        cJSON_AddBoolToObject(g,"motionValid",gps.motionValid);
        cJSON_AddBoolToObject(g,"altitudeValid",gps.altitudeValid);
        cJSON_AddNumberToObject(g,"latitude",gps.latitude);
        cJSON_AddNumberToObject(g,"longitude",gps.longitude);
        cJSON_AddNumberToObject(g,"altitude",gps.altitudeM);
        cJSON_AddNumberToObject(g,"speed",gps.speedMps);
        cJSON_AddNumberToObject(g,"course",gps.courseDeg);
        cJSON_AddNumberToObject(g,"hdop",gps.hdop);
        cJSON_AddNumberToObject(g,"satellites",gps.satellites);
        cJSON_AddNumberToObject(g,"ageMs",(uint32_t)(millis()-gps.fixMs));
        if (gps.utcValid) cJSON_AddNumberToObject(g,"utcSeconds",gps.utcSeconds);
        cJSON_AddNumberToObject(result.get(), "timeMs", millis());
        cJSON_AddNumberToObject(result.get(), "alt", s.altitudeM);
        cJSON_AddNumberToObject(result.get(), "vario", s.climbRateMps);
        cJSON_AddNumberToObject(result.get(), "pressure", s.pressurePa);
        cJSON_AddNumberToObject(result.get(), "baroAlt", s.baroAltitudeM);
        cJSON_AddNumberToObject(result.get(), "earthZAccel", s.earthZAccelMps2);
        cJSON_AddNumberToObject(result.get(), "kalmanBias", s.kalmanAccelBias);
        cJSON_AddNumberToObject(result.get(), "pitch", s.pitchDeg);
        cJSON_AddNumberToObject(result.get(), "roll", s.rollDeg);
        cJSON_AddBoolToObject(result.get(), "baroOk", s.baroOk);
        cJSON_AddBoolToObject(result.get(), "imuOk", s.imuOk);
        cJSON_AddBoolToObject(result.get(), "imuCalibrated", s.imuCalibrated);
        cJSON_AddBoolToObject(result.get(), "imuFusionActive", s.imuFusionActive);
        cJSON_AddNumberToObject(result.get(), "imuCalibrationStatus", s.imuCalibrationStatus);
    } else if (!strcmp(cmd, "getSettings")) {
        if (!allowed(params, "")) return fail("unknown_param");
        settings(result.get());
    } else if (!strcmp(cmd, "setFusion")) {
        if (!params->child || !allowed(params, "|alpha||kfA||kfAB||kfBaro||qnh|")) return fail("invalid_params");
        Tunables t = TunablesStore::read();
        if (!patchFloat(params, "alpha", t.fusion.compFilterAlpha) ||
            !patchFloat(params, "kfA", t.fusion.kfAccelVar) ||
            !patchFloat(params, "kfAB", t.fusion.kfAccelBiasVar) ||
            !patchFloat(params, "kfBaro", t.fusion.kfBaroVar) ||
            !patchFloat(params, "qnh", t.fusion.qnhHpa) ||
            !TunablesStore::validFusion(t.fusion)) return fail("invalid_fusion");
        if (!TunablesStore::writeFusion(t.fusion)) return fail("settings_save_failed");
        settings(result.get());
    } else if (!strcmp(cmd, "setAudio")) {
        if (!params->child || !allowed(params, "|deadband||climbMax||sinkAlarm||toneMin||toneMax||sinkTone||strongSink||hysteresis||volume||periodMin||periodMax||weakLift|")) return fail("invalid_params");
        Tunables t = TunablesStore::read();
        if (!patchFloat(params, "deadband", t.audio.climbDeadbandMps) ||
            !patchFloat(params, "climbMax", t.audio.climbMaxMps) ||
            !patchFloat(params, "sinkAlarm", t.audio.sinkAlarmMps) ||
            !patchTone(params, "toneMin", t.audio.toneMinHz) ||
            !patchTone(params, "toneMax", t.audio.toneMaxHz) ||
            !patchTone(params, "sinkTone", t.audio.sinkToneHz) ||
            !patchFloat(params, "strongSink", t.audio.strongSinkMps) ||
            !patchFloat(params, "hysteresis", t.audio.hysteresisMps) ||
            !patchInt(params, "volume", t.audio.volume, 0, 100) ||
            !patchInt(params, "periodMin", t.audio.periodMinMs, 60, 2000) ||
            !patchInt(params, "periodMax", t.audio.periodMaxMs, 60, 2000) ||
            !patchBool(params, "weakLift", t.audio.weakLift) ||
            !TunablesStore::validAudio(t.audio)) return fail("invalid_audio");
        if (!TunablesStore::writeAudio(t.audio)) return fail("settings_save_failed");
        settings(result.get());
    } else if (!strcmp(cmd, "calibrateAltitude")) {
        double altitude;
        if (!allowed(params, "|altitude|") || !number(params, "altitude", altitude) ||
            altitude < -500 || altitude > 9000) return fail("invalid_altitude");
        const VarioState s = SharedState::readFresh(millis());
        if (!s.baroOk || !isfinite(s.pressurePa) || s.pressurePa < 20000 || s.pressurePa > 120000)
            return fail("barometer_unavailable");
        const float qnh = s.pressurePa / (100.0f * powf(1.0f - (float)altitude / 44330.0f, 1.0f / 0.190295f));
        Tunables t = TunablesStore::read();
        t.fusion.qnhHpa = qnh;
        if (!TunablesStore::validFusion(t.fusion)) return fail("invalid_qnh");
        if (!TunablesStore::writeFusion(t.fusion)) return fail("settings_save_failed");
        cJSON_AddNumberToObject(result.get(), "qnh", qnh);
        cJSON_AddNumberToObject(result.get(), "altitude", altitude);
    } else if (!strcmp(cmd, "calibrateImu")) {
        if (!allowed(params, "|ground|") || !cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(params, "ground")))
            return fail("ground_required");
        const VarioState s = SharedState::readFresh(millis());
        if (!s.imuOk) return fail("imu_unavailable");
        if (!s.groundStable) return fail("ground_not_stable");
        if ((s.imuCalibrationStatus == 1 || s.imuCalibrationStatus == 5) || !requestGroundImuCalibration()) return fail("calibration_busy");
        cJSON_AddBoolToObject(result.get(), "accepted", true);
    } else if (!strcmp(cmd, "startFlight")) {
        if (!allowed(params, "")) return fail("unknown_param");
        if (!SharedState::readFresh(millis()).outputReady) return fail("measurement_unavailable");
        FlightControl::requestStart();
        cJSON_AddBoolToObject(result.get(), "accepted", true);
    } else if (!strcmp(cmd, "stopFlight")) {
        if (!allowed(params, "")) return fail("unknown_param");
        FlightControl::requestStop();
        cJSON_AddBoolToObject(result.get(), "accepted", true);
    } else if (!strcmp(cmd, "getLog")) {
        if (!allowed(params, "")) return fail("unknown_param");
        cJSON_AddStringToObject(result.get(), "text", DebugLog::snapshot().c_str());
    } else if (!strcmp(cmd, "getTrend")) {
        if (!allowed(params, "|since||limit|")) return fail("unknown_param");
        double since = 0, limit = 30;
        if ((cJSON_HasObjectItem(params, "since") && !integer(params, "since", since, 0, 4294967295.0)) ||
            (cJSON_HasObjectItem(params, "limit") && !integer(params, "limit", limit, 1, 100))) return fail("invalid_params");
        std::unique_ptr<TrendPoint[]> points(new (std::nothrow) TrendPoint[(size_t)limit]);
        if (!points) return fail("out_of_memory");
        bool more = false;
        const size_t n = TrendBuffer::page((uint32_t)since, points.get(), (size_t)limit, more);
        cJSON* list = cJSON_AddArrayToObject(result.get(), "points");
        uint32_t next = (uint32_t)since;
        for (size_t i = 0; i < n; ++i) {
            const TrendPoint& p = points[i];
            const double values[] = {(double)p.tMs, p.altitudeM, p.varioMps, p.pressurePa};
            cJSON_AddItemToArray(list, cJSON_CreateDoubleArray(values, 4));
            next = p.tMs;
        }
        cJSON_AddNumberToObject(result.get(), "nextSince", next);
        cJSON_AddBoolToObject(result.get(), "hasMore", more);
        cJSON_AddNumberToObject(result.get(), "capacity", TrendBuffer::capacity());
    } else return fail("unknown_command");

    cJSON_AddBoolToObject(reply.get(), "ok", true);
    cJSON_AddItemToObject(reply.get(), "result", result.release());
    return print(reply.get());
}
#endif
