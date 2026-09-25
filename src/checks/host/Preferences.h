#pragma once
#include <map>
#include <string>
#include <vector>
#include <cstring>
extern bool failStorage;
extern std::map<std::string, std::vector<uint8_t>> fakeNvs;
class Preferences {
public:
    bool begin(const char*, bool) { return true; }
    size_t getBytesLength(const char* key) { return fakeNvs[key].size(); }
    size_t getBytes(const char* key, void* dst, size_t size) {
        auto& value = fakeNvs[key];
        if (value.size() != size) return 0;
        memcpy(dst, value.data(), size); return size;
    }
    size_t putBytes(const char* key, const void* src, size_t size) {
        if (failStorage) return 0;
        const auto* p = static_cast<const uint8_t*>(src);
        fakeNvs[key] = std::vector<uint8_t>(p, p+size); return size;
    }
    float getFloat(const char* key, float fallback) { float v; return getBytes(key, &v, sizeof(v)) ? v : fallback; }
    int getInt(const char* key, int fallback) { int v; return getBytes(key, &v, sizeof(v)) ? v : fallback; }
};
