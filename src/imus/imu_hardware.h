#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "i2c_bus.h"

struct ImuHardwareSample {
    float accelG[3] = {};
    float gyroDps[3] = {};
    float temperatureC = 0.0f;
    bool accelSaturated = false;
    bool gyroSaturated = false;
};

namespace ImuI2C {
inline bool write(uint8_t reg, uint8_t value, TickType_t timeout = 0) {
    I2CLockGuard lock(timeout);
    if (!lock.ok()) return false;
    Wire.beginTransmission(I2CAddr::IMU);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

inline bool read(uint8_t reg, uint8_t* data, size_t length, TickType_t timeout = 0) {
    I2CLockGuard lock(timeout);
    if (!lock.ok() || length > 255) return false;
    Wire.beginTransmission(I2CAddr::IMU);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom((uint8_t)I2CAddr::IMU, (uint8_t)length) != length) return false;
    for (size_t i = 0; i < length; ++i) data[i] = Wire.read();
    return true;
}

inline bool readByte(uint8_t reg, uint8_t& value, TickType_t timeout = 0) {
    return read(reg, &value, 1, timeout);
}

inline int16_t little16(const uint8_t* p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

inline bool clipped(int16_t x, int16_t y, int16_t z, int limit = 32000) {
    return abs((int)x) >= limit || abs((int)y) >= limit || abs((int)z) >= limit;
}
}
