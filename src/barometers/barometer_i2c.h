#pragma once

#include "config.h"
#include "i2c_bus.h"
#include <Wire.h>

// Shared register access for I2C barometers. Drivers keep their own recovery
// counters, while bus locking and transaction validation live in one place.
namespace BarometerI2C {

inline bool writeRegister(uint8_t reg, uint8_t value, bool nonBlocking,
                          uint32_t& runtimeBusMisses) {
    I2CLockGuard lock(nonBlocking ? 0 : Timing::I2C_MUTEX_TIMEOUT);
    if (!lock.ok()) {
        if (nonBlocking) ++runtimeBusMisses;
        return false;
    }
    Wire.beginTransmission(I2CAddr::BAROMETER);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

inline bool readRegisters(uint8_t reg, uint8_t* data, size_t size,
                          bool nonBlocking, uint32_t& runtimeBusMisses,
                          bool repeatedStart = true) {
    I2CLockGuard lock(nonBlocking ? 0 : Timing::I2C_MUTEX_TIMEOUT);
    if (!lock.ok()) {
        if (nonBlocking) ++runtimeBusMisses;
        return false;
    }
    Wire.beginTransmission(I2CAddr::BAROMETER);
    Wire.write(reg);
    if (Wire.endTransmission(repeatedStart ? false : true) != 0 ||
        Wire.requestFrom((uint8_t)I2CAddr::BAROMETER, (uint8_t)size) != (int)size) {
        return false;
    }
    for (size_t i = 0; i < size; ++i) data[i] = (uint8_t)Wire.read();
    return true;
}

} // namespace BarometerI2C
