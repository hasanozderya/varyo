#pragma once
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"

// Thread-safe wrapper around the shared I2C bus. Every peripheral driver
// (selected barometer, selected IMU, SSD1306/U8g2) must acquire this lock before touching
// Wire and release it immediately after. Hold times should be a few
// hundred microseconds — never span a full sensor conversion cycle, or
// you'll stall the other core's bus access for milliseconds at a time.
namespace I2CBus {
    void begin();
    bool ping(uint8_t addr, TickType_t timeout = 0);
    bool recover(); // bounded stuck-SDA recovery, only between transactions
    void sensorCycleComplete();
    bool displayWindow();
    bool lock(TickType_t timeout = Timing::I2C_MUTEX_TIMEOUT);
    void unlock();
}

// RAII guard — the preferred way to take the lock so it can never be
// left held on an early return.
class I2CLockGuard {
public:
    explicit I2CLockGuard(TickType_t timeout = Timing::I2C_MUTEX_TIMEOUT)
        : locked_(I2CBus::lock(timeout)) {}
    ~I2CLockGuard() { if (locked_) I2CBus::unlock(); }
    bool ok() const { return locked_; }
    I2CLockGuard(const I2CLockGuard&) = delete;
    I2CLockGuard& operator=(const I2CLockGuard&) = delete;
private:
    bool locked_;
};
