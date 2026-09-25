#include "i2c_bus.h"
#include "config.h"
#include <atomic>

namespace {
    SemaphoreHandle_t s_mutex = nullptr;
    std::atomic<uint32_t> s_cycleUs{0};
}

void I2CBus::sensorCycleComplete() { s_cycleUs = micros(); }

bool I2CBus::displayWindow() {
    const uint32_t done = s_cycleUs;
    const uint32_t elapsed = micros() - done;
    // Leave the end of each 10 ms sensor period clear. If the sensor task
    // stops, still permit the UI to display its independently detected fault.
    return !done || elapsed < 4000 || elapsed > 30000;
}

void I2CBus::begin() {
    Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
    Wire.setClock(Timing::I2C_CLOCK_HZ);
    Wire.setTimeOut(Timing::I2C_TIMEOUT_MS);
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex != nullptr);
}

bool I2CBus::ping(uint8_t addr, TickType_t timeout) {
    I2CLockGuard lock(timeout);
    if (!lock.ok()) return false;
    Wire.beginTransmission(addr);
    const uint8_t result = Wire.endTransmission();
    return result == 0;
}

bool I2CBus::recover() {
    I2CLockGuard lock(0);
    if (!lock.ok() || digitalRead(Pins::I2C_SDA) != LOW) return false;
    Wire.end();
    pinMode(Pins::I2C_SDA, INPUT_PULLUP);
    pinMode(Pins::I2C_SCL, OUTPUT_OPEN_DRAIN);
    digitalWrite(Pins::I2C_SCL, HIGH);
    for (unsigned i = 0; i < 9 && digitalRead(Pins::I2C_SDA) == LOW; ++i) {
        digitalWrite(Pins::I2C_SCL, LOW); delayMicroseconds(5);
        digitalWrite(Pins::I2C_SCL, HIGH); delayMicroseconds(5);
    }
    // STOP, then restore the same configured bus; no guessed extra pins.
    pinMode(Pins::I2C_SDA, OUTPUT_OPEN_DRAIN);
    digitalWrite(Pins::I2C_SDA, LOW); delayMicroseconds(5);
    digitalWrite(Pins::I2C_SCL, HIGH); delayMicroseconds(5);
    digitalWrite(Pins::I2C_SDA, HIGH);
    Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
    Wire.setClock(Timing::I2C_CLOCK_HZ);
    Wire.setTimeOut(Timing::I2C_TIMEOUT_MS);
    return true;
}

bool I2CBus::lock(TickType_t timeout) {
    return s_mutex && xSemaphoreTake(s_mutex, timeout) == pdTRUE;
}

void I2CBus::unlock() {
    if (s_mutex) xSemaphoreGive(s_mutex);
}
