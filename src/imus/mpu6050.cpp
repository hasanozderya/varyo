#include "mpu6050.h"

bool Mpu6050Driver::begin() {
    if (!I2CBus::ping(I2CAddr::IMU, 0)) return false;
    startRecovery();
    for (unsigned i = 0; i < 50; ++i) {
        if (serviceRecovery()) return true;
        delay(5);
    }
    return false;
}

void Mpu6050Driver::startRecovery() { recoveryStep_ = 0; }

bool Mpu6050Driver::serviceRecovery() {
    if (recoveryStep_ == 0) {
        if (!ImuI2C::write(0x6B, 0x80)) return false;
        recoveryMs_ = millis();
        recoveryStep_ = 1;
        return false;
    }
    if (recoveryStep_ == 1) {
        if ((uint32_t)(millis() - recoveryMs_) < 100) return false;
        if (!ImuI2C::write(0x6B, 0x01)) return false;
        recoveryMs_ = millis();
        recoveryStep_ = 2;
        return false;
    }
    if (recoveryStep_ == 2) {
        if ((uint32_t)(millis() - recoveryMs_) < 50) return false;
        uint8_t id;
        if (!ImuI2C::readByte(0x75, id) ||
            (id != 0x68 && id != 0x70 && id != 0x71 && id != 0x73)) return false;
        if (!ImuI2C::write(0x1A, 0x03) || !ImuI2C::write(0x19, 9) ||
            !ImuI2C::write(0x1B, 0x08) || !ImuI2C::write(0x1C, 0x10)) return false;
        recoveryStep_ = 3;
    }
    return recoveryStep_ == 3;
}

bool Mpu6050Driver::read(ImuHardwareSample& sample) {
    uint8_t ready, bytes[14];
    if (!ImuI2C::readByte(0x3A, ready) || !(ready & 1) ||
        !ImuI2C::read(0x3B, bytes, sizeof(bytes))) return false;
    int16_t raw[7];
    for (int i = 0; i < 7; ++i)
        raw[i] = (int16_t)(((uint16_t)bytes[i * 2] << 8) | bytes[i * 2 + 1]);
    for (int i = 0; i < 3; ++i) sample.accelG[i] = raw[i] / 4096.0f;
    for (int i = 0; i < 3; ++i) sample.gyroDps[i] = raw[i + 4] / 65.5f;
    sample.temperatureC = raw[3] / 340.0f + 36.53f;
    sample.accelSaturated = ImuI2C::clipped(raw[0], raw[1], raw[2]);
    sample.gyroSaturated = ImuI2C::clipped(raw[4], raw[5], raw[6]);
    return true;
}
