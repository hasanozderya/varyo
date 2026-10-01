#include "bmi160.h"

bool Bmi160Driver::begin() {
    if (!I2CBus::ping(I2CAddr::IMU, 0)) return false;
    startRecovery();
    for (int i = 0; i < 60; ++i) { if (serviceRecovery()) return true; delay(5); }
    return false;
}
void Bmi160Driver::startRecovery() { recoveryStep_ = 0; }
bool Bmi160Driver::serviceRecovery() {
    if (!recoveryStep_) {
        if (!ImuI2C::write(0x7E, 0xB6)) return false;
        recoveryMs_ = millis(); recoveryStep_ = 1; return false;
    }
    if (recoveryStep_ == 1) {
        if ((uint32_t)(millis() - recoveryMs_) < 100) return false;
        uint8_t id;
        if (!ImuI2C::readByte(0x00, id) || id != 0xD1 ||
            !ImuI2C::write(0x40, 0x28) || !ImuI2C::write(0x41, 0x08) ||
            !ImuI2C::write(0x42, 0x28) || !ImuI2C::write(0x43, 0x02) ||
            !ImuI2C::write(0x7E, 0x11)) return false;
        recoveryMs_ = millis(); recoveryStep_ = 2; return false;
    }
    if (recoveryStep_ == 2) {
        if ((uint32_t)(millis() - recoveryMs_) < 5) return false;
        if (!ImuI2C::write(0x7E, 0x15)) return false;
        recoveryMs_ = millis(); recoveryStep_ = 3; return false;
    }
    if (recoveryStep_ == 3 && (uint32_t)(millis() - recoveryMs_) >= 85) recoveryStep_ = 4;
    return recoveryStep_ == 4;
}
bool Bmi160Driver::read(ImuHardwareSample& sample) {
    uint8_t status, b[12], temp[2];
    if (!ImuI2C::readByte(0x1B, status) || (status & 0xC0) != 0xC0 ||
        !ImuI2C::read(0x0C, b, sizeof(b)) || !ImuI2C::read(0x20, temp, 2)) return false;
    int16_t g[3], a[3];
    for (int i = 0; i < 3; ++i) {
        g[i] = ImuI2C::little16(b + i * 2); a[i] = ImuI2C::little16(b + 6 + i * 2);
        sample.gyroDps[i] = g[i] / 65.536f; sample.accelG[i] = a[i] / 4096.0f;
    }
    sample.temperatureC = 23.0f + ImuI2C::little16(temp) / 512.0f;
    sample.accelSaturated = ImuI2C::clipped(a[0], a[1], a[2]);
    sample.gyroSaturated = ImuI2C::clipped(g[0], g[1], g[2]);
    return true;
}
