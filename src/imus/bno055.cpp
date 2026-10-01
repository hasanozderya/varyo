#include "bno055.h"

bool Bno055Driver::begin() {
    if (!I2CBus::ping(I2CAddr::IMU, 0)) return false;
    startRecovery();
    for (int i = 0; i < 180; ++i) { if (serviceRecovery()) return true; delay(5); }
    return false;
}
void Bno055Driver::startRecovery() { recoveryStep_ = 0; }
bool Bno055Driver::serviceRecovery() {
    if (!recoveryStep_) {
        if (!ImuI2C::write(0x3D, 0x00)) return false;
        delay(20);
        if (!ImuI2C::write(0x3F, 0x20)) return false;
        recoveryMs_ = millis(); recoveryStep_ = 1; return false;
    }
    if (recoveryStep_ == 1) {
        if ((uint32_t)(millis() - recoveryMs_) < 700) return false;
        uint8_t id;
        if (!ImuI2C::readByte(0x00, id) || id != 0xA0 ||
            !ImuI2C::write(0x3D, 0x00)) return false;
        delay(20);
        if (!ImuI2C::write(0x07, 0x00) || !ImuI2C::write(0x3E, 0x00) ||
            !ImuI2C::write(0x3B, 0x00) || !ImuI2C::write(0x3F, 0x00) ||
            !ImuI2C::write(0x07, 0x01) || !ImuI2C::write(0x08, 0x0E) ||
            !ImuI2C::write(0x0A, 0x12) || !ImuI2C::write(0x07, 0x00) ||
            !ImuI2C::write(0x3D, 0x07)) return false;
        delay(20); recoveryStep_ = 2;
    }
    return recoveryStep_ == 2;
}
bool Bno055Driver::read(ImuHardwareSample& sample) {
    uint8_t ab[6], gb[6], temp;
    if (!ImuI2C::read(0x08, ab, 6) || !ImuI2C::read(0x14, gb, 6) ||
        !ImuI2C::readByte(0x34, temp)) return false;
    int16_t a[3], g[3];
    for (int i = 0; i < 3; ++i) {
        a[i] = ImuI2C::little16(ab + i * 2); g[i] = ImuI2C::little16(gb + i * 2);
        sample.accelG[i] = a[i] / 980.665f; sample.gyroDps[i] = g[i] / 16.0f;
    }
    sample.temperatureC = (int8_t)temp;
    sample.accelSaturated = ImuI2C::clipped(a[0], a[1], a[2], 7700);
    sample.gyroSaturated = ImuI2C::clipped(g[0], g[1], g[2]);
    return true;
}
