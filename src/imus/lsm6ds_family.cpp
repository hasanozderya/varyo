#include "lsm6ds_family.h"

namespace {
constexpr uint8_t EXPECTED_CHIP_ID =
    VARIO_IMU_TYPE == IMU_LSM6DSOX ? 0x6C : 0x69;
constexpr float TEMPERATURE_DIVISOR =
    VARIO_IMU_TYPE == IMU_LSM6DSOX ? 256.0f : 16.0f;
}

bool Lsm6dsDriver::begin() {
    if (!I2CBus::ping(I2CAddr::IMU, 0)) return false;
    startRecovery();
    for (int i = 0; i < 30; ++i) { if (serviceRecovery()) return true; delay(5); }
    return false;
}
void Lsm6dsDriver::startRecovery() { recoveryStep_ = 0; }
bool Lsm6dsDriver::serviceRecovery() {
    if (!recoveryStep_) {
        if (!ImuI2C::write(0x12, 0x01)) return false;
        recoveryMs_ = millis(); recoveryStep_ = 1; return false;
    }
    if (recoveryStep_ == 1) {
        if ((uint32_t)(millis() - recoveryMs_) < 20) return false;
        uint8_t id;
        if (!ImuI2C::readByte(0x0F, id) || id != EXPECTED_CHIP_ID ||
            !ImuI2C::write(0x12, 0x44) || !ImuI2C::write(0x10, 0x4C) ||
            !ImuI2C::write(0x11, 0x44)) return false;
        recoveryStep_ = 2;
    }
    return true;
}
bool Lsm6dsDriver::read(ImuHardwareSample& sample) {
    uint8_t status, b[14];
    if (!ImuI2C::readByte(0x1E, status) || (status & 3) != 3 ||
        !ImuI2C::read(0x20, b, sizeof(b))) return false;
    const int16_t t = ImuI2C::little16(b);
    int16_t g[3], a[3];
    for (int i = 0; i < 3; ++i) {
        g[i] = ImuI2C::little16(b + 2 + i * 2); a[i] = ImuI2C::little16(b + 8 + i * 2);
        sample.gyroDps[i] = g[i] * 0.0175f; sample.accelG[i] = a[i] * 0.000244f;
    }
    sample.temperatureC = 25.0f + t / TEMPERATURE_DIVISOR;
    sample.accelSaturated = ImuI2C::clipped(a[0], a[1], a[2]);
    sample.gyroSaturated = ImuI2C::clipped(g[0], g[1], g[2]);
    return true;
}
