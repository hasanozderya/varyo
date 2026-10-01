#include "bmi270.h"
extern "C" {
#include <bmi270_api/bmi270.h>
}

namespace {
struct bmi2_dev device{};
uint8_t address = I2CAddr::IMU;
BMI2_INTF_RETURN_TYPE readBus(uint8_t reg, uint8_t* data, uint32_t length, void*) {
    return ImuI2C::read(reg, data, length, Timing::I2C_MUTEX_TIMEOUT)
        ? BMI2_INTF_RET_SUCCESS : -1;
}
BMI2_INTF_RETURN_TYPE writeBus(uint8_t reg, const uint8_t* data, uint32_t length, void*) {
    I2CLockGuard lock(Timing::I2C_MUTEX_TIMEOUT);
    if (!lock.ok()) return -1;
    Wire.beginTransmission(I2CAddr::IMU); Wire.write(reg);
    for (uint32_t i = 0; i < length; ++i) Wire.write(data[i]);
    return Wire.endTransmission() == 0 ? BMI2_INTF_RET_SUCCESS : -1;
}
void delayUs(uint32_t period, void*) {
    if (period >= 1000) delay(period / 1000);
    if (period % 1000) delayMicroseconds(period % 1000);
}
}

bool Bmi270Driver::begin() {
    if (!I2CBus::ping(I2CAddr::IMU, 0)) return false;
    startRecovery();
    return serviceRecovery();
}
void Bmi270Driver::startRecovery() { recoveryStep_ = 0; }
bool Bmi270Driver::serviceRecovery() {
    if (recoveryStep_ == 1) return true;
    memset(&device, 0, sizeof(device)); address = I2CAddr::IMU;
    device.intf = BMI2_I2C_INTF; device.intf_ptr = &address;
    device.read = readBus; device.write = writeBus; device.delay_us = delayUs;
    device.read_write_len = 32;
    if (bmi270_init(&device) != BMI2_OK) return false;
    struct bmi2_sens_config cfg[2]{};
    cfg[0].type = BMI2_ACCEL; cfg[1].type = BMI2_GYRO;
    if (bmi2_get_sensor_config(cfg, 2, &device) != BMI2_OK) return false;
    cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    cfg[0].cfg.acc.range = BMI2_ACC_RANGE_8G;
    cfg[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_500;
    cfg[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    cfg[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    if (bmi2_set_sensor_config(cfg, 2, &device) != BMI2_OK) return false;
    const uint8_t sensors[2] = {BMI2_ACCEL, BMI2_GYRO};
    if (bmi270_sensor_enable(sensors, 2, &device) != BMI2_OK) return false;
    recoveryStep_ = 1;
    return true;
}
bool Bmi270Driver::read(ImuHardwareSample& sample) {
    uint8_t status;
    if (!ImuI2C::readByte(0x03, status) || (status & 0xC0) != 0xC0) return false;
    struct bmi2_sens_data data{}; uint16_t temperature = 0;
    if (bmi2_get_sensor_data(&data, &device) != BMI2_OK ||
        bmi2_get_temperature_data(&temperature, &device) != BMI2_OK) return false;
    const int16_t a[3] = {data.acc.x, data.acc.y, data.acc.z};
    const int16_t g[3] = {data.gyr.x, data.gyr.y, data.gyr.z};
    for (int i = 0; i < 3; ++i) {
        sample.accelG[i] = a[i] / 4096.0f; sample.gyroDps[i] = g[i] / 65.536f;
    }
    sample.temperatureC = temperature == 0x8000 ? 23.0f :
        23.0f + (int16_t)temperature / 512.0f;
    sample.accelSaturated = ImuI2C::clipped(a[0], a[1], a[2]);
    sample.gyroSaturated = ImuI2C::clipped(g[0], g[1], g[2]);
    return true;
}
