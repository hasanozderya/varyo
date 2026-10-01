#pragma once
#include "imu_hardware.h"

class Mpu6050Driver {
public:
    bool begin();
    void startRecovery();
    bool serviceRecovery();
    bool read(ImuHardwareSample& sample);
    static const char* modelName() { return "MPU6050"; }
private:
    uint32_t recoveryMs_ = 0;
    uint8_t recoveryStep_ = 0;
};
