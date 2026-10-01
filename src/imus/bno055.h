#pragma once
#include "imu_hardware.h"
class Bno055Driver {
public:
    bool begin(); void startRecovery(); bool serviceRecovery();
    bool read(ImuHardwareSample& sample);
    static const char* modelName() { return "BNO055"; }
private: uint32_t recoveryMs_ = 0; uint8_t recoveryStep_ = 0;
};
