#pragma once
#include "imu_hardware.h"
class Bmi270Driver {
public:
    bool begin(); void startRecovery(); bool serviceRecovery();
    bool read(ImuHardwareSample& sample);
    static const char* modelName() { return "BMI270"; }
private: uint8_t recoveryStep_ = 0;
};
