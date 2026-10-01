#pragma once
#include "imu_hardware.h"
class Lsm6dsDriver {
public:
    bool begin(); void startRecovery(); bool serviceRecovery();
    bool read(ImuHardwareSample& sample);
    static const char* modelName() {
#if VARIO_IMU_TYPE == IMU_LSM6DSOX
        return "LSM6DSOX";
#elif VARIO_IMU_TYPE == IMU_LSM6DS3TR
        return "LSM6DS3TR-C";
#else
        return "LSM6DS3";
#endif
    }
private: uint32_t recoveryMs_ = 0; uint8_t recoveryStep_ = 0;
};
