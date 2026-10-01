#pragma once
#include <Arduino.h>
#include "config.h"

// DPS310 and register-compatible HP303B pressure driver.
class DPS3xxBarometer {
public:
    DPS3xxBarometer();
    struct Diagnostics {
        uint8_t chipId=0;
        bool calibrationRead=false;
        bool calibrationValid=false;
    };
    bool begin();
    void startRecovery();
    bool serviceRecovery();
    void update();
    bool hasNewSample() const { return newSample_; }
    void clearNewSample() { newSample_=false; }
    float pressurePa() const { return pressurePa_; }
    float temperatureC() const { return temperatureC_; }
    uint32_t rawD1() const { return (uint32_t)rawPressure_ & 0xFFFFFFU; }
    uint32_t rawD2() const { return (uint32_t)rawTemperature_ & 0xFFFFFFU; }
    uint32_t filteredD2() const { return rawD2(); }
    uint32_t runtimeBusMisses() const { return runtimeBusMisses_; }
    uint32_t sampleTimeUs() const { return sampleUs_; }
    Diagnostics diagnostics() const { return diagnostics_; }
    static constexpr const char* modelName() {
#if VARIO_BAROMETER_TYPE == BAROMETER_HP303B
        return "HP303B";
#else
        return "DPS310";
#endif
    }
    static float pressureToAltitude(float pressurePa,float qnhHpa);
private:
    bool writeRegister(uint8_t reg,uint8_t value,bool nonBlocking=false);
    bool readRegisters(uint8_t reg,uint8_t* data,size_t size,bool nonBlocking=false);
    bool readCalibration(bool nonBlocking);
    static int32_t signExtend(uint32_t value,uint8_t bits);
    int32_t c0_=0,c1_=0,c00_=0,c10_=0,c01_=0,c11_=0,c20_=0,c21_=0,c30_=0;
    bool initialized_=false,newSample_=false;
    uint8_t recoveryStep_=0,tempSource_=0;
    uint32_t recoveryUs_=0,sampleUs_=0,runtimeBusMisses_=0;
    int32_t rawPressure_=0,rawTemperature_=0;
    float pressurePa_=101325.0f,temperatureC_=20.0f;
    const bool applyTemperatureWorkaround_;
    Diagnostics diagnostics_{};
};
