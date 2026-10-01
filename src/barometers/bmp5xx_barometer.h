#pragma once
#include <Arduino.h>
#include "config.h"

// Bosch BMP580/BMP581 driver. These sensors expose factory-compensated data.
class BMP5xxBarometer {
public:
    BMP5xxBarometer();
    struct Diagnostics { uint8_t chipId=0; bool calibrationRead=true,calibrationValid=false; };
    bool begin();
    void startRecovery();
    bool serviceRecovery();
    void update();
    bool hasNewSample() const{return newSample_;}
    void clearNewSample(){newSample_=false;}
    float pressurePa() const{return pressurePa_;}
    float temperatureC() const{return temperatureC_;}
    uint32_t rawD1() const{return rawPressure_;}
    uint32_t rawD2() const{return rawTemperature_;}
    uint32_t filteredD2() const{return rawTemperature_;}
    uint32_t runtimeBusMisses() const{return runtimeBusMisses_;}
    uint32_t sampleTimeUs() const{return sampleUs_;}
    Diagnostics diagnostics() const{return diagnostics_;}
    static constexpr const char* modelName() {
#if VARIO_BAROMETER_TYPE == BAROMETER_BMP580
        return "BMP580";
#else
        return "BMP581";
#endif
    }
    static float pressureToAltitude(float pressurePa,float qnhHpa);
private:
    bool writeRegister(uint8_t reg,uint8_t value,bool nonBlocking=false);
    bool readRegisters(uint8_t reg,uint8_t* data,size_t size,bool nonBlocking=false);
    bool initialized_=false,newSample_=false;
    uint8_t recoveryStep_=0;
    uint32_t recoveryUs_=0,lastReadUs_=0,sampleUs_=0,runtimeBusMisses_=0;
    uint32_t rawPressure_=0,rawTemperature_=0;
    float pressurePa_=101325.0f,temperatureC_=20.0f;
    const uint8_t expectedChipId_;
    Diagnostics diagnostics_{};
};
