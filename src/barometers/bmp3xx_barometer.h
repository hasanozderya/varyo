#pragma once
#include <Arduino.h>
#include "config.h"

// Bosch BMP388/BMP390 driver using the shared BMP3 compensation model.
class BMP3xxBarometer {
public:
    BMP3xxBarometer();
    struct Diagnostics { uint8_t chipId=0; bool calibrationRead=false,calibrationValid=false; };
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
#if VARIO_BAROMETER_TYPE == BAROMETER_BMP388
        return "BMP388";
#else
        return "BMP390";
#endif
    }
    static float pressureToAltitude(float pressurePa,float qnhHpa);
private:
    bool writeRegister(uint8_t reg,uint8_t value,bool nonBlocking=false);
    bool readRegisters(uint8_t reg,uint8_t* data,size_t size,bool nonBlocking=false);
    bool readCalibration(bool nonBlocking);
    bool initialized_=false,newSample_=false;
    uint8_t recoveryStep_=0;
    uint32_t recoveryUs_=0,sampleUs_=0,runtimeBusMisses_=0;
    uint32_t rawPressure_=0,rawTemperature_=0;
    double t1_=0,t2_=0,t3_=0,p1_=0,p2_=0,p3_=0,p4_=0,p5_=0,p6_=0;
    double p7_=0,p8_=0,p9_=0,p10_=0,p11_=0;
    float pressurePa_=101325.0f,temperatureC_=20.0f;
    const uint8_t expectedChipId_;
    Diagnostics diagnostics_{};
};
