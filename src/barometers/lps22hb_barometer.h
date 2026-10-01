#pragma once
#include <Arduino.h>

class LPS22HBBarometer {
public:
    struct Diagnostics { uint8_t chipId=0; bool calibrationRead=true, calibrationValid=false; };
    bool begin();
    void startRecovery();
    bool serviceRecovery();
    void update();
    bool hasNewSample() const { return newSample_; }
    void clearNewSample() { newSample_=false; }
    float pressurePa() const { return pressurePa_; }
    float temperatureC() const { return temperatureC_; }
    uint32_t rawD1() const { return rawPressure_ & 0xFFFFFF; }
    uint32_t rawD2() const { return (uint16_t)rawTemperature_; }
    uint32_t filteredD2() const { return (uint16_t)rawTemperature_; }
    uint32_t runtimeBusMisses() const { return runtimeBusMisses_; }
    uint32_t sampleTimeUs() const { return sampleUs_; }
    Diagnostics diagnostics() const { return diagnostics_; }
    static constexpr const char* modelName() { return "LPS22HB"; }
    static float pressureToAltitude(float pressurePa,float qnhHpa);
private:
    bool writeRegister(uint8_t reg,uint8_t value,bool nonBlocking=false);
    bool readRegister(uint8_t reg,uint8_t& value,bool nonBlocking=false);
    bool initialized_=false,newSample_=false;
    uint32_t sampleUs_=0,runtimeBusMisses_=0;
    int32_t rawPressure_=0;
    int16_t rawTemperature_=0;
    float pressurePa_=101325.0f,temperatureC_=20.0f;
    Diagnostics diagnostics_{};
};
