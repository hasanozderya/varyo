#include "bmp28x_barometer.h"
#include "config.h"
#include "barometer_i2c.h"
#include "pressure_math.h"
#include <math.h>

namespace {
    constexpr uint8_t REG_ID=0xD0, REG_RESET=0xE0, REG_CALIB=0x88;
    constexpr uint8_t REG_CONFIG=0xF5, REG_CTRL=0xF4, REG_DATA=0xF7;
    constexpr uint8_t RESET_CODE=0xB6;
    constexpr uint32_t SAMPLE_PERIOD_US=30000;
    uint16_t u16(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1]<<8); }
    int16_t s16(const uint8_t* p) { return (int16_t)u16(p); }
}

BMP28xBarometer::BMP28xBarometer()
    : expectedChipId_(VARIO_BAROMETER_TYPE == BAROMETER_BME280 ? 0x60 : 0x58) {}

bool BMP28xBarometer::writeRegister(uint8_t reg, uint8_t value, bool nonBlocking) {
    return BarometerI2C::writeRegister(reg, value, nonBlocking, runtimeBusMisses_);
}

bool BMP28xBarometer::readRegisters(uint8_t reg, uint8_t* data, size_t size, bool nonBlocking) {
    return BarometerI2C::readRegisters(reg, data, size, nonBlocking, runtimeBusMisses_);
}

bool BMP28xBarometer::readCalibration(bool nonBlocking) {
    uint8_t c[24];
    diagnostics_.calibrationRead=readRegisters(REG_CALIB,c,sizeof(c),nonBlocking);
    if (!diagnostics_.calibrationRead) return false;
    t1_=u16(c); t2_=s16(c+2); t3_=s16(c+4);
    p1_=u16(c+6); p2_=s16(c+8); p3_=s16(c+10); p4_=s16(c+12);
    p5_=s16(c+14); p6_=s16(c+16); p7_=s16(c+18); p8_=s16(c+20); p9_=s16(c+22);
    diagnostics_.calibrationValid=t1_!=0 && t1_!=0xFFFF && p1_!=0 && p1_!=0xFFFF;
    return diagnostics_.calibrationValid;
}

bool BMP28xBarometer::begin() {
    startRecovery();
    for (unsigned i=0;i<20;++i) { if (serviceRecovery()) return true; delay(5); }
    return false;
}

void BMP28xBarometer::startRecovery() {
    initialized_=newSample_=false; recoveryStep_=0; lastReadUs_=sampleUs_=0;
    diagnostics_={};
}

bool BMP28xBarometer::serviceRecovery() {
    if (initialized_) return true;
    if (recoveryStep_==0) {
        uint8_t id=0;
        if (!readRegisters(REG_ID,&id,1,true)) return false;
        if (id != expectedChipId_) return false;
        diagnostics_.chipId=id;
        if (!writeRegister(REG_RESET,RESET_CODE,true)) return false;
        recoveryUs_=micros(); recoveryStep_=1; return false;
    }
    if ((uint32_t)(micros()-recoveryUs_)<5000) return false;
    if (!readCalibration(true)) { recoveryStep_=0; return false; }
    // Normal mode, temperature x2, pressure x8, internal IIR off. The shared
    // pressure/output filters remain the single source of smoothing.
    if (!writeRegister(REG_CONFIG,0x00,true) || !writeRegister(REG_CTRL,0x53,true)) {
        recoveryStep_=0; return false;
    }
    initialized_=true; lastReadUs_=micros(); return true;
}

bool BMP28xBarometer::readMeasurement() {
    uint8_t data[6];
    if (!readRegisters(REG_DATA,data,sizeof(data),true)) return false;
    rawPressure_=((uint32_t)data[0]<<12)|((uint32_t)data[1]<<4)|(data[2]>>4);
    rawTemperature_=((uint32_t)data[3]<<12)|((uint32_t)data[4]<<4)|(data[5]>>4);
    int32_t fine=0;
    const float temp=bmp280CompensatedTemperatureC(rawTemperature_,t1_,t2_,t3_,fine);
    const float pressure=bmp280CompensatedPressurePa(rawPressure_,fine,p1_,p2_,p3_,p4_,p5_,p6_,p7_,p8_,p9_);
    if (!isfinite(temp)||!isfinite(pressure)||pressure<1000||pressure>130000) return false;
    temperatureC_=temp; pressurePa_=pressure; sampleUs_=micros(); newSample_=true; return true;
}

void BMP28xBarometer::update() {
    if (!initialized_) return;
    const uint32_t now=micros();
    if ((uint32_t)(now-lastReadUs_)<SAMPLE_PERIOD_US) return;
    lastReadUs_=now;
    readMeasurement();
}

float BMP28xBarometer::pressureToAltitude(float pressurePa,float qnhHpa) {
    return pressureToAltitudeM(pressurePa, qnhHpa);
}
