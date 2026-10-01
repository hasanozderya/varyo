#include "lps22hb_barometer.h"
#include "config.h"
#include "barometer_i2c.h"
#include "pressure_math.h"
#include <math.h>

namespace {
    constexpr uint8_t REG_WHO_AM_I=0x0F, REG_CTRL1=0x10, REG_CTRL2=0x11;
    constexpr uint8_t REG_STATUS=0x27, REG_PRESS_XL=0x28, REG_TEMP_L=0x2B;
}

bool LPS22HBBarometer::writeRegister(uint8_t reg,uint8_t value,bool nonBlocking) {
    return BarometerI2C::writeRegister(reg, value, nonBlocking, runtimeBusMisses_);
}

bool LPS22HBBarometer::readRegister(uint8_t reg,uint8_t& value,bool nonBlocking) {
    return BarometerI2C::readRegisters(reg, &value, 1, nonBlocking, runtimeBusMisses_);
}

bool LPS22HBBarometer::begin() {
    startRecovery();
    for(unsigned i=0;i<5;++i) { if(serviceRecovery()) return true; delay(2); }
    return false;
}

void LPS22HBBarometer::startRecovery() {
    initialized_=newSample_=false; diagnostics_={}; sampleUs_=0;
}

bool LPS22HBBarometer::serviceRecovery() {
    if(initialized_) return true;
    uint8_t id=0;
    if(!readRegister(REG_WHO_AM_I,id,true)||id!=0xB1) return false;
    diagnostics_.chipId=id;
    // 75 Hz continuous mode, internal LPF off, BDU on. CTRL2 stays at zero;
    // datasheet requires single-byte output reads with this BDU/I2C setup.
    if(!writeRegister(REG_CTRL2,0x00,true)||!writeRegister(REG_CTRL1,0x52,true)) return false;
    diagnostics_.calibrationValid=initialized_=true;
    return true;
}

void LPS22HBBarometer::update() {
    if(!initialized_) return;
    uint8_t status=0;
    if(!readRegister(REG_STATUS,status,true)||(status&0x01)==0) return;
    uint8_t b[5];
    for(unsigned i=0;i<3;++i) if(!readRegister(REG_PRESS_XL+i,b[i],true)) return;
    for(unsigned i=0;i<2;++i) if(!readRegister(REG_TEMP_L+i,b[3+i],true)) return;
    rawPressure_=(int32_t)((uint32_t)b[0]|((uint32_t)b[1]<<8)|((uint32_t)b[2]<<16));
    if(rawPressure_&0x800000) rawPressure_|=~0xFFFFFF;
    rawTemperature_=(int16_t)((uint16_t)b[3]|((uint16_t)b[4]<<8));
    const float pressure=(float)rawPressure_*(100.0f/4096.0f);
    const float temperature=(float)rawTemperature_/100.0f;
    if(!isfinite(pressure)||pressure<1000||pressure>130000) return;
    pressurePa_=pressure; temperatureC_=temperature;
    sampleUs_=micros(); newSample_=true;
}

float LPS22HBBarometer::pressureToAltitude(float pressurePa,float qnhHpa) {
    return pressureToAltitudeM(pressurePa, qnhHpa);
}
