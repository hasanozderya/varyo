#include "bmp5xx_barometer.h"
#include "config.h"
#include "barometer_i2c.h"
#include "pressure_math.h"
#include <math.h>

namespace {
constexpr uint8_t REG_ID=0x01,REG_DATA=0x1D,REG_STATUS=0x28;
constexpr uint8_t REG_OSR=0x36,REG_ODR=0x37,REG_CMD=0x7E;
constexpr uint32_t SAMPLE_PERIOD_US=25000;
}

BMP5xxBarometer::BMP5xxBarometer()
    : expectedChipId_(VARIO_BAROMETER_TYPE == BAROMETER_BMP580 ? 0x50 : 0x51) {}

bool BMP5xxBarometer::writeRegister(uint8_t reg,uint8_t value,bool nonBlocking){
    return BarometerI2C::writeRegister(reg, value, nonBlocking, runtimeBusMisses_);
}

bool BMP5xxBarometer::readRegisters(uint8_t reg,uint8_t* data,size_t size,bool nonBlocking){
    return BarometerI2C::readRegisters(reg, data, size, nonBlocking, runtimeBusMisses_);
}

bool BMP5xxBarometer::begin(){
    startRecovery();
    for(unsigned i=0;i<30;++i){if(serviceRecovery())return true;delay(5);}
    return false;
}

void BMP5xxBarometer::startRecovery(){
    initialized_=newSample_=false;recoveryStep_=0;sampleUs_=lastReadUs_=0;diagnostics_={};
}

bool BMP5xxBarometer::serviceRecovery(){
    if(initialized_)return true;
    if(recoveryStep_==0){
        uint8_t id=0;
        if(!readRegisters(REG_ID,&id,1,true))return false;
        if(id!=expectedChipId_)return false;
        diagnostics_.chipId=id;
        if(!writeRegister(REG_CMD,0xB6,true))return false;
        recoveryUs_=micros();recoveryStep_=1;return false;
    }
    if((uint32_t)(micros()-recoveryUs_)<5000)return false;
    if(recoveryStep_==1){
        uint8_t status=0;
        if(!readRegisters(REG_STATUS,&status,1,true)||(status&0x02)==0||(status&0x04)!=0)return false;
        // Enter standby first. Configuration writes made outside standby may be ignored.
        if(!writeRegister(REG_ODR,0xBC,true)){recoveryStep_=0;return false;}
        recoveryUs_=micros();recoveryStep_=2;return false;
    }
    if((uint32_t)(micros()-recoveryUs_)<3000)return false;
    // Temperature 2x, pressure 8x, pressure enabled, 50 Hz, normal mode.
    // deep_dis=1 prevents the standby value from being interpreted as deep standby.
    if(!writeRegister(REG_OSR,0x59,true)||!writeRegister(REG_ODR,0xBD,true)){
        recoveryStep_=0;return false;
    }
    diagnostics_.calibrationValid=initialized_=true;lastReadUs_=micros();return true;
}

void BMP5xxBarometer::update(){
    if(!initialized_)return;
    const uint32_t now=micros();
    if((uint32_t)(now-lastReadUs_)<SAMPLE_PERIOD_US)return;
    lastReadUs_=now;
    uint8_t b[6];
    if(!readRegisters(REG_DATA,b,sizeof(b),true))return;
    rawTemperature_=(uint32_t)b[0]|((uint32_t)b[1]<<8)|((uint32_t)b[2]<<16);
    rawPressure_=(uint32_t)b[3]|((uint32_t)b[4]<<8)|((uint32_t)b[5]<<16);
    const float temperature=bmp5CompensatedTemperatureC(rawTemperature_);
    const float pressure=bmp5CompensatedPressurePa(rawPressure_);
    if(!isfinite(pressure)||!isfinite(temperature)||pressure<1000||pressure>130000)return;
    pressurePa_=pressure;temperatureC_=temperature;sampleUs_=micros();newSample_=true;
}

float BMP5xxBarometer::pressureToAltitude(float pressurePa,float qnhHpa){
    return pressureToAltitudeM(pressurePa, qnhHpa);
}
