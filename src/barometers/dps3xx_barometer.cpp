#include "dps3xx_barometer.h"
#include "config.h"
#include "barometer_i2c.h"
#include "pressure_math.h"
#include <math.h>

namespace {
constexpr uint8_t REG_PRESS=0x00,REG_TEMP=0x03,REG_PRS_CFG=0x06,REG_TMP_CFG=0x07;
constexpr uint8_t REG_MEAS_CFG=0x08,REG_CFG=0x09,REG_RESET=0x0C,REG_ID=0x0D;
constexpr uint8_t REG_COEF=0x10,REG_COEF_SRC=0x28;
constexpr float PRESSURE_SCALE_16X=253952.0f;
constexpr float TEMPERATURE_SCALE_8X=7864320.0f;
}

DPS3xxBarometer::DPS3xxBarometer()
    : applyTemperatureWorkaround_(VARIO_BAROMETER_TYPE == BAROMETER_DPS310) {}

bool DPS3xxBarometer::writeRegister(uint8_t reg,uint8_t value,bool nonBlocking) {
    return BarometerI2C::writeRegister(reg, value, nonBlocking, runtimeBusMisses_);
}

bool DPS3xxBarometer::readRegisters(uint8_t reg,uint8_t* data,size_t size,bool nonBlocking) {
    return BarometerI2C::readRegisters(reg, data, size, nonBlocking, runtimeBusMisses_);
}

int32_t DPS3xxBarometer::signExtend(uint32_t value,uint8_t bits) {
    const uint32_t sign=1UL<<(bits-1);
    return (int32_t)((value^sign)-sign);
}

bool DPS3xxBarometer::readCalibration(bool nonBlocking) {
    uint8_t b[18];
    diagnostics_.calibrationRead=readRegisters(REG_COEF,b,sizeof(b),nonBlocking);
    if(!diagnostics_.calibrationRead)return false;
    c0_=signExtend(((uint32_t)b[0]<<4)|(b[1]>>4),12);
    c1_=signExtend(((uint32_t)(b[1]&0x0F)<<8)|b[2],12);
    c00_=signExtend(((uint32_t)b[3]<<12)|((uint32_t)b[4]<<4)|(b[5]>>4),20);
    c10_=signExtend(((uint32_t)(b[5]&0x0F)<<16)|((uint32_t)b[6]<<8)|b[7],20);
    c01_=signExtend(((uint32_t)b[8]<<8)|b[9],16);
    c11_=signExtend(((uint32_t)b[10]<<8)|b[11],16);
    c20_=signExtend(((uint32_t)b[12]<<8)|b[13],16);
    c21_=signExtend(((uint32_t)b[14]<<8)|b[15],16);
    c30_=signExtend(((uint32_t)b[16]<<8)|b[17],16);
    diagnostics_.calibrationValid=(c0_!=0||c1_!=0) && (c00_!=0||c10_!=0);
    return diagnostics_.calibrationValid;
}

bool DPS3xxBarometer::begin(){
    startRecovery();
    for(unsigned i=0;i<30;++i){if(serviceRecovery())return true;delay(5);}
    return false;
}

void DPS3xxBarometer::startRecovery(){
    initialized_=newSample_=false;recoveryStep_=0;sampleUs_=0;diagnostics_={};
}

bool DPS3xxBarometer::serviceRecovery(){
    if(initialized_)return true;
    if(recoveryStep_==0){
        uint8_t id=0;
        if(!readRegisters(REG_ID,&id,1,true))return false;
        diagnostics_.chipId=id;
        if(!writeRegister(REG_RESET,0x89,true))return false;
        recoveryUs_=micros();recoveryStep_=1;return false;
    }
    if((uint32_t)(micros()-recoveryUs_)<12000)return false;
    uint8_t ready=0,source=0;
    if(!readRegisters(REG_MEAS_CFG,&ready,1,true)||(ready&0xC0)!=0xC0)return false;
    if(!readRegisters(REG_COEF_SRC,&source,1,true)||!readCalibration(true)){recoveryStep_=0;return false;}
    tempSource_=source&0x80;
    // Infineon's documented DPS310 fuse-bit temperature workaround.
    if(applyTemperatureWorkaround_ &&
       (!writeRegister(0x0E,0xA5,true)||!writeRegister(0x0F,0x96,true)||
        !writeRegister(0x62,0x02,true)||!writeRegister(0x0E,0x00,true)||
        !writeRegister(0x0F,0x00,true))){recoveryStep_=0;return false;}
    // Pressure: 32 Hz / 16x. Temperature: 4 Hz / 8x. This stays within the
    // conversion-time budget and still refreshes pressure at vario speed.
    if(!writeRegister(REG_PRS_CFG,0x54,true)||
       !writeRegister(REG_TMP_CFG,(uint8_t)(tempSource_|0x23),true)||
       !writeRegister(REG_CFG,0x04,true)||
       !writeRegister(REG_MEAS_CFG,0x07,true)){recoveryStep_=0;return false;}
    initialized_=true;return true;
}

void DPS3xxBarometer::update(){
    if(!initialized_)return;
    uint8_t status=0,b[6];
    if(!readRegisters(REG_MEAS_CFG,&status,1,true)||(status&0x10)==0)return;
    if(!readRegisters(REG_PRESS,b,sizeof(b),true))return;
    rawPressure_=signExtend(((uint32_t)b[0]<<16)|((uint32_t)b[1]<<8)|b[2],24);
    rawTemperature_=signExtend(((uint32_t)b[3]<<16)|((uint32_t)b[4]<<8)|b[5],24);
    float pressure=0,temperature=0;
    dps3xxCompensate(rawPressure_,rawTemperature_,PRESSURE_SCALE_16X,TEMPERATURE_SCALE_8X,
                     c0_,c1_,c00_,c10_,c01_,c11_,c20_,c21_,c30_,
                     pressure,temperature);
    if(!isfinite(pressure)||!isfinite(temperature)||pressure<1000||pressure>130000)return;
    pressurePa_=pressure;temperatureC_=temperature;sampleUs_=micros();newSample_=true;
}

float DPS3xxBarometer::pressureToAltitude(float pressurePa,float qnhHpa){
    return pressureToAltitudeM(pressurePa, qnhHpa);
}
