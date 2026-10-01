#include "bmp3xx_barometer.h"
#include "config.h"
#include "barometer_i2c.h"
#include "pressure_math.h"
#include <math.h>

namespace {
constexpr uint8_t REG_ID=0x00,REG_STATUS=0x03,REG_DATA=0x04,REG_PWR=0x1B;
constexpr uint8_t REG_OSR=0x1C,REG_ODR=0x1D,REG_CONFIG=0x1F,REG_CALIB=0x31,REG_CMD=0x7E;
uint16_t u16(const uint8_t* p){return (uint16_t)p[0]|((uint16_t)p[1]<<8);}
int16_t s16(const uint8_t* p){return (int16_t)u16(p);}
}

BMP3xxBarometer::BMP3xxBarometer()
    : expectedChipId_(VARIO_BAROMETER_TYPE == BAROMETER_BMP388 ? 0x50 : 0x60) {}

bool BMP3xxBarometer::writeRegister(uint8_t reg,uint8_t value,bool nonBlocking){
    return BarometerI2C::writeRegister(reg, value, nonBlocking, runtimeBusMisses_);
}

bool BMP3xxBarometer::readRegisters(uint8_t reg,uint8_t* data,size_t size,bool nonBlocking){
    return BarometerI2C::readRegisters(reg, data, size, nonBlocking, runtimeBusMisses_);
}

bool BMP3xxBarometer::readCalibration(bool nonBlocking){
    uint8_t b[21];
    diagnostics_.calibrationRead=readRegisters(REG_CALIB,b,sizeof(b),nonBlocking);
    if(!diagnostics_.calibrationRead)return false;
    t1_=(double)u16(b)*256.0;
    t2_=(double)u16(b+2)/1073741824.0;
    t3_=(double)(int8_t)b[4]/281474976710656.0;
    p1_=((double)s16(b+5)-16384.0)/1048576.0;
    p2_=((double)s16(b+7)-16384.0)/536870912.0;
    p3_=(double)(int8_t)b[9]/4294967296.0;
    p4_=(double)(int8_t)b[10]/137438953472.0;
    p5_=(double)u16(b+11)*8.0;
    p6_=(double)u16(b+13)/64.0;
    p7_=(double)(int8_t)b[15]/256.0;
    p8_=(double)(int8_t)b[16]/32768.0;
    p9_=(double)s16(b+17)/281474976710656.0;
    p10_=(double)(int8_t)b[19]/281474976710656.0;
    p11_=(double)(int8_t)b[20]/36893488147419103232.0;
    diagnostics_.calibrationValid=u16(b)!=0&&u16(b)!=0xFFFF&&u16(b+11)!=0;
    return diagnostics_.calibrationValid;
}

bool BMP3xxBarometer::begin(){
    startRecovery();
    for(unsigned i=0;i<30;++i){if(serviceRecovery())return true;delay(5);}
    return false;
}

void BMP3xxBarometer::startRecovery(){
    initialized_=newSample_=false;recoveryStep_=0;sampleUs_=0;diagnostics_={};
}

bool BMP3xxBarometer::serviceRecovery(){
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
    if(!readCalibration(true)){recoveryStep_=0;return false;}
    // Pressure 8x, temperature 2x, 50 Hz, internal IIR bypassed, normal mode.
    if(!writeRegister(REG_OSR,0x0B,true)||!writeRegister(REG_ODR,0x02,true)||
       !writeRegister(REG_CONFIG,0x00,true)||!writeRegister(REG_PWR,0x33,true)){
        recoveryStep_=0;return false;
    }
    initialized_=true;return true;
}

void BMP3xxBarometer::update(){
    if(!initialized_)return;
    uint8_t status=0,b[6];
    if(!readRegisters(REG_STATUS,&status,1,true)||(status&0x60)!=0x60)return;
    if(!readRegisters(REG_DATA,b,sizeof(b),true))return;
    rawPressure_=(uint32_t)b[0]|((uint32_t)b[1]<<8)|((uint32_t)b[2]<<16);
    rawTemperature_=(uint32_t)b[3]|((uint32_t)b[4]<<8)|((uint32_t)b[5]<<16);
    const double dt=(double)rawTemperature_-t1_;
    const double tlin=dt*t2_+dt*dt*t3_;
    const double t2=tlin*tlin,t3=t2*tlin;
    const double po1=p5_+p6_*tlin+p7_*t2+p8_*t3;
    const double po2=(double)rawPressure_*(p1_+p2_*tlin+p3_*t2+p4_*t3);
    const double rp=(double)rawPressure_;
    const double pressure=po1+po2+rp*rp*(p9_+p10_*tlin)+rp*rp*rp*p11_;
    if(!isfinite(pressure)||!isfinite(tlin)||pressure<1000||pressure>130000)return;
    pressurePa_=(float)pressure;temperatureC_=(float)tlin;sampleUs_=micros();newSample_=true;
}

float BMP3xxBarometer::pressureToAltitude(float pressurePa,float qnhHpa){
    return pressureToAltitudeM(pressurePa, qnhHpa);
}
