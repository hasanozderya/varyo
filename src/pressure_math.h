#pragma once
#include <stdint.h>
#include <math.h>

inline float pressureToAltitudeM(float pressurePa, float qnhHpa) {
    if (!isfinite(pressurePa) || !isfinite(qnhHpa) || pressurePa <= 0.0f ||
        qnhHpa <= 0.0f) {
        return NAN;
    }
    return 44330.0f *
        (1.0f - powf(pressurePa / (qnhHpa * 100.0f), 0.190295f));
}

// MS5607 compensated pressure, Pa. Keep the fractional Pascal until the
// final float store; truncating the final division creates ~8 cm altitude
// steps near sea level even when the raw ADC changes smoothly.
inline float ms5607CompensatedPressurePa(uint32_t d1, int64_t sens, int64_t off) {
    return (float)(((double)d1 * (double)sens / 2097152.0 - (double)off) / 32768.0);
}

inline float bmp280CompensatedTemperatureC(uint32_t adcT, uint16_t t1,
                                           int16_t t2, int16_t t3,
                                           int32_t& fine) {
    const int32_t v1=((((int32_t)(adcT>>3)-((int32_t)t1<<1))*(int32_t)t2)>>11);
    const int32_t delta=(int32_t)(adcT>>4)-(int32_t)t1;
    const int32_t v2=(int32_t)(((((int64_t)delta*delta)>>12)*(int32_t)t3)>>14);
    fine=v1+v2;
    return ((fine*5+128)>>8)/100.0f;
}

inline float bmp280CompensatedPressurePa(uint32_t adcP, int32_t fine,
                                         uint16_t p1, int16_t p2, int16_t p3,
                                         int16_t p4, int16_t p5, int16_t p6,
                                         int16_t p7, int16_t p8, int16_t p9) {
    int64_t v1=(int64_t)fine-128000;
    int64_t v2=v1*v1*(int64_t)p6;
    v2+=v1*(int64_t)p5*131072LL;
    v2+=(int64_t)p4*34359738368LL;
    v1=((v1*v1*(int64_t)p3)>>8)+v1*(int64_t)p2*4096LL;
    v1=(((((int64_t)1)<<47)+v1)*(int64_t)p1)>>33;
    if(v1==0) return NAN;
    int64_t pressure=1048576-(int64_t)adcP;
    pressure=(((pressure<<31)-v2)*3125)/v1;
    v1=((int64_t)p9*(pressure>>13)*(pressure>>13))>>25;
    v2=((int64_t)p8*pressure)>>19;
    pressure=((pressure+v1+v2)>>8)+(int64_t)p7*16LL;
    return (float)pressure/256.0f;
}

inline int32_t signExtend24(uint32_t value) {
    value &= 0xFFFFFFU;
    return (value & 0x800000U) ? (int32_t)(value | 0xFF000000U) : (int32_t)value;
}

inline float bmp5CompensatedTemperatureC(uint32_t rawTemperature) {
    return (float)signExtend24(rawTemperature) / 65536.0f;
}

inline float bmp5CompensatedPressurePa(uint32_t rawPressure) {
    return (float)(rawPressure & 0xFFFFFFU) / 64.0f;
}

inline void dps3xxCompensate(int32_t rawPressure, int32_t rawTemperature,
                             float pressureScale, float temperatureScale,
                             int32_t c0, int32_t c1, int32_t c00, int32_t c10,
                             int32_t c01, int32_t c11, int32_t c20,
                             int32_t c21, int32_t c30,
                             float& pressurePa, float& temperatureC) {
    const float t=(float)rawTemperature/temperatureScale;
    const float p=(float)rawPressure/pressureScale;
    temperatureC=0.5f*(float)c0+(float)c1*t;
    pressurePa=(float)c00+p*((float)c10+p*((float)c20+p*(float)c30))+
               t*(float)c01+t*p*((float)c11+p*(float)c21);
}
