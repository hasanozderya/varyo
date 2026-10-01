#pragma once

#include <Arduino.h>

// Select the fitted IMU. Override VARIO_IMU_TYPE and VARIO_IMU_ADDRESS from
// PlatformIO build flags when maintaining multiple hardware variants.
#define IMU_MPU6050    1
#define IMU_LSM6DS3    2
#define IMU_LSM6DS3TR  3
#define IMU_LSM6DSOX   4
#define IMU_BMI160     5
#define IMU_BMI270     6
#define IMU_BNO055     7

#ifndef VARIO_IMU_TYPE
#define VARIO_IMU_TYPE IMU_MPU6050
#endif

#ifndef VARIO_IMU_ADDRESS
#if VARIO_IMU_TYPE == IMU_LSM6DS3 || VARIO_IMU_TYPE == IMU_LSM6DS3TR || \
    VARIO_IMU_TYPE == IMU_LSM6DSOX
#define VARIO_IMU_ADDRESS 0x6A
#elif VARIO_IMU_TYPE == IMU_BNO055
#define VARIO_IMU_ADDRESS 0x28
#else
#define VARIO_IMU_ADDRESS 0x68
#endif
#endif

// Select the fitted barometer. Only one driver is compiled into a firmware
// variant. Alternate I2C addresses can be supplied with a build flag.
#define BAROMETER_MS5607   1
#define BAROMETER_MS5611   2
#define BAROMETER_BMP280   3
#define BAROMETER_LPS22HB  4
#define BAROMETER_DPS310   5
#define BAROMETER_HP303B   6
#define BAROMETER_BMP388   7
#define BAROMETER_BMP390   8
#define BAROMETER_BMP580   9
#define BAROMETER_BMP581  10
#define BAROMETER_BME280  11

#ifndef VARIO_BAROMETER_TYPE
#define VARIO_BAROMETER_TYPE BAROMETER_MS5607
#endif

#ifndef VARIO_BAROMETER_ADDRESS
#if VARIO_BAROMETER_TYPE == BAROMETER_BMP280 || \
    VARIO_BAROMETER_TYPE == BAROMETER_BME280 || \
    VARIO_BAROMETER_TYPE == BAROMETER_BMP388 || \
    VARIO_BAROMETER_TYPE == BAROMETER_BMP390
#define VARIO_BAROMETER_ADDRESS 0x76
#elif VARIO_BAROMETER_TYPE == BAROMETER_LPS22HB
#define VARIO_BAROMETER_ADDRESS 0x5C
#elif VARIO_BAROMETER_TYPE == BAROMETER_DPS310 || \
      VARIO_BAROMETER_TYPE == BAROMETER_HP303B
#define VARIO_BAROMETER_ADDRESS 0x77
#elif VARIO_BAROMETER_TYPE == BAROMETER_BMP580 || \
      VARIO_BAROMETER_TYPE == BAROMETER_BMP581
#define VARIO_BAROMETER_ADDRESS 0x47
#else
#define VARIO_BAROMETER_ADDRESS 0x77
#endif
#endif

#if VARIO_BAROMETER_TYPE < BAROMETER_MS5607 || \
    VARIO_BAROMETER_TYPE > BAROMETER_BME280
#error "Unsupported VARIO_BAROMETER_TYPE"
#endif

#if (VARIO_BAROMETER_TYPE == BAROMETER_LPS22HB) && \
    (VARIO_BAROMETER_ADDRESS != 0x5C && VARIO_BAROMETER_ADDRESS != 0x5D)
#error "LPS22HB address must be 0x5C or 0x5D"
#elif (VARIO_BAROMETER_TYPE == BAROMETER_BMP580 || \
       VARIO_BAROMETER_TYPE == BAROMETER_BMP581) && \
      (VARIO_BAROMETER_ADDRESS != 0x46 && VARIO_BAROMETER_ADDRESS != 0x47)
#error "BMP580/BMP581 address must be 0x46 or 0x47"
#elif (VARIO_BAROMETER_TYPE != BAROMETER_LPS22HB && \
       VARIO_BAROMETER_TYPE != BAROMETER_BMP580 && \
       VARIO_BAROMETER_TYPE != BAROMETER_BMP581) && \
      (VARIO_BAROMETER_ADDRESS != 0x76 && VARIO_BAROMETER_ADDRESS != 0x77)
#error "Selected barometer address must be 0x76 or 0x77"
#endif

namespace Pins {
constexpr int I2C_SDA = 8;
constexpr int I2C_SCL = 9;
constexpr int GPS_RX = 2;
constexpr int GPS_TX = 1;
constexpr int DISPLAY_CS = 10;
constexpr int DISPLAY_DC = 11;
constexpr int DISPLAY_RST = 12;
constexpr int DISPLAY_BL = 13;
constexpr int SPI_SCK = 14;
constexpr int SPI_MOSI = 15;
constexpr int SPI_MISO = 16;
constexpr int AUDIO_DATA = 4;
constexpr int AUDIO_BCLK = 5;
constexpr int AUDIO_WS = 6;
} // namespace Pins

namespace I2CAddr {
constexpr uint8_t BAROMETER = VARIO_BAROMETER_ADDRESS;
constexpr uint8_t IMU = VARIO_IMU_ADDRESS;
constexpr uint8_t OLED = 0x3C;
} // namespace I2CAddr
