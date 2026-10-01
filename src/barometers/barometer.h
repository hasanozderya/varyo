#pragma once
#include "config.h"

#if VARIO_BAROMETER_TYPE == BAROMETER_MS5607 || \
    VARIO_BAROMETER_TYPE == BAROMETER_MS5611
#include "ms56xx_barometer.h"
using Barometer = MS56xxBarometer;
#elif VARIO_BAROMETER_TYPE == BAROMETER_BMP280 || \
      VARIO_BAROMETER_TYPE == BAROMETER_BME280
#include "bmp28x_barometer.h"
using Barometer = BMP28xBarometer;
#elif VARIO_BAROMETER_TYPE == BAROMETER_LPS22HB
#include "lps22hb_barometer.h"
using Barometer = LPS22HBBarometer;
#elif VARIO_BAROMETER_TYPE == BAROMETER_DPS310 || \
      VARIO_BAROMETER_TYPE == BAROMETER_HP303B
#include "dps3xx_barometer.h"
using Barometer = DPS3xxBarometer;
#elif VARIO_BAROMETER_TYPE == BAROMETER_BMP388 || \
      VARIO_BAROMETER_TYPE == BAROMETER_BMP390
#include "bmp3xx_barometer.h"
using Barometer = BMP3xxBarometer;
#elif VARIO_BAROMETER_TYPE == BAROMETER_BMP580 || \
      VARIO_BAROMETER_TYPE == BAROMETER_BMP581
#include "bmp5xx_barometer.h"
using Barometer = BMP5xxBarometer;
#else
#error "Unsupported VARIO_BAROMETER_TYPE in config.h"
#endif
