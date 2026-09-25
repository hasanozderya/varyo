#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>

// Select one control transport. BLE is the default for the Android app.
// Define VARIO_USE_WIFI=1 to build the legacy web interface instead.
#ifndef VARIO_USE_WIFI
#define VARIO_USE_WIFI 0
#endif

// =====================================================================
//  HARDWARE PIN MAP — adjust to match your wiring
// =====================================================================
namespace Pins {
    // I2C bus (MS5607 + MPU6050 share this bus)
    constexpr int I2C_SDA = 8;
    constexpr int I2C_SCL = 9;

    // GPS UART (NEO-M8N) — ESP32-S3 UART0 on GPIO 43/44
    constexpr int GPS_RX  = 2;   // ESP32 RX  <- GPS TX
    constexpr int GPS_TX  = 1;   // ESP32 TX  -> GPS RX

    // 2.8" SPI TFT, ILI9341 module. Safe available pins on this board:
    // 10/11/12/13 = display control, and 14/15/16 = SPI bus.
    // Keep GPIO 1/3 boot serial pins free and avoid GPS UART pins 17/18.
    constexpr int DISPLAY_CS  = 10;
    constexpr int DISPLAY_DC  = 11;
    constexpr int DISPLAY_RST = 12;
    constexpr int DISPLAY_BL  = 13;
    constexpr int SPI_SCK     = 14;
    constexpr int SPI_MOSI    = 15;
    constexpr int SPI_MISO    = 16;

    // MAX98357A: remove the previous passive buzzer from GPIO4.
    constexpr int AUDIO_DATA = 4;
    constexpr int AUDIO_BCLK = 5;
    constexpr int AUDIO_WS = 6;
}

// =====================================================================
//  I2C DEVICE ADDRESSES
// =====================================================================
namespace I2CAddr {
    constexpr uint8_t MS5607  = 0x77;   // CSB -> GND  (0x76 if CSB -> VCC)
    constexpr uint8_t MPU6050 = 0x68;   // AD0 -> GND  (0x69 if AD0 -> VCC)
    constexpr uint8_t OLED    = 0x3C;
}

// =====================================================================
//  BUS / TASK TIMING
// =====================================================================
namespace Timing {
    constexpr uint32_t I2C_CLOCK_HZ       = 400000;
    constexpr uint16_t I2C_TIMEOUT_MS     = 4;
    constexpr uint32_t GPS_BAUD           = 9600;

    constexpr uint32_t VARIO_TASK_HZ      = 100;  // Core 0: fusion + audio
    constexpr uint32_t UI_TASK_HZ         = 500;  // GPS drain + bounded OLED chunks
    constexpr uint32_t DISPLAY_REFRESH_HZ = 5;     // final UI: 5 Hz; vario/audio remain 100 Hz

    constexpr TickType_t I2C_MUTEX_TIMEOUT = pdMS_TO_TICKS(50);

    // WiFi AP, boot sonrasi (sensorler hazir olur olmaz) acilir.
    // - Hic kimse baglanmazsa WIFI_ON_DURATION_MS sonunda pes edip kapanir
    //   (yoksa kimse baglanmasa bile radyo sonsuza kadar acik kalirdi).
    // - Biri baglandiktan sonra bu sinir devre disi kalir, baglantı
    //   surdukce AP acik kalir.
    // - Baglanti kesilince WIFI_DISCONNECT_GRACE_MS kadar tolerans
    //   taninir (gecici kopma/telefon ekran kilidi vb.), o sure boyunca
    //   kimse tekrar baglanmazsa AP tamamen kapanir (WiFi.mode(WIFI_OFF)).
    // Tekrar acmak icin cihazi resetlemek/guc vermek gerekir.
    constexpr uint32_t WIFI_ON_DURATION_MS      = 3UL * 60UL * 1000UL; // 3 dakika (ilk baglanti icin ust sinir)
    constexpr uint32_t WIFI_DISCONNECT_GRACE_MS = 10UL * 1000UL;       // 10 saniye (kopma toleransi)
}


// =====================================================================
//  DEVICE-SPECIFIC MPU6050 ACCELEROMETER CALIBRATION
//  Derived from the 2026-09-21 six-orientation static capture on this unit.
//  Raw accelerometer values are corrected as (raw_g - offset) * scale
//  BEFORE attitude/fusion. Both terms are required: the previous scale-only
//  correction left the Z faces at -0.867 g / +0.992 g and injected about
//  -1.25 m/s^2 into the stationary vertical-acceleration estimate.
//  A small residual norm error can still be saved by ground calibration.
// =====================================================================
namespace ImuCalibration {
    constexpr float ACCEL_OFFSET_X = 0.011786f;
    constexpr float ACCEL_OFFSET_Y = 0.002527f;
    constexpr float ACCEL_OFFSET_Z = 0.068462f;
    constexpr float ACCEL_SCALE_X  = 0.993355f;
    constexpr float ACCEL_SCALE_Y  = 1.002767f;
    constexpr float ACCEL_SCALE_Z  = 0.984137f;
}

// =====================================================================
//  VARIO / FUSION TUNING — compile-time DEFAULTS ONLY, ve SADECE ILK
//  BOOT (NVS bos/temiz) icin gecerli. TunablesStore::init() artik once
//  NVS'den (flash) okumayi dener; web UI'dan yapilan her degisiklik
//  otomatik olarak NVS'e yazilip kalici hale gelir. Bu degerleri
//  degistirmek yalnizca "hic web'den kayit yapilmamis, sifirdan flashlanmis
//  bir cihazin" ilk acilista neyle baslayacagini etkiler — NVS'de zaten
//  kayitli deger varsa bu sabitler goz ardi edilir.
// =====================================================================
namespace VarioTuning {
    // Added response lag: 63% in 0.40 s, 90% in about 0.92 s. The calibrated
    // IMU/Kalman path already rejects fast noise, so a long final-stage lag
    // only makes the value hang after physical motion has stopped.
    // No zero clamp: sustained weak lift/sink remains measurable.
    constexpr float OUTPUT_LPF_TAU_S = 0.40f;
    // Complementary filter blend for pitch/roll (closer to 1 = trust the
    // integrated gyro more; the accelerometer only corrects long-term drift).
    constexpr float COMP_FILTER_ALPHA = 0.98f;
    // First-order low-pass on RAW pressure before pressure->altitude and
    // Kalman correction. Keep this short: a long prefilter delays the
    // barometer relative to the IMU and creates a tail after stopping.
    constexpr float BARO_PRESSURE_LPF_TAU_S = 0.1f;


    // 3-state Kalman filter (altitude / climb-rate / accel-bias).
    // IMPORTANT (direction, easy to get backwards): acceleration is used
    // as the PREDICT step's control input, not as a measurement. A LARGER
    // KF_ACCEL_VAR tells the filter "trust the accel-driven prediction
    // LESS" -> the barometer correction in correct() is allowed to pull
    // harder. Whether that smooths the output depends on which sensor is
    // noisier; this is not a generic smoothing-strength control.
    constexpr float KF_ACCEL_VAR       = 7.01f;   // m^2/s^4
    constexpr float KF_ACCEL_BIAS_VAR  = 1e-6f;  // accel-bias random-walk noise
    constexpr float KF_BARO_VAR        = 0.2804f;  // m^2; tuning start point, verify with hardware capture
    constexpr float KF_ADAPT_FACTOR    = 1.0f;   // Yüksek ivmelerde eklenecek adaptif varyans çarpanı
    constexpr float KF_BIAS_LIMIT_MPS2 = 0.50f;  // axis calibration should keep residual far below this
    constexpr float CLIMB_DEADBAND_MPS = 0.10f;  // silence within +/- this
    constexpr float CLIMB_MAX_MPS      = 5.0f;   // audio pitch/rate scale ceiling
    constexpr float SINK_ALARM_MPS     = -2.0f;  // continuous low tone below this
   

    // Use calibrated IMU fusion when healthy; barometer remains the fallback.
    constexpr bool     USE_BARO_REGRESSION_OUTPUT = false;
    // The physical unit shows roughly 15-20 Pa low-frequency bench noise.
    // A longer real-timestamp regression rejects that noise while preserving
    // the slope of a sustained climb/sink. IMU fusion remains the fast path.
    constexpr uint32_t BARO_REGRESSION_WINDOW_MS  = 3000;
    constexpr uint8_t  BARO_REGRESSION_MIN_SAMPLES = 12;
    // Numerical floor only. A hidden 0.35 minimum made entered settings
    // such as 0.10 or 0.20 ineffective, despite being accepted and saved.
    constexpr float KF_BARO_VAR_FLOOR = 1e-4f;
    
}

namespace VarioDebug {
    constexpr bool     SERIAL_CSV = true;  // log CSV to serial for PC graphing/debugging
    constexpr uint32_t SERIAL_CSV_HZ = 12;
    constexpr bool     MUTE_BUZZER_DURING_DIAG = true; // mute the buzzer while the PC logger is running
}

// =====================================================================
//  BUZZER AUDIO TUNING — TONE_MIN/MAX_HZ, SINK_TONE_HZ, and the climb/
//  sink thresholds above are compile-time DEFAULTS, seeded into
//  Tunables at startup (see the note above). BEEP_PERIOD_*/DUTY stay
//  compile-time-only (not exposed to the web UI) since they're cadence
//  shaping details rather than something you'd typically tune live.
// =====================================================================
namespace AudioTuning {
    constexpr int      TONE_MIN_HZ        = 700;  // pitch at the climb deadband
    constexpr int      TONE_MAX_HZ        = 2200; // pitch at CLIMB_MAX_MPS
    constexpr int      SINK_TONE_HZ       = 450;  // continuous sink tone
    constexpr uint32_t BEEP_PERIOD_MAX_MS = 700;  // slow beep near the deadband
    constexpr uint32_t BEEP_PERIOD_MIN_MS = 120;  // fast beep near CLIMB_MAX_MPS
    constexpr float    BEEP_DUTY_FRACTION = 0.45f; // fraction of period spent "on"
}
