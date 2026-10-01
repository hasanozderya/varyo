#pragma once
#include <Arduino.h>

// Non-blocking BMP280/BME280 pressure driver. The humidity channel of a
// BME280 is intentionally unused; both chips share the pressure engine.
class BMP28xBarometer {
public:
    BMP28xBarometer();
    struct Diagnostics {
        uint8_t chipId = 0;
        bool calibrationRead = false;
        bool calibrationValid = false;
    };
    bool begin();
    void startRecovery();
    bool serviceRecovery();
    void update();
    bool hasNewSample() const { return newSample_; }
    void clearNewSample() { newSample_ = false; }
    float pressurePa() const { return pressurePa_; }
    float temperatureC() const { return temperatureC_; }
    uint32_t rawD1() const { return rawPressure_; }
    uint32_t rawD2() const { return rawTemperature_; }
    uint32_t filteredD2() const { return rawTemperature_; }
    uint32_t runtimeBusMisses() const { return runtimeBusMisses_; }
    uint32_t sampleTimeUs() const { return sampleUs_; }
    Diagnostics diagnostics() const { return diagnostics_; }
    static constexpr const char* modelName() {
#if VARIO_BAROMETER_TYPE == BAROMETER_BME280
        return "BME280";
#else
        return "BMP280";
#endif
    }
    static float pressureToAltitude(float pressurePa, float qnhHpa);
private:
    bool writeRegister(uint8_t reg, uint8_t value, bool nonBlocking = false);
    bool readRegisters(uint8_t reg, uint8_t* data, size_t size, bool nonBlocking = false);
    bool readCalibration(bool nonBlocking);
    bool readMeasurement();
    uint16_t t1_=0, p1_=0;
    int16_t t2_=0, t3_=0, p2_=0, p3_=0, p4_=0, p5_=0, p6_=0, p7_=0, p8_=0, p9_=0;
    bool initialized_=false, newSample_=false;
    uint8_t recoveryStep_=0;
    uint32_t recoveryUs_=0, lastReadUs_=0, sampleUs_=0, runtimeBusMisses_=0;
    uint32_t rawPressure_=0, rawTemperature_=0;
    float pressurePa_=101325.0f, temperatureC_=20.0f;
    const uint8_t expectedChipId_;
    Diagnostics diagnostics_{};
};
