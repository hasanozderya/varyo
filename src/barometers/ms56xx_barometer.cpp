#include "ms56xx_barometer.h"
#include "pressure_math.h"
#include "config.h"
#include "i2c_bus.h"
#include <Wire.h>
#include <math.h>
#include <string.h>

namespace {
    constexpr uint8_t CMD_RESET      = 0x1E;
    constexpr uint8_t CMD_CONVERT_D1 = 0x48; // pressure,    OSR = 4096
    constexpr uint8_t CMD_CONVERT_D2 = 0x58; // temperature, OSR = 4096
    constexpr uint8_t CMD_ADC_READ   = 0x00;
    constexpr uint8_t CMD_PROM_BASE  = 0xA0;
    constexpr uint32_t CONVERSION_US = 9300;  // datasheet max 9.04ms @OSR4096 + 0.26ms margin
    constexpr uint16_t REFRESH_TEMP_EVERY_N_SAMPLES = 20; // temp drifts slowly
    constexpr float TEMP_D2_LPF_ALPHA = 0.10f; // ~4 s time constant at ~2.2 D2 samples/s

    // Three mutually consistent samples can establish a new baseline after
    // a gap or a bad first sample. Isolated outliers never become measurements.
    bool acceptRaw(uint32_t raw, uint32_t& accepted, uint32_t& pending,
                   uint8_t& count, uint32_t limit) {
        if (raw == 0 || raw == 0xFFFFFF) { count = 0; return false; }
        if (accepted != 0 && abs((int32_t)raw - (int32_t)accepted) < (int32_t)limit) {
            accepted = raw;
            count = 0;
            return true;
        }
        if (count == 0 || abs((int32_t)raw - (int32_t)pending) >= (int32_t)(limit / 4))
            count = 0;
        pending = raw;
        if (++count < 3) return false;
        accepted = raw;
        count = 0;
        return true;
    }

    // Standard TE/Measurement-Specialties PROM CRC4 (application note
    // AN520), shared by the whole MS56xx family.
    uint8_t crc4(const uint16_t prom[8]) {
        uint16_t p[8];
        memcpy(p, prom, sizeof(p));
        p[7] &= 0xFF00; // CRC nibble itself is excluded from the calculation
        uint16_t remainder = 0;
        for (int i = 0; i < 16; i++) {
            remainder ^= (i % 2 == 1) ? (p[i >> 1] & 0x00FF) : (p[i >> 1] >> 8);
            for (int bit = 8; bit > 0; bit--) {
                remainder = (remainder & 0x8000) ? (remainder << 1) ^ 0x3000
                                                  : (remainder << 1);
            }
        }
        return (remainder >> 12) & 0x000F;
    }
}

MS56xxBarometer::MS56xxBarometer()
    : useMs5611Compensation_(VARIO_BAROMETER_TYPE == BAROMETER_MS5611) {}

bool MS56xxBarometer::sendCommand(uint8_t cmd, bool nonBlocking) {
    I2CLockGuard lock(nonBlocking ? 0 : Timing::I2C_MUTEX_TIMEOUT);
    if (!lock.ok()) { if (nonBlocking) runtimeBusMisses_++; return false; }    
    Wire.beginTransmission(I2CAddr::BAROMETER);
    Wire.write(cmd);
    return (Wire.endTransmission() == 0);
}

uint32_t MS56xxBarometer::readAdc(bool nonBlocking) {
    I2CLockGuard lock(nonBlocking ? 0 : Timing::I2C_MUTEX_TIMEOUT);
    if (!lock.ok()) { if (nonBlocking) runtimeBusMisses_++; return 0; }
    Wire.beginTransmission(I2CAddr::BAROMETER);
    Wire.write(CMD_ADC_READ);
    if (Wire.endTransmission() != 0) return 0;
    if (Wire.requestFrom((uint8_t)I2CAddr::BAROMETER, (uint8_t)3) != 3) return 0;
    uint32_t v = 0;
    for (int i = 0; i < 3; i++) v = (v << 8) | Wire.read();
    return v;
}

bool MS56xxBarometer::readProm(bool nonBlocking) {
    I2CLockGuard lock(nonBlocking ? 0 : Timing::I2C_MUTEX_TIMEOUT);
    if (!lock.ok()) return false;
    for (int i = 0; i < 8; i++) {
        Wire.beginTransmission(I2CAddr::BAROMETER);
        Wire.write(CMD_PROM_BASE + i * 2);
        if (Wire.endTransmission() != 0 ||
            Wire.requestFrom((uint8_t)I2CAddr::BAROMETER, (uint8_t)2) != 2) return false;
        uint8_t hi = Wire.read(), lo = Wire.read();
        coeff_[i] = ((uint16_t)hi << 8) | lo;
    }
    return true;
}

bool MS56xxBarometer::begin() {
    if (!I2CBus::ping(I2CAddr::BAROMETER, 0)) return false;
    startRecovery();
    for (unsigned i = 0; i < 20; ++i) {
        if (serviceRecovery()) return true;
        delay(5);
    }
    return false;
}

void MS56xxBarometer::startRecovery() {
    initialized_ = false;
    recoveryStep_ = 0;
    phase_ = Phase::Idle;
    newSample_ = false;
    d1_ = d2_ = pendingD1_ = pendingD2_ = lastD2Us_ = sampleUs_ = 0;
    pendingD1Count_ = pendingD2Count_ = 0;
    d2Filtered_ = 0;
    pressureSamplesSinceTemp_ = REFRESH_TEMP_EVERY_N_SAMPLES;
}

bool MS56xxBarometer::serviceRecovery() {
    if (initialized_) return true;
    if (recoveryStep_ == 0) {
        if (!sendCommand(CMD_RESET, true)) return false;
        recoveryUs_ = micros();
        recoveryStep_ = 1;
        return false;
    }
    if (recoveryStep_ != 1 || (uint32_t)(micros() - recoveryUs_) < 5000) return false;
    promReadOk_ = readProm(true);
    if (!promReadOk_) return false;
    crcMatch_ = ((coeff_[7] & 0x0F) == crc4(coeff_));
    initialized_ = crcMatch_;
    for (unsigned i = 1; i <= 6; ++i)
        if (coeff_[i] == 0 || coeff_[i] == 0xFFFF) initialized_ = false;
    recoveryStep_ = 2;
    return initialized_;
}

MS56xxBarometer::Diagnostics MS56xxBarometer::diagnostics() const {
    Diagnostics d{};
    memcpy(d.coeff, coeff_, sizeof(d.coeff));
    d.crcExpected   = coeff_[7] & 0x0F;
    d.crcCalculated = crc4(coeff_);
    d.promReadOk    = promReadOk_;
    d.crcMatch      = crcMatch_;
    return d;
}

void MS56xxBarometer::compute() {
    // MS5607-02BA compensation. NOTE: the exponents here DIFFER from the
    // very similar-looking MS5611 formula (OFF: C2<<17 / (C4*dT)>>6,
    // SENS: C1<<16 / (C3*dT)>>7 — one bit off in either direction from
    // MS5611). Porting MS5611 code as-is to an MS5607 is the #1 cause of
    // a "pressure reads roughly half/double the real value" bug.
    const int64_t d2Comp = (int64_t)llroundf(d2Filtered_ > 0.0f ? d2Filtered_ : (float)d2_);
    int64_t dT   = d2Comp - ((int64_t)coeff_[5] << 8);
    int64_t TEMP = 2000 + (dT * coeff_[6]) / (1LL << 23);

    int64_t OFF, SENS;
    if (useMs5611Compensation_) {
        OFF  = ((int64_t)coeff_[2] << 16) + ((int64_t)coeff_[4] * dT) / (1LL << 7);
        SENS = ((int64_t)coeff_[1] << 15) + ((int64_t)coeff_[3] * dT) / (1LL << 8);
    } else {
        OFF  = ((int64_t)coeff_[2] << 17) + ((int64_t)coeff_[4] * dT) / (1LL << 6);
        SENS = ((int64_t)coeff_[1] << 16) + ((int64_t)coeff_[3] * dT) / (1LL << 7);
    }

    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    if (TEMP < 2000) {
        int64_t d = (TEMP - 2000) * (TEMP - 2000);
        T2    = (dT * dT) / (1LL << 31);
        if (useMs5611Compensation_) {
            OFF2  = (5 * d) / 2;
            SENS2 = (5 * d) / 4;
            if (TEMP < -1500) {
                int64_t d2 = (TEMP + 1500) * (TEMP + 1500);
                OFF2  += 7 * d2;
                SENS2 += (11 * d2) / 2;
            }
        } else {
            OFF2  = (61 * d) / (1LL << 4);
            SENS2 = 2 * d;
            if (TEMP < -1500) {
                int64_t d2 = (TEMP + 1500) * (TEMP + 1500);
                OFF2  += 15 * d2;
                SENS2 += 8  * d2;
            }
        }
    }
    TEMP -= T2; OFF -= OFF2; SENS -= SENS2;

    const float P = ms5607CompensatedPressurePa(d1_, SENS, OFF);

    if (P > 1000 && P < 130000) {   // sanity gate: 10..1300 mbar
        pressurePa_   = (float)P;
        temperatureC_ = TEMP / 100.0f;
        newSample_    = true;
    }
}

void MS56xxBarometer::update() {
    if (!initialized_) return;
    uint32_t now = micros();
    if (phase_ == Phase::WaitD1 && (uint32_t)(now - convStartUs_) >= CONVERSION_US) {
        uint32_t raw = readAdc(true);
        ++pressureSamplesSinceTemp_;
        if (acceptRaw(raw, d1_, pendingD1_, pendingD1Count_, 100000) &&
            d2_ != 0 && (uint32_t)(now - lastD2Us_) < 2000000UL) {
            sampleUs_ = convStartUs_ + CONVERSION_US / 2;
            compute();
        }
        phase_ = Phase::Idle;
    } else if (phase_ == Phase::WaitD2 && (uint32_t)(now - convStartUs_) >= CONVERSION_US) {
        uint32_t raw = readAdc(true);
        if (acceptRaw(raw, d2_, pendingD2_, pendingD2Count_, 50000)) {
            if (d2Filtered_ <= 0 || (uint32_t)(now - lastD2Us_) >= 2000000UL)
                d2Filtered_ = (float)raw;
            else d2Filtered_ += TEMP_D2_LPF_ALPHA * ((float)raw - d2Filtered_);
            lastD2Us_ = now;
        } else pressureSamplesSinceTemp_ = REFRESH_TEMP_EVERY_N_SAMPLES;
        phase_ = Phase::Idle;
    }
    // Pipeline the next conversion immediately after reading the previous one.
    // Timestamp AFTER the command, not before a possibly delayed bus transfer.
    if (phase_ == Phase::Idle) {
        if (pressureSamplesSinceTemp_ >= REFRESH_TEMP_EVERY_N_SAMPLES) {
            if (sendCommand(CMD_CONVERT_D2, true)) {
                phase_ = Phase::WaitD2;
                pressureSamplesSinceTemp_ = 0;
                convStartUs_ = micros();
            }
        } else {
            if (sendCommand(CMD_CONVERT_D1, true)) {
                phase_ = Phase::WaitD1;
                convStartUs_ = micros();
            }
        }
    }
}

float MS56xxBarometer::pressureToAltitude(float pressurePa, float qnhHpa) {
    return pressureToAltitudeM(pressurePa, qnhHpa);
}
