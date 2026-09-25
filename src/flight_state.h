#pragma once
#include <stdint.h>

enum class FlightMode : uint8_t { Unknown, Ground, Flying, Landed };

// A skipped poll is not motion. Still require recent IMU evidence throughout
// the eight-second quiet interval before permitting a requested calibration.
class GroundStability {
public:
    void observeImu(uint32_t nowMs, bool stationary) {
        imuMs_ = nowMs; stationary_ = stationary; haveImu_ = true;
    }
    bool update(uint32_t nowMs, bool measurementValid, float varioMps,
                bool gpsMotionValid, float speedMps);
private:
    uint32_t imuMs_ = 0, quietSinceMs_ = 0;
    bool stationary_ = false, haveImu_ = false, quiet_ = false;
};

// Sensor-derived flight state never calibrates sensors or mutes the vario.
class FlightStateMachine {
public:
    void update(uint32_t nowMs, bool measurementValid, float varioMps,
                bool groundStable, bool gpsMotionValid, float speedMps);
    void start(uint32_t nowMs);
    void stop(uint32_t nowMs);
    FlightMode mode() const { return mode_; }
    uint32_t session() const { return session_; }
    uint32_t durationMs(uint32_t nowMs) const {
        return mode_ == FlightMode::Flying ? nowMs-startMs_ : durationMs_;
    }
private:
    FlightMode mode_ = FlightMode::Unknown;
    uint32_t session_ = 0, startMs_ = 0, durationMs_ = 0;
    uint32_t movingSince_ = 0, landingSince_ = 0;
    bool moving_ = false, landing_ = false, manualStop_ = false;
};

namespace FlightControl {
    void requestStart();
    void requestStop();
    int takeRequest(); // owner: measurement task
}
