#pragma once

// Queue a ground-only calibration for the task that owns the I2C sensor.
bool requestGroundImuCalibration();

// Core 0: high-priority, fixed-rate (100Hz) task. Reads selected barometer +
// selected IMU, runs the two-stage sensor fusion, drives the buzzer, and
// publishes the result to SharedState for the UI task to consume.
// Must never be starved — this is the "zero interruption" real-time path.
void varioTaskFunc(void* pvParameters);
