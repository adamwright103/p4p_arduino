// imu.h
// Thin wrapper around the BNO085 (Adafruit_BNO08x library, I2C).
//
// Exposes a continuous, unwrapped heading in radians from the on-chip fused
// game rotation vector, plus the calibrated gyro yaw rate. The game rotation
// vector uses no magnetometer, so the heading is relative and drifts slowly;
// the docking filter only ever uses differences of it, which is what we want.
//
// Same unwrap logic and method names as the Gyroscope class in sensors.h, so
// this can replace it later without touching callers.
#pragma once
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

class Imu {
 public:
  explicit Imu(int8_t reset_pin = -1, float yaw_sign = 1.0f);

  // Returns false if the chip is not found or the reports cannot be enabled.
  bool begin(uint8_t i2c_addr, uint32_t report_interval_us);

  // Drains every pending report. Call as often as possible; never blocks.
  // Returns the number of new heading samples consumed.
  uint8_t poll();

  float heading() const { return heading_; }             // unwrapped, rad
  float headingWrapped() const { return wrapPi(heading_); } // (-pi, pi]
  float yawRate() const { return yaw_rate_; }             // rad/s
  float angle() const { return heading_ - offset_; }      // since resetAngle()
  void resetAngle() { offset_ = heading_; }

  uint32_t lastSampleMs() const { return last_sample_ms_; }
  uint32_t sampleCount() const { return samples_; }
  uint16_t resetCount() const { return resets_; }         // spontaneous resets

  static float wrapPi(float a);

 private:
  bool enableReports();

  Adafruit_BNO08x bno_;
  sh2_SensorValue_t value_;
  uint32_t report_us_ = 10000;
  float yaw_sign_ = 1.0f;

  float heading_ = 0.0f;
  float last_yaw_ = 0.0f;
  float offset_ = 0.0f;
  float yaw_rate_ = 0.0f;
  bool have_yaw_ = false;

  uint32_t last_sample_ms_ = 0;
  uint32_t samples_ = 0;
  uint16_t resets_ = 0;
};
