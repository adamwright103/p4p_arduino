// imu.h
// Thin wrapper around the BNO085 (Adafruit_BNO08x library, I2C).
//
// Every output here is a fused / calibrated BNO085 report, not raw silicon:
//
//   heading   SH2_GAME_ROTATION_VECTOR -- quaternion fused from accel + gyro,
//             no magnetometer, so the heading is relative and drifts slowly.
//             Unwrapped here into a continuous angle; the docking filter only
//             ever uses differences of it, which is what we want.
//   accel     SH2_LINEAR_ACCELERATION (accel + gyro fused, gravity removed) or
//             SH2_ACCELEROMETER (calibrated, gravity included), selected by
//             IMU_ACCEL_GRAVITY_REMOVED. Sensor frame, m/s^2.
//   yaw rate  SH2_GYROSCOPE_CALIBRATED -- bias-corrected gyro, rad/s.
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

  // Fused acceleration, m/s^2, in the SENSOR frame -- not rotated into the
  // body frame, so this assumes the chip is mounted square to the chassis.
  // See "Bring-up" in README.md for the check.
  float accelX() const { return accel_x_; }
  float accelY() const { return accel_y_; }
  float accelZ() const { return accel_z_; }
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
  float accel_x_ = 0.0f;
  float accel_y_ = 0.0f;
  float accel_z_ = 0.0f;
  bool have_yaw_ = false;

  uint32_t last_sample_ms_ = 0;
  uint32_t samples_ = 0;
  uint16_t resets_ = 0;
};
