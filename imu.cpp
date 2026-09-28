// imu.cpp
#include "imu.h"

Imu::Imu(int8_t reset_pin, float yaw_sign)
    : bno_(reset_pin), yaw_sign_(yaw_sign) {}

bool Imu::begin(uint8_t i2c_addr, uint32_t report_interval_us) {
  report_us_ = report_interval_us;
  if (!bno_.begin_I2C(i2c_addr)) return false;
  if (!enableReports()) return false;
  bno_.wasReset();   // clear the flag raised by the power-up reset
  have_yaw_ = false;
  return true;
}

bool Imu::enableReports() {
  bool ok = bno_.enableReport(SH2_GAME_ROTATION_VECTOR, report_us_);
  ok = bno_.enableReport(SH2_GYROSCOPE_CALIBRATED, report_us_) && ok;
  return ok;
}

float Imu::wrapPi(float a) {
  while (a > PI) a -= TWO_PI;
  while (a < -PI) a += TWO_PI;
  return a;
}

uint8_t Imu::poll() {
  // The BNO can reset on its own. Reports must be re-enabled, and the yaw
  // jump across the reset must not be integrated into the heading.
  if (bno_.wasReset()) {
    resets_++;
    enableReports();
    have_yaw_ = false;
  }

  uint8_t n = 0;
  while (bno_.getSensorEvent(&value_)) {
    switch (value_.sensorId) {
      case SH2_GAME_ROTATION_VECTOR: {
        const auto& q = value_.un.gameRotationVector;   // i, j, k, real
        // Yaw about Z from the fused quaternion. No extra smoothing: the
        // chip already filters, and averaging a wrapped angle would be wrong.
        float yaw = yaw_sign_ * atan2(2.0f * (q.real * q.k + q.i * q.j),
                                      1.0f - 2.0f * (q.j * q.j + q.k * q.k));
        if (!have_yaw_) {           // first sample after start or reset
          last_yaw_ = yaw;
          have_yaw_ = true;
        }
        heading_ += wrapPi(yaw - last_yaw_);   // unwrap into a continuous angle
        last_yaw_ = yaw;
        last_sample_ms_ = millis();
        samples_++;
        n++;
        break;
      }
      case SH2_GYROSCOPE_CALIBRATED:
        yaw_rate_ = yaw_sign_ * value_.un.gyroscope.z;
        break;
      default:
        break;
    }
  }
  return n;
}
