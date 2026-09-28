// drive.h
// Mecanum chassis driver. The only file that knows wheels exist.
//
// Takes a body-frame twist in SI units and produces four servo pulse widths:
//
//     +Vx  forward, m/s
//     +Vy  LEFT,    m/s
//     +Wz  ANTICLOCKWISE viewed from above, rad/s
//
// That is a right-handed frame and it matches the sign of Imu::heading(), so
// a positive Wz makes the reported heading increase. Ported from the mecanum
// mixing in MechEng 706 servo_control.h, converted from the old arbitrary
// [-300, 300] effort units to SI, with the (L+l) rotation scaling restored and
// saturation made uniform.
#pragma once
#include <Arduino.h>
#include <Servo.h>
#include "config.h"

class Drive {
 public:
  // Attaches all four servos and holds them at neutral.
  void begin();

  // Commanded body twist, SI. Applied immediately. If the request exceeds what
  // the wheels can do, all four are scaled by the same factor: the bot goes
  // slower than asked but in the direction asked.
  void setBodyVelocity(float vx, float vy, float wz);

  // Neutral on all four, immediately. Also zeroes the applied-velocity echo.
  void stop();

  // What was actually applied, after saturation scaling (body frame, SI).
  // This is what goes out in telemetry, so the Pi can see the difference
  // between what it asked for and what the chassis could deliver.
  float appliedVx() const { return applied_vx_; }
  float appliedVy() const { return applied_vy_; }
  float appliedWz() const { return applied_wz_; }

  // True if the last setBodyVelocity() had to be scaled down.
  bool saturated() const { return saturated_; }

 private:
  // norm is -1..+1 of full deflection, in body sense; mirror flips the
  // left-hand pair. Returns the pulse width to write.
  static uint16_t toPulse(float norm, float mirror);

  Servo lf_, rf_, lr_, rr_;

  float applied_vx_ = 0.0f;
  float applied_vy_ = 0.0f;
  float applied_wz_ = 0.0f;
  bool saturated_ = false;
};
