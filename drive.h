// drive.h
// Mecanum chassis driver. The only file that knows wheels exist.
//
// Takes a body-frame twist in SI units and produces four servo pulse widths:
//
//     +Vx  forward, m/s
//     +Vy  LEFT,    m/s
//     +Wz  ANTICLOCKWISE viewed from above, rad/s
//
// That is a right-handed frame, and it is REP-103, so it matches the frame the
// Pi's IMU node reports in: a positive Wz should make the Pi's heading
// increase. Should, not does -- the IMU was unplugged when Vx and Vy were
// measured, so Wz is the one axis still unverified. Ported from the mecanum
// mixing in MechEng 706 servo_control.h, converted from the old arbitrary
// [-300, 300] effort units to SI, with the (L+l) rotation scaling restored and
// saturation made uniform.
//
// Two entry points, and the difference matters:
//
//   setBodyVelocity()    SI in, through the measured speed map. What the Pi
//                        uses. A request is honoured as closely as the drive
//                        can manage and saturates at the measured limit.
//   setBodyDeflection()  raw wheel command in, map bypassed. What the
//                        calibration rig uses, so that the rig does not
//                        depend on the very map it exists to measure.
#pragma once
#include <Arduino.h>
#include <Servo.h>
#include "config.h"

class Drive {
 public:
  // Attaches all four servos and holds them at neutral.
  void begin();

  // Commanded body twist, SI. Applied immediately. Each axis is converted by
  // its own measured speed curve (see speedmap.h) -- the curves are strongly
  // non-linear and a strafe is slower than a drive, so there is no single
  // scale factor that could do this.
  //
  // If the request exceeds what the wheels can do, all four are scaled by the
  // same factor: the bot goes slower than asked but in the direction asked.
  void setBodyVelocity(float vx, float vy, float wz);

  // Raw wheel command, as a fraction of full pulse deflection, in the same
  // body sense as setBodyVelocity() -- so (1, 0, 0) is "every wheel flat out
  // in the pattern that drives forward", not "1 m/s". The speed map is not
  // consulted. Saturation and MOTOR_MAX_DEFLECTION still apply.
  void setBodyDeflection(float dx, float dy, float dw);

  // Neutral on all four, immediately. Also zeroes the applied echo.
  void stop();

  // What was actually applied, after saturation scaling (body frame, SI).
  // Converted back out through the speed map, so this is a real velocity and
  // not the request echoed: on a saturated or deadbanded command it will not
  // match what was asked for.
  float appliedVx() const;
  float appliedVy() const;
  float appliedWz() const;

  // The same thing before the map, as fractions of full deflection.
  float appliedDx() const { return applied_dx_; }
  float appliedDy() const { return applied_dy_; }
  float appliedDw() const { return applied_dw_; }

  // True if the last command had to be scaled down to fit the wheels.
  bool saturated() const { return saturated_; }

 private:
  // norm is -1..+1 of full deflection, in body sense; mirror flips the
  // left-hand pair. Returns the pulse width to write.
  static uint16_t toPulse(float norm, float mirror);

  Servo lf_, rf_, lr_, rr_;

  float applied_dx_ = 0.0f;
  float applied_dy_ = 0.0f;
  float applied_dw_ = 0.0f;
  bool saturated_ = false;
};
