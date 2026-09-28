// drive.cpp
#include "drive.h"

void Drive::begin() {
  lf_.attach(PIN_LEFT_FRONT);
  rf_.attach(PIN_RIGHT_FRONT);
  lr_.attach(PIN_LEFT_REAR);
  rr_.attach(PIN_RIGHT_REAR);
  stop();
}

void Drive::stop() {
  lf_.writeMicroseconds(MOTOR_US_NEUTRAL);
  rf_.writeMicroseconds(MOTOR_US_NEUTRAL);
  lr_.writeMicroseconds(MOTOR_US_NEUTRAL);
  rr_.writeMicroseconds(MOTOR_US_NEUTRAL);
  applied_vx_ = 0.0f;
  applied_vy_ = 0.0f;
  applied_wz_ = 0.0f;
  saturated_ = false;
}

void Drive::setBodyVelocity(float vx, float vy, float wz) {
  vx *= DRIVE_VX_SIGN;
  vy *= DRIVE_VY_SIGN * STRAFE_GAIN;
  wz *= DRIVE_WZ_SIGN;

  // Textbook mecanum inverse kinematics, X-roller configuration. Each term is
  // the wheel's contact-patch speed in m/s. The rotation term is the tangential
  // speed the wheel must add at radius (L+l) from the centre.
  const float rot = CHASSIS_L_PLUS_W * wz;
  float lf = vx - vy - rot;
  float rf = vx + vy + rot;
  float lr = vx + vy - rot;
  float rr = vx - vy + rot;

  // Normalise to full deflection.
  const float inv = 1.0f / WHEEL_MAX_MPS;
  lf *= inv;  rf *= inv;  lr *= inv;  rr *= inv;

  // Uniform saturation. Clipping each wheel on its own would bend the motion
  // away from the commanded direction exactly when the bot is working hardest,
  // so instead find the worst offender and scale the whole set by one factor.
  float peak = fabs(lf);
  if (fabs(rf) > peak) peak = fabs(rf);
  if (fabs(lr) > peak) peak = fabs(lr);
  if (fabs(rr) > peak) peak = fabs(rr);

  float scale = 1.0f;
  saturated_ = (peak > 1.0f);
  if (saturated_) scale = 1.0f / peak;

  lf *= scale;  rf *= scale;  lr *= scale;  rr *= scale;

  lf_.writeMicroseconds(toPulse(lf, MOTOR_MIRROR_LEFT));
  rf_.writeMicroseconds(toPulse(rf, MOTOR_MIRROR_RIGHT));
  lr_.writeMicroseconds(toPulse(lr, MOTOR_MIRROR_LEFT));
  rr_.writeMicroseconds(toPulse(rr, MOTOR_MIRROR_RIGHT));

  // Scaling all four wheels by one factor is exactly a uniform scaling of the
  // body twist, so the echo is just the request times that factor -- reported
  // before the sign trims, in the caller's own convention.
  applied_vx_ = vx * scale * DRIVE_VX_SIGN;
  applied_vy_ = vy * scale / STRAFE_GAIN * DRIVE_VY_SIGN;
  applied_wz_ = wz * scale * DRIVE_WZ_SIGN;
}

uint16_t Drive::toPulse(float norm, float mirror) {
  norm *= mirror;
  if (norm >  1.0f) norm =  1.0f;
  if (norm < -1.0f) norm = -1.0f;
  const float span = (float)(MOTOR_US_MAX - MOTOR_US_NEUTRAL);
  long us = (long)MOTOR_US_NEUTRAL + (long)(norm * span);
  if (us < (long)MOTOR_US_MIN) us = MOTOR_US_MIN;
  if (us > (long)MOTOR_US_MAX) us = MOTOR_US_MAX;
  return (uint16_t)us;
}
