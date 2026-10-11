// drive.cpp
#include "drive.h"
#include "speedmap.h"

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
  applied_dx_ = 0.0f;
  applied_dy_ = 0.0f;
  applied_dw_ = 0.0f;
  saturated_ = false;
}

void Drive::setBodyVelocity(float vx, float vy, float wz) {
  // Each axis through its own measured curve. Mixing the three in deflection
  // space rather than in m/s is exact for motion along a single axis, which is
  // what the map was measured on, and an approximation for a combination --
  // the alternative would be a per-wheel speed curve, and a chassis whose
  // strafe is 30% slower than its drive at the same wheel command is telling
  // us no such single curve exists.
  setBodyDeflection(SpeedMap::forwardDeflection(vx),
                    SpeedMap::strafeDeflection(vy),
                    SpeedMap::rotationDeflection(CHASSIS_L_PLUS_W * wz));
}

float Drive::appliedVx() const { return SpeedMap::forwardMps(applied_dx_); }
float Drive::appliedVy() const { return SpeedMap::strafeMps(applied_dy_); }
float Drive::appliedWz() const {
  return SpeedMap::forwardMps(applied_dw_) / CHASSIS_L_PLUS_W;
}

void Drive::setBodyDeflection(float dx, float dy, float dw) {
  dx *= DRIVE_VX_SIGN;
  dy *= DRIVE_VY_SIGN;
  dw *= DRIVE_WZ_SIGN;

  // Textbook mecanum inverse kinematics, X-roller configuration. Each term is
  // the wheel's share of the motion, as a fraction of full deflection. Which
  // physical corner each of these four reaches is set by the pin constants in
  // config.h, and that mapping is measured, not assumed -- see the comment on
  // PIN_LEFT_FRONT.
  float lf = dx - dy - dw;
  float rf = dx + dy + dw;
  float lr = dx + dy - dw;
  float rr = dx - dy + dw;

  // Uniform saturation. Clipping each wheel on its own would bend the motion
  // away from the commanded direction exactly when the bot is working hardest,
  // so instead find the worst offender and scale the whole set by one factor.
  float peak = fabs(lf);
  if (fabs(rf) > peak) peak = fabs(rf);
  if (fabs(lr) > peak) peak = fabs(lr);
  if (fabs(rr) > peak) peak = fabs(rr);

  // The ceiling is MOTOR_MAX_DEFLECTION, not 1.0: full deflection does not
  // turn the wheels reliably, so it must not be reachable by any command.
  float scale = 1.0f;
  saturated_ = (peak > MOTOR_MAX_DEFLECTION);
  if (saturated_) scale = MOTOR_MAX_DEFLECTION / peak;

  lf *= scale;  rf *= scale;  lr *= scale;  rr *= scale;

  lf_.writeMicroseconds(toPulse(lf, MOTOR_MIRROR_LEFT));
  rf_.writeMicroseconds(toPulse(rf, MOTOR_MIRROR_RIGHT));
  lr_.writeMicroseconds(toPulse(lr, MOTOR_MIRROR_LEFT));
  rr_.writeMicroseconds(toPulse(rr, MOTOR_MIRROR_RIGHT));

  // Scaling all four wheels by one factor is exactly a uniform scaling of the
  // body command, so the echo is just the request times that factor --
  // reported before the sign trims, in the caller's own convention.
  applied_dx_ = dx * scale * DRIVE_VX_SIGN;
  applied_dy_ = dy * scale * DRIVE_VY_SIGN;
  applied_dw_ = dw * scale * DRIVE_WZ_SIGN;
}

uint16_t Drive::toPulse(float norm, float mirror) {
  norm *= mirror;
  // Belt and braces: the uniform scaling above already keeps every wheel
  // inside the ceiling, so this only catches a caller reaching toPulse()
  // around it.
  if (norm >  MOTOR_MAX_DEFLECTION) norm =  MOTOR_MAX_DEFLECTION;
  if (norm < -MOTOR_MAX_DEFLECTION) norm = -MOTOR_MAX_DEFLECTION;
  const float span = (float)(MOTOR_US_MAX - MOTOR_US_NEUTRAL);
  long us = (long)MOTOR_US_NEUTRAL + (long)(norm * span);
  if (us < (long)MOTOR_US_MIN) us = MOTOR_US_MIN;
  if (us > (long)MOTOR_US_MAX) us = MOTOR_US_MAX;
  return (uint16_t)us;
}
