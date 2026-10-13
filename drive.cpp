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
  write(MOTOR_US_NEUTRAL, MOTOR_US_NEUTRAL, MOTOR_US_NEUTRAL, MOTOR_US_NEUTRAL);
  applied_vx_ = 0.0f;
  applied_vy_ = 0.0f;
  applied_wz_ = 0.0f;
  saturated_ = false;
}

void Drive::write(uint16_t lf, uint16_t rf, uint16_t lr, uint16_t rr) {
  lf_.writeMicroseconds(lf);
  rf_.writeMicroseconds(rf);
  lr_.writeMicroseconds(lr);
  rr_.writeMicroseconds(rr);
  pulse_us_[CH_LF] = lf;
  pulse_us_[CH_RF] = rf;
  pulse_us_[CH_LR] = lr;
  pulse_us_[CH_RR] = rr;
}

void Drive::setBodyVelocity(float vx, float vy, float wz) {
  // The request as the caller gave it, in the body frame, for the echo below.
  const float req_vx = vx, req_vy = vy, req_wz = wz;
  // The trims and the strafe gain act on the body's own axes, so they come
  // before the channel rotation: after it, the mixing's vy is the body's vx.
  vx *= DRIVE_VX_SIGN;
  // The turn a strafe causes (config.h), cancelled in the body's own frame.
  wz += STRAFE_YAW_FF * vy;
  vy *= DRIVE_VY_SIGN * STRAFE_GAIN;
  wz *= DRIVE_WZ_SIGN;
  if (DRIVE_CHANNELS_ROTATED) {
    // See config.h: the mixing's frame is the body's turned a quarter turn with
    // the yaw mirrored, so turn the request into it before mixing.
    const float fwd = vx;
    vx = vy;
    vy = -fwd;
    wz = -wz;
  }

  // Textbook mecanum inverse kinematics, X-roller configuration. Each term is
  // the wheel's contact-patch speed in m/s. The rotation term is the tangential
  // speed the wheel must add at radius (L+l) from the centre.
  const float rot = CHASSIS_L_PLUS_W * wz;
  float lf = vx - vy - rot;
  float rf = vx + vy + rot;
  float lr = vx + vy - rot;
  float rr = vx - vy + rot;

  if (DRIVE_WHEEL_CAL) {
    // Each wheel's own response (config.h). Saturation stays uniform: the
    // largest scale at which every wheel's pulse still fits the span.
    const float span = (float)(MOTOR_US_MAX - MOTOR_US_NEUTRAL);
    const float w[NUM_CHANNELS] = {lf, rf, lr, rr};
    float scale = 1.0f;
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
      const float need = wheelOffsetUs(ch, w[ch]);
      if (need > span) {
        const uint8_t dir = (w[ch] >= 0.0f) ? 0 : 1;
        const float room = (span - WHEEL_DEADBAND_US[ch][dir]) * WHEEL_MPS_PER_US[ch][dir];
        const float s = room / fabs(w[ch]);
        if (s < scale) scale = s;
      }
    }
    saturated_ = (scale < 1.0f);
    write(wheelPulse(CH_LF, lf * scale, MOTOR_MIRROR_LEFT),
          wheelPulse(CH_RF, rf * scale, MOTOR_MIRROR_RIGHT),
          wheelPulse(CH_LR, lr * scale, MOTOR_MIRROR_LEFT),
          wheelPulse(CH_RR, rr * scale, MOTOR_MIRROR_RIGHT));
    applied_vx_ = req_vx * scale;
    applied_vy_ = req_vy * scale;
    applied_wz_ = req_wz * scale;
    return;
  }

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

  write(toPulse(lf, MOTOR_MIRROR_LEFT), toPulse(rf, MOTOR_MIRROR_RIGHT),
        toPulse(lr, MOTOR_MIRROR_LEFT), toPulse(rr, MOTOR_MIRROR_RIGHT));

  // Scaling all four wheels by one factor is exactly a uniform scaling of the
  // body twist, so the echo is just the request times that factor -- reported
  // before the sign trims and the channel rotation, in the caller's own frame.
  applied_vx_ = req_vx * scale;
  applied_vy_ = req_vy * scale;
  applied_wz_ = req_wz * scale;
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

float Drive::wheelOffsetUs(uint8_t ch, float speed) {
  const float mag = fabs(speed);
  if (mag < WHEEL_STOP_MPS) return 0.0f;
  const uint8_t dir = (speed >= 0.0f) ? 0 : 1;
  return WHEEL_DEADBAND_US[ch][dir] + mag / WHEEL_MPS_PER_US[ch][dir];
}

uint16_t Drive::wheelPulse(uint8_t ch, float speed, float mirror) {
  const float span = (float)(MOTOR_US_MAX - MOTOR_US_NEUTRAL);
  float off = wheelOffsetUs(ch, speed);
  if (off > span) off = span;
  if (speed < 0.0f) off = -off;
  const float signed_us = off * mirror;
  long us = (long)MOTOR_US_NEUTRAL + (long)(signed_us + (signed_us >= 0.0f ? 0.5f : -0.5f));
  if (us < (long)MOTOR_US_MIN) us = MOTOR_US_MIN;
  if (us > (long)MOTOR_US_MAX) us = MOTOR_US_MAX;
  return (uint16_t)us;
}
