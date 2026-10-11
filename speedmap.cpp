// speedmap.cpp
#include "speedmap.h"

namespace {

// Linear interpolation of a monotonic table, clamped at both ends. `from` and
// `to` pick which column is the input: the tables are stored once and read in
// both directions.
float interp(const SpeedPoint* t, uint8_t n, float x, bool mps_in) {
  const float first = mps_in ? t[0].mps : t[0].defl;
  const float last = mps_in ? t[n - 1].mps : t[n - 1].defl;
  const float out_first = mps_in ? t[0].defl : t[0].mps;
  const float out_last = mps_in ? t[n - 1].defl : t[n - 1].mps;

  if (x <= first) return out_first;
  if (x >= last) return out_last;

  for (uint8_t i = 1; i < n; i++) {
    const float hi = mps_in ? t[i].mps : t[i].defl;
    if (x > hi) continue;
    const float lo = mps_in ? t[i - 1].mps : t[i - 1].defl;
    const float out_lo = mps_in ? t[i - 1].defl : t[i - 1].mps;
    const float out_hi = mps_in ? t[i].defl : t[i].mps;
    const float span = hi - lo;
    if (span <= 0.0f) return out_hi;        // duplicate row: take the upper
    return out_lo + (out_hi - out_lo) * (x - lo) / span;
  }
  return out_last;
}

// Magnitude through the table, sign reapplied. A request of exactly zero is
// zero deflection -- "hold station" has to mean stationary, not "the slowest
// thing the drive can do".
float signedLookup(const SpeedPoint* t, uint8_t n, float mps) {
  if (mps == 0.0f) return 0.0f;
  const float d = interp(t, n, fabs(mps), true);
  return (mps < 0.0f) ? -d : d;
}

float signedInverse(const SpeedPoint* t, uint8_t n, float defl) {
  if (defl == 0.0f) return 0.0f;
  const float v = interp(t, n, fabs(defl), false);
  return (defl < 0.0f) ? -v : v;
}

}  // namespace

namespace SpeedMap {

float forwardDeflection(float mps) {
  return signedLookup(SPEED_MAP_FORWARD, SPEED_MAP_FORWARD_N, mps);
}
float strafeDeflection(float mps) {
  return signedLookup(SPEED_MAP_STRAFE, SPEED_MAP_STRAFE_N, mps);
}
float rotationDeflection(float wheel_mps) {
  return signedLookup(SPEED_MAP_FORWARD, SPEED_MAP_FORWARD_N, wheel_mps);
}
float forwardMps(float deflection) {
  return signedInverse(SPEED_MAP_FORWARD, SPEED_MAP_FORWARD_N, deflection);
}
float strafeMps(float deflection) {
  return signedInverse(SPEED_MAP_STRAFE, SPEED_MAP_STRAFE_N, deflection);
}

float maxForwardMps() { return SPEED_MAP_FORWARD[SPEED_MAP_FORWARD_N - 1].mps; }
float maxStrafeMps() { return SPEED_MAP_STRAFE[SPEED_MAP_STRAFE_N - 1].mps; }
float minForwardMps() { return SPEED_MAP_FORWARD[0].mps; }
float minStrafeMps() { return SPEED_MAP_STRAFE[0].mps; }

}  // namespace SpeedMap
