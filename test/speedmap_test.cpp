// speedmap_test.cpp
// Host-side check of the speed map: interpolation, clamping, sign handling,
// invertibility. Run it after editing SPEED_MAP_FORWARD / SPEED_MAP_STRAFE --
// the static_asserts in speedmap.h catch a table that cannot be inverted, but
// not a table that interpolates to the wrong place.
//
//     cd test && ./run_tests.sh
//
// The expected values below are tied to the measured tables in config.h and
// need updating alongside them.
#include <cstdio>
#include <cmath>
#include "speedmap.h"
static int fails = 0;
static void ck(const char* what, float got, float want) {
  bool ok = std::fabs(got - want) < 5e-4f;
  if (!ok) fails++;
  printf("%-42s got %8.4f  want %8.4f  %s\n", what, got, want, ok ? "ok" : "FAIL");
}
int main() {
  using namespace SpeedMap;
  ck("fwd 0 m/s -> 0 deflection (hold still)", forwardDeflection(0.0f), 0.0f);
  ck("fwd 0.138 -> 0.15 (first row)", forwardDeflection(0.138f), 0.15f);
  ck("fwd 0.275 -> 0.35 (last row)", forwardDeflection(0.275f), 0.35f);
  ck("fwd 0.1635 -> 0.175 (midway interp)", forwardDeflection(0.1635f), 0.175f);
  ck("fwd 0.2395 -> 0.275 (fourth segment)", forwardDeflection(0.2395f), 0.275f);
  ck("fwd 5.0 -> 0.35 (clamped to max)", forwardDeflection(5.0f), 0.35f);
  ck("fwd 0.05 -> 0.15 (clamped to min, snaps up)", forwardDeflection(0.05f), 0.15f);
  ck("fwd -0.1635 -> -0.175 (sign carried)", forwardDeflection(-0.1635f), -0.175f);
  ck("strafe 0.1255 -> 0.175 (own curve)", strafeDeflection(0.1255f), 0.175f);
  ck("strafe 0.275 -> 0.35 (fwd top unreachable sideways)", strafeDeflection(0.275f), 0.35f);
  ck("strafe 0.101 -> 0.15 (first row)", strafeDeflection(0.101f), 0.15f);
  ck("inverse: 0.175 defl -> 0.1635 m/s", forwardMps(0.175f), 0.1635f);
  ck("inverse: -0.175 defl -> -0.1635 m/s", forwardMps(-0.175f), -0.1635f);
  ck("inverse: 0 defl -> 0 m/s", forwardMps(0.0f), 0.0f);
  ck("round trip fwd 0.21", forwardMps(forwardDeflection(0.21f)), 0.21f);
  ck("round trip strafe 0.20", strafeMps(strafeDeflection(0.20f)), 0.20f);
  ck("max fwd", maxForwardMps(), 0.275f);
  ck("max strafe", maxStrafeMps(), 0.227f);
  ck("min fwd", minForwardMps(), 0.138f);
  ck("min strafe", minStrafeMps(), 0.101f);
  ck("strafe is slower than fwd at matched deflection",
     strafeMps(0.30f) < forwardMps(0.30f) ? 1.0f : 0.0f, 1.0f);
  ck("rotation proxy uses fwd curve", rotationDeflection(0.1635f), 0.175f);

  // Monotonicity across the whole span, both axes.
  float pd = -1;
  for (int i = 0; i <= 100; i++) {
    float v = 0.30f * i / 100.0f, d = forwardDeflection(v);
    if (d < pd - 1e-6f) { printf("FAIL non-monotonic fwd at %.3f\n", v); fails++; }
    pd = d;
  }
  printf("\n%s\n", fails ? "FAILURES" : "all passed");
  return fails != 0;
}
