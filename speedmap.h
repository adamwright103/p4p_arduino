// speedmap.h
// Measured velocity <-> wheel-command conversion, per axis.
//
// There are no encoders on this chassis, so this is the only thing that makes
// a commanded velocity mean anything in SI. It is a lookup table of timed 1 m
// runs (motor_cal/RESULTS.md), interpolated linearly, and it is per axis
// because the axes genuinely differ:
//
//   - The curve is strongly non-linear. A straight line through the measured
//     forward points does not pass through the origin: there is a deadband at
//     the bottom where the servos do not turn at all, and a plateau at the top
//     where more deflection buys no more speed.
//   - A strafe is 21-37% slower than a drive at the same wheel command,
//     because mecanum rollers slip sideways. That is a limit of the drive, not
//     an error to be corrected, so the strafe axis gets its own curve rather
//     than a gain on the forward one. Asking for 0.30 m/s sideways does not
//     get it; it saturates and says so.
//
// Both directions of each axis use the same curve. Servo response is symmetric
// about neutral, so a reverse or a right strafe is the mirror of its measured
// counterpart -- an assumption, and the one place this file is not measurement.
#pragma once
#include <Arduino.h>
#include "config.h"

// Table invariants, checked at compile time. The interpolation is read in both
// directions, so both columns have to rise together or the map cannot be
// inverted -- and a non-invertible map is exactly what you get by pasting in a
// row from the flat top of the curve, where more deflection gave less speed.
// A hand-edit that breaks this fails the build instead.
constexpr bool rises(const SpeedPoint* t, uint8_t n, uint8_t i) {
  return i >= n ? true
                : (t[i].mps > t[i - 1].mps && t[i].defl > t[i - 1].defl) &&
                      rises(t, n, i + 1);
}

constexpr uint8_t SPEED_MAP_FORWARD_N =
    sizeof(SPEED_MAP_FORWARD) / sizeof(SpeedPoint);
constexpr uint8_t SPEED_MAP_STRAFE_N =
    sizeof(SPEED_MAP_STRAFE) / sizeof(SpeedPoint);

static_assert(SPEED_MAP_FORWARD_N >= 2, "forward map needs two rows to interpolate");
static_assert(SPEED_MAP_STRAFE_N >= 2, "strafe map needs two rows to interpolate");
static_assert(rises(SPEED_MAP_FORWARD, SPEED_MAP_FORWARD_N, 1),
              "SPEED_MAP_FORWARD must rise in both columns -- drop any row from "
              "the flat top of the curve");
static_assert(rises(SPEED_MAP_STRAFE, SPEED_MAP_STRAFE_N, 1),
              "SPEED_MAP_STRAFE must rise in both columns -- drop any row from "
              "the flat top of the curve");
static_assert(SPEED_MAP_FORWARD[0].mps > 0.0f && SPEED_MAP_FORWARD[0].defl > 0.0f,
              "forward map rows must be positive; sign is applied separately");
static_assert(SPEED_MAP_STRAFE[0].mps > 0.0f && SPEED_MAP_STRAFE[0].defl > 0.0f,
              "strafe map rows must be positive; sign is applied separately");
static_assert(SPEED_MAP_FORWARD[SPEED_MAP_FORWARD_N - 1].defl <= MOTOR_MAX_DEFLECTION,
              "forward map asks for more deflection than MOTOR_MAX_DEFLECTION");
static_assert(SPEED_MAP_STRAFE[SPEED_MAP_STRAFE_N - 1].defl <= MOTOR_MAX_DEFLECTION,
              "strafe map asks for more deflection than MOTOR_MAX_DEFLECTION");

namespace SpeedMap {

// Velocity (m/s) -> wheel command, as a signed fraction of full deflection.
// Sign is carried through; magnitude is clamped to the measured range, so a
// request faster than the drive can go returns the fastest measured command
// rather than extrapolating off the end of the data.
float forwardDeflection(float mps);
float strafeDeflection(float mps);

// Wheel surface speed (m/s) -> wheel command, for the rotation term. Uses the
// forward curve: for a pure drive the wheel's contact patch speed and the
// body speed are the same thing, which makes the forward table a reasonable
// stand-in for a wheel-speed curve. It is a proxy, and an unverified one --
// rotation has never been measured on this chassis.
float rotationDeflection(float wheel_mps);

// The inverses, for reporting what was actually applied.
float forwardMps(float deflection);
float strafeMps(float deflection);

// The ends of the measured range. Anything outside is unreachable.
float maxForwardMps();
float maxStrafeMps();
float minForwardMps();   // slowest non-zero speed the drive will produce
float minStrafeMps();

}  // namespace SpeedMap
