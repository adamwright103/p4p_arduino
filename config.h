// config.h
// Every number you might want to change, in one place. Pins and I2C address
// follow the Mega + sensor shield: SDA = 20, SCL = 21.
#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// Serial link to the Pi (Serial0 on the Mega)
// ---------------------------------------------------------------------------
constexpr unsigned long SERIAL_BAUD = 115200;

// How often a telemetry line is sent, Hz. The BNO is polled far more often.
constexpr uint16_t STREAM_RATE_HZ = 50;

// How often the motor outputs are recomputed and the watchdog is checked, Hz.
constexpr uint16_t CONTROL_RATE_HZ = 50;

// Longest gap between V commands before the motors are zeroed. The bot stays
// armed across a timeout -- only an explicit S disarms it.
constexpr uint32_t CMD_TIMEOUT_MS = 400;

// Longest accepted command line, bytes. "V,-1.234567,-1.234567,-1.234567" is
// 32, so this leaves room.
constexpr uint8_t CMD_BUF_LEN = 48;

// ---------------------------------------------------------------------------
// IMU (BNO085)
// ---------------------------------------------------------------------------
// BNO085 report interval in microseconds. 10000 = 100 Hz.
constexpr uint32_t IMU_REPORT_US = 10000;

// 0x4A by default, 0x4B if the DI/ADR pin is pulled high.
constexpr uint8_t IMU_I2C_ADDR = 0x4A;

// Reset pin, or -1 if not wired.
constexpr int8_t IMU_RESET_PIN = -1;

// How many times setup() looks for the BNO, 250 ms apart, before carrying on
// without it. The BNO085 now sits on the Pi's I2C bus, so a Mega without one
// is normal: the heading and yaw-rate columns then read 0.
constexpr uint8_t IMU_BEGIN_ATTEMPTS = 4;

// +1 with the chip mounted face up. Set to -1 if the chip is mounted upside
// down, so that heading still increases anticlockwise (viewed from above).
// Verified +1 by rotation test: clockwise rotation drives heading negative.
constexpr float IMU_YAW_SIGN = 1.0f;

// ---------------------------------------------------------------------------
// Drive: mecanum chassis
// ---------------------------------------------------------------------------
// Motor PWM pins, from the MechEng 706 sensor shield wiring.
constexpr uint8_t PIN_LEFT_FRONT  = 46;
constexpr uint8_t PIN_LEFT_REAR   = 47;
constexpr uint8_t PIN_RIGHT_REAR  = 50;
constexpr uint8_t PIN_RIGHT_FRONT = 51;

// Continuous-rotation servo pulse widths, microseconds. Neutral must be the
// midpoint of min and max or full scale will be asymmetric.
constexpr uint16_t MOTOR_US_MIN     = 700;
constexpr uint16_t MOTOR_US_NEUTRAL = 1500;
constexpr uint16_t MOTOR_US_MAX     = 2300;

// The left-hand motors are mounted mirrored, so the same pulse width drives
// them the opposite way in the body frame. Mixing below is written as textbook
// kinematics; this is where the hardware quirk is applied, on its own, so it
// stays visible.
constexpr float MOTOR_MIRROR_LEFT  = -1.0f;
constexpr float MOTOR_MIRROR_RIGHT =  1.0f;

// Per-axis direction trim for bring-up. If the bot drives backwards, strafes
// the wrong way, or spins the wrong way, flip the offending one to -1 rather
// than editing the mixing. Expected to be all +1 once the wiring is confirmed.
// See "Bring-up" in README.md for the 4-step check.
constexpr float DRIVE_VX_SIGN = 1.0f;   // +Vx should drive forward
constexpr float DRIVE_VY_SIGN = 1.0f;   // +Vy should strafe LEFT
constexpr float DRIVE_WZ_SIGN = 1.0f;   // +Wz should rotate ANTICLOCKWISE

// The four motor channels sit one wheel round from where the mixing assumes.
// Measured on the robot 2026-10-13 (one axis at a time on the floor, and a
// docking bag): a forward request drove it LEFT, a left request drove it BACK,
// and an anticlockwise one turned it CLOCKWISE. The sign trims above cannot
// express that, since it swaps the axes. With this true, setBodyVelocity turns
// each request into the mixing's frame, (vx, vy, wz) -> (vy, -vx, -wz), which
// cancels it exactly; the trims and STRAFE_GAIN still act on the body's own
// axes, before the turn. For X rollers it matches this wiring: the LEFT_FRONT
// channel drives the front-right wheel, RIGHT_FRONT the rear-right (reversed),
// LEFT_REAR the front-left (reversed) and RIGHT_REAR the rear-left. The proper
// fix is to reassign the four PIN_* and the mirror flags; then set this false.
constexpr bool DRIVE_CHANNELS_ROTATED = true;

// Mecanum geometry: half-length + half-width of the wheel base, metres. Scales
// how much wheel speed a given Wz costs. The chassis measures 76.5 + 89 mm
// (153 x 178 mm between roller centres, 2026-10-13), but the drive calibration
// of 2026-10-13 found the robot turning as if this were 0.1124 m (the rollers
// slip). The effective value is the one that makes a commanded Wz come out
// right, so that is what is used. Refitted from two runs: 0.1384.
constexpr float CHASSIS_L_PLUS_W = 0.1384f;

// *** UNCALIBRATED -- see "Calibration" in README.md ***
// Wheel surface speed at full pulse deflection, m/s. This is the single
// constant that turns SI velocity into PWM. Everything else follows from it:
//     max forward / strafe = WHEEL_MAX_MPS
//     max rotation         = WHEEL_MAX_MPS / CHASSIS_L_PLUS_W
// The value below is a placeholder guess. Until it is measured, commanded
// velocities are proportional but not accurate.
constexpr float WHEEL_MAX_MPS = 0.50f;

// Mecanum wheels slip more sideways than forwards, so a commanded strafe
// undershoots. Raise above 1.0 to compensate once measured. 1.0 = no
// correction. Measured 2026-10-13 (two runs): sideways the chassis makes 0.77
// of what the same wheel speeds make forwards, so 1 / 0.77.
constexpr float STRAFE_GAIN = 1.30f;

// Per-wheel calibration (2026-10-13). Each motor does nothing until its pulse is
// some distance from neutral, and that distance differs by channel and by
// direction; past it, each wheel speeds up by its own m/s per microsecond,
// which also differs by direction (the front-right wheel's by 1.6 times). One
// constant for all four (WHEEL_MAX_MPS) left small requests in the deadband and,
// with two motors starting late in one direction, turned every strafe into a
// clockwise turn as well. With DRIVE_WHEEL_CAL true, the mixing asks each wheel
// for a speed in m/s and wheelPulse() inverts that wheel's own response: the
// pulse starts at the wheel's deadband and adds 1 / WHEEL_MPS_PER_US
// microseconds per m/s. Fitted (controller_ws tools/drive_wheel_fit.py
// --gain-per-direction) from two calibration runs together, 60 single-axis
// steps from rest (drivecal_20261013_043214 on the single-constant firmware,
// drivecal_20261013_052711 on the first per-wheel constants, with the pulses
// logged), through the mixing below: each step's speed from the accelerometer
// (the board poses misread sideways motion when ambiguous) and its turn rate
// from the gyro; see "Calibration" in README.md. Rows are the
// channels LF, RF, LR, RR (the PIN_* names, not the wheels they drive, see
// DRIVE_CHANNELS_ROTATED); [0] is the mixing's positive sense, [1] negative.
// false restores the single-constant mapping through WHEEL_MAX_MPS.
constexpr bool DRIVE_WHEEL_CAL = true;
constexpr float WHEEL_DEADBAND_US[4][2] = {
  {76.0f, 58.2f}, {51.2f, 55.5f}, {83.7f, 55.2f}, {63.9f, 54.5f}};
constexpr float WHEEL_MPS_PER_US[4][2] = {
  {1.838e-3f, 1.124e-3f}, {1.735e-3f, 1.613e-3f}, {1.482e-3f, 1.804e-3f}, {1.592e-3f, 1.510e-3f}};
// Strafing turns the chassis: with the wheels calibrated, a strafe still turned
// it about -0.52 rad/s per m/s (clockwise going left; 2026-10-13, ten strafe
// steps of 0.02-0.15 m/s, gyro). No per-wheel speed explains it (the rollers
// must grip differently front and rear when sliding), so it is cancelled at
// the request: each strafe adds this much turn, rad/s per m/s, before mixing.
// 0 turns it off.
constexpr float STRAFE_YAW_FF = 0.52f;

// A wheel asked for less than this is held at neutral, so a near-zero request
// does not sit on the edge of its deadband. Small: 0.5 mm/s is 0.005 rad/s of
// turn, and the MPC's heading corrections are often a few hundredths of that.
constexpr float WHEEL_STOP_MPS = 0.0005f;
