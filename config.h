// config.h
// Every number you might want to change, in one place. Pins and I2C address
// follow the Mega + sensor shield: SDA = 20, SCL = 21.
#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// Serial link to the Pi (Serial0 on the Mega)
// ---------------------------------------------------------------------------
// 500000 is an exact divisor of the Mega's 16 MHz clock (UBRR = 3, 0% error)
// and well inside what the on-board ATmega16U2 USB bridge will carry. Do not
// use 230400: it lands 3.5% off on this clock and drops characters.
// *** The Pi must be opened at this same rate. ***
constexpr unsigned long SERIAL_BAUD = 500000;

// How often a telemetry line is sent, Hz. Keep this at or below the IMU report
// rate (1e6 / IMU_REPORT_US) -- streaming faster only repeats samples.
constexpr uint16_t STREAM_RATE_HZ = 200;

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
// BNO085 report interval in microseconds, applied to every enabled report
// (game rotation vector, acceleration, gyro). 5000 = 200 Hz. The chip will
// take anything up to 400 Hz on each of these, but not all three at once at
// full rate, and the uplink only streams at STREAM_RATE_HZ, so there is no
// point going faster.
constexpr uint32_t IMU_REPORT_US = 5000;

// Which fused acceleration report to stream as ax/ay.
//   true  -> SH2_LINEAR_ACCELERATION, fused accel with gravity removed
//   false -> SH2_ACCELEROMETER, calibrated accel, gravity included
// Linear acceleration is the right one for a floor robot: any chassis tilt,
// suspension sag or hard cornering otherwise leaks a slice of the 9.81 m/s^2
// gravity vector into ax/ay, as a bias the Pi cannot tell from real motion.
constexpr bool IMU_ACCEL_GRAVITY_REMOVED = true;

// 0x4A by default, 0x4B if the DI/ADR pin is pulled high.
constexpr uint8_t IMU_I2C_ADDR = 0x4A;

// I2C bus clock, Hz. 400000 is Fast mode, the BNO085's documented maximum
// (datasheet 1.2.2 and 6.5.1). The 100 kHz default cannot carry three reports
// at 200 Hz: one batched transfer takes ~4.9 ms there, which does not fit in
// the 5 ms budget. At 400 kHz the same transfer is ~1.2 ms. Drop back to
// 100000 if the bus proves unreliable with long shield wiring -- and halve
// STREAM_RATE_HZ and double IMU_REPORT_US if you do.
constexpr uint32_t IMU_I2C_HZ = 400000;

// Reset pin, or -1 if not wired.
constexpr int8_t IMU_RESET_PIN = -1;

// +1 with the chip mounted face up. Set to -1 if the chip is mounted upside
// down, so that heading still increases anticlockwise (viewed from above).
// Verified +1 by rotation test: clockwise rotation drives heading negative.
constexpr float IMU_YAW_SIGN = 1.0f;

// ---------------------------------------------------------------------------
// Drive: mecanum chassis
// ---------------------------------------------------------------------------
// Motor PWM pins, by PHYSICAL chassis corner -- which is NOT the order the
// MechEng 706 sensor shield's wiring suggests. The shield's pin order is a
// quarter turn out from the chassis: measured 2026-10-12 by timing 1 m runs
// (motor_cal/RESULTS.md), the old assignment made +Vx strafe the bot LEFT and
// +Vy drive it BACKWARDS.
//
// Pin pulse deviations measured for the two runs, with the mirrors below
// applied, in pin order (46, 47, 50, 51):
//
//     (-, -, +, +)  ->  bot strafed left       (front pair vs rear pair)
//     (+, -, -, +)  ->  bot drove backwards    (left pair vs right pair)
//
// Textbook mecanum needs left-vs-right opposite for translation along x and
// front-vs-rear opposite for a strafe, and this is the assignment that makes
// those two patterns come out as forward and left -- the only one, given that
// MOTOR_MIRROR_LEFT applies to the physically left-hand pair. So the mixing
// and the mirror constants are untouched; the whole fix is which pin sits at
// which corner.
//
// Sign trims could not have fixed this. A quarter turn is not two sign flips.
constexpr uint8_t PIN_LEFT_FRONT  = 51;   // shield order said 46
constexpr uint8_t PIN_LEFT_REAR   = 46;   // shield order said 47
constexpr uint8_t PIN_RIGHT_REAR  = 47;   // shield order said 50
constexpr uint8_t PIN_RIGHT_FRONT = 50;   // shield order said 51

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

// Mecanum geometry: half-length + half-width of the wheel base, metres.
// 15 cm + 9 cm on this chassis. Scales how much wheel speed a given Wz costs.
constexpr float CHASSIS_L_PLUS_W = 0.24f;

// ---------------------------------------------------------------------------
// Speed map: measured velocity <-> wheel command
// ---------------------------------------------------------------------------
// This replaces the old WHEEL_MAX_MPS, which was a single linear scale factor
// and a placeholder guess at that. It could not have worked: the real curve is
// neither straight nor headed for the origin, and the two axes do not share
// one.
//
// Each row is one timed 1 m run: the speed achieved, and the wheel command
// that achieved it as a fraction of full pulse deflection. Rows must be sorted
// ascending and must be monotonic in BOTH columns, or the interpolation in
// speedmap.cpp cannot be inverted. speedmap.h static_asserts it.
//
// Measured 2026-10-12, 5% steps across the operating band, timed 1 m runs.
// Single trials -- motor_cal/RESULTS.md.
//
// Round 1's 40% and 60% rows are deliberately NOT merged in. The same physical
// wheel command came out 12% faster for a strafe and 10% slower for a drive
// between the two sessions, so merging them would put a step in the curve that
// is battery and floor, not chassis. These tables are one session throughout.
//
// They therefore stop at 35%, and so does the reachable speed: 0.275 m/s
// forward, 0.227 m/s sideways. Round 1 saw 0.327 m/s forward at 60%, but above
// the operating band and in the other session, so it is not in here. Extending
// the range means a fresh sweep out to 60% in one sitting.
struct SpeedPoint {
  float mps;    // measured body speed, m/s
  float defl;   // wheel command that produced it, fraction of full deflection
};

// Forward / reverse (+-Vx). Gains per 5% step: 0.052, 0.035, 0.030, 0.021 --
// the curve is still bending hard at the top of the band.
constexpr SpeedPoint SPEED_MAP_FORWARD[] = {
    {0.138f, 0.15f},
    {0.189f, 0.20f},
    {0.225f, 0.25f},
    {0.254f, 0.30f},
    {0.275f, 0.35f},
};

// Left / right strafe (+-Vy). 17-26% slower than forward at the same command,
// and the last step gains only 0.008 m/s: 35% is the knee, not a ceiling to
// push past. Measured, not modelled -- there is no STRAFE_GAIN.
constexpr SpeedPoint SPEED_MAP_STRAFE[] = {
    {0.101f, 0.15f},
    {0.150f, 0.20f},
    {0.187f, 0.25f},
    {0.219f, 0.30f},
    {0.227f, 0.35f},
};

// Ceiling on wheel command, as a fraction of full pulse deflection. The
// saturation in Drive scales to this instead of to 1.0, so no command can ever
// reach full deflection.
//
// 0.80 because 100% does not work: at full deflection the wheels do not turn
// correctly on either axis (measured 2026-10-12, both sweeps -- see
// motor_cal/RESULTS.md). Nothing above ~60% is useful anyway; measured speed is
// flat from 60% to 80% (0.327 -> 0.329 m/s forward) and a left strafe at 80%
// came out *slower* than at 60%, so the top of the range buys current draw and
// no speed. Raise this only with fresh measurements that justify it.
constexpr float MOTOR_MAX_DEFLECTION = 0.80f;

// There is deliberately no STRAFE_GAIN. A strafe is slower than a drive at the
// same wheel command -- 21% to 37% slower, measured -- because mecanum rollers
// slip sideways, and no gain can buy that speed back: scaling the request up
// just asks for wheel speed the chassis does not convert into motion. It is a
// limit of the drive, so it is reported as one. SPEED_MAP_STRAFE below says
// what a strafe can actually do, and a request past that saturates.
