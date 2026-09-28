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

// Mecanum geometry: half-length + half-width of the wheel base, metres.
// 15 cm + 9 cm on this chassis. Scales how much wheel speed a given Wz costs.
constexpr float CHASSIS_L_PLUS_W = 0.24f;

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
// correction.
constexpr float STRAFE_GAIN = 1.0f;
