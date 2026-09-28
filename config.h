// config.h
// Every number you might want to change, in one place. Pins and I2C address
// follow the Mega + sensor shield: SDA = 20, SCL = 21.
#pragma once
#include <stdint.h>

// USB serial link to the Pi (Serial0 on the Mega).
constexpr unsigned long SERIAL_BAUD = 115200;

// How often a heading line is sent, Hz. The BNO is polled far more often.
constexpr uint16_t STREAM_RATE_HZ = 50;

// BNO085 report interval in microseconds. 10000 = 100 Hz.
constexpr uint32_t IMU_REPORT_US = 10000;

// 0x4A by default, 0x4B if the DI/ADR pin is pulled high.
constexpr uint8_t IMU_I2C_ADDR = 0x4A;

// Reset pin, or -1 if not wired.
constexpr int8_t IMU_RESET_PIN = -1;

// +1 with the chip mounted face up. Set to -1 if the chip is mounted upside
// down, so that heading still increases anticlockwise (viewed from above).
constexpr float IMU_YAW_SIGN = 1.0f;
