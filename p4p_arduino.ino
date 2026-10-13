/*
  p4p_arduino.ino

  Low-level driver for the P4P mecanum bot. Takes body-velocity commands from
  the Pi over USB serial and streams IMU heading, the applied velocity, the
  request as received and the four motor pulse widths back. See README.md for
  the full serial API.

  Uplink, at STREAM_RATE_HZ:
      t_ms,heading_rad,yaw_rate_dps,vx,vy,wz,flags,resets,bad,
      req_vx,req_vy,req_wz,us_lf,us_rf,us_lr,us_rr        (one line)
  Downlink:
      E | S | V,<vx>,<vy>,<wz>

  Files:  config.h        constants
          timing.h        RateTimer, a non-blocking fixed-rate timer
          imu.h/.cpp      BNO085 wrapper
          drive.h/.cpp    mecanum mixing and motor output
          command.h/.cpp  serial command parser
          system_setup.*  everything setup() has to do
  Needs the "Adafruit BNO08x" library (and its dependency Adafruit BusIO).
*/
#include "config.h"
#include "timing.h"
#include "system_setup.h"

static RateTimer stream_timer(1000000UL / STREAM_RATE_HZ);
static RateTimer control_timer(1000000UL / CONTROL_RATE_HZ);

// Telemetry flag bits.
static constexpr uint8_t FLAG_ARMED     = 0x01;
static constexpr uint8_t FLAG_TIMEOUT   = 0x02;  // no V within CMD_TIMEOUT_MS
static constexpr uint8_t FLAG_SATURATED = 0x04;  // command scaled down to fit

static bool timed_out = false;

void printTelemetry() {
  uint8_t flags = 0;
  if (cmd.armed())       flags |= FLAG_ARMED;
  if (timed_out)         flags |= FLAG_TIMEOUT;
  if (drive.saturated()) flags |= FLAG_SATURATED;

  Serial.print(imu.lastSampleMs());
  Serial.print(',');
  Serial.print(imu.heading(), 4);
  Serial.print(',');
  Serial.print(degrees(imu.yawRate()), 1);
  Serial.print(',');
  Serial.print(drive.appliedVx(), 3);
  Serial.print(',');
  Serial.print(drive.appliedVy(), 3);
  Serial.print(',');
  Serial.print(drive.appliedWz(), 3);
  Serial.print(',');
  Serial.print(flags);
  Serial.print(',');
  Serial.print(imu.resetCount());
  Serial.print(',');
  Serial.print(cmd.badCount());
  // For calibrating the drive: the last V exactly as received (before any
  // saturation scaling, and still shown while disarmed or timed out), and the
  // pulse widths the motors were given. A row is 78 bytes with no IMU on the
  // Mega, 107 at most: 34-46% of the link at 50 Hz. Past the 64-byte TX
  // buffer print() waits for the UART, 1-4 ms a row; nothing here minds.
  Serial.print(',');
  Serial.print(cmd.vx(), 3);
  Serial.print(',');
  Serial.print(cmd.vy(), 3);
  Serial.print(',');
  Serial.print(cmd.wz(), 3);
  for (uint8_t ch = 0; ch < Drive::NUM_CHANNELS; ch++) {
    Serial.print(',');
    Serial.print(drive.pulseUs(ch));
  }
  Serial.println();
}

void setup() {
  systemSetup();
}

void loop() {
  imu.poll();                  // drain the BNO every pass; never blocks
  cmd.poll();                  // drain the RX buffer every pass; never blocks

  if (control_timer.due()) {   // CONTROL_RATE_HZ
    // The watchdog is checked here rather than in the parser, so that a flood
    // of inbound commands can never starve it and a silent Pi can never leave
    // a stale velocity latched on the motors.
    timed_out = (cmd.ageMs() > CMD_TIMEOUT_MS);

    if (!cmd.armed() || timed_out) {
      drive.stop();
    } else {
      drive.setBodyVelocity(cmd.vx(), cmd.vy(), cmd.wz());
    }
  }

  if (stream_timer.due()) {    // STREAM_RATE_HZ
    printTelemetry();
  }

  // Keep delay() out of here.
}
