/*
  p4p_arduino.ino

  Low-level driver for the P4P mecanum bot. Takes body-velocity commands from
  the Pi over USB serial and reports back the twist the chassis was actually
  given. See README.md for the full serial API.

  Downlink, from the Pi:   E | S | V,<vx>,<vy>,<wz>
  Uplink, at STREAM_RATE_HZ:        A,<vx>,<vy>,<wz>

  The uplink mirrors the downlink's shape on purpose: same tag-first layout,
  same comma separation, same three decimal places the Pi sends. One line is
  one twist, in SI, in the body frame -- and it is what was *applied*, after
  the speed map and after saturation, not the request echoed back.

  There is no IMU here any more. The BNO085 moved to the Pi, which owns it
  directly over its own I2C bus (p4p_imu in the conv-chart repo), so heading
  and acceleration are no longer this firmware's business.

  Files:  config.h        constants, including the measured speed-map tables
          timing.h        RateTimer, a non-blocking fixed-rate timer
          drive.h/.cpp    mecanum mixing and motor output
          speedmap.h/.cpp measured velocity <-> wheel command, per axis
          command.h/.cpp  serial command parser
          system_setup.*  everything setup() has to do
  Needs the "Servo" library.
*/
#include "config.h"
#include "timing.h"
#include "system_setup.h"

static RateTimer stream_timer(1000000UL / STREAM_RATE_HZ);
static RateTimer control_timer(1000000UL / CONTROL_RATE_HZ);

// One line of applied body twist, SI, in the Pi's own format. Three decimals
// because that is what the Pi's format_velocity() sends and far finer than an
// open-loop chassis with no encoders can deliver.
//
// Zero here means one of four things and does not distinguish them: disarmed,
// watchdog expired, a commanded V,0,0,0, or a command below the deadband that
// the map could not honour. The Pi knows which it asked for.
void printTelemetry() {
  Serial.print(F("A,"));
  Serial.print(drive.appliedVx(), 3);
  Serial.print(',');
  Serial.print(drive.appliedVy(), 3);
  Serial.print(',');
  Serial.println(drive.appliedWz(), 3);
}

void setup() {
  systemSetup();
}

void loop() {
  cmd.poll();                  // drain the RX buffer every pass; never blocks

  if (control_timer.due()) {   // CONTROL_RATE_HZ
    // The watchdog is checked here rather than in the parser, so that a flood
    // of inbound commands can never starve it and a silent Pi can never leave
    // a stale velocity latched on the motors.
    const bool timed_out = (cmd.ageMs() > CMD_TIMEOUT_MS);

    if (!cmd.armed() || timed_out) {
      drive.stop();
    } else {
      drive.setBodyVelocity(cmd.vx(), cmd.vy(), cmd.wz());
    }
  }

  if (stream_timer.due()) {    // STREAM_RATE_HZ
    printTelemetry();
  }
}
