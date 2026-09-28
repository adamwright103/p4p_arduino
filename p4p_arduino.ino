/*
  p4p_arduino.ino

  Streams the BNO085 heading over USB serial as CSV at STREAM_RATE_HZ:
      t_ms,heading_rad,yaw_rate_dps,resets
  heading_rad is continuous and unwrapped (what the Pi-side filter uses);
  wrap it to +/-180 on the Pi if a human needs to read it. resets counts
  spontaneous BNO resets (should stay at 0).

  Files:  config.h        constants
          timing.h        RateTimer, a non-blocking fixed-rate timer
          imu.h/.cpp      BNO085 wrapper
          system_setup.*  everything setup() has to do
  Needs the "Adafruit BNO08x" library (and its dependency Adafruit BusIO).
*/
#include "config.h"
#include "timing.h"
#include "system_setup.h"

static RateTimer stream_timer(1000000UL / STREAM_RATE_HZ);

void printHeading() {
  Serial.print(imu.lastSampleMs());
  Serial.print(',');
  Serial.print(imu.heading(), 4);
  Serial.print(',');
  Serial.print(degrees(imu.yawRate()), 1);
  Serial.print(',');
  Serial.println(imu.resetCount());
}

void setup() {
  systemSetup();
}

void loop() {
  imu.poll();                 // drain the BNO every pass; never blocks

  if (stream_timer.due()) {   // STREAM_RATE_HZ
    printHeading();
  }

  // Later: a 50 Hz control task on its own RateTimer, the CMD_VEL parser,
  // motor mixing, watchdog. Keep delay() out of here.
}