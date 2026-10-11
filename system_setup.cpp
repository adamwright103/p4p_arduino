// system_setup.cpp
#include <Arduino.h>
#include "config.h"
#include "system_setup.h"
#include "speedmap.h"

Drive drive;
Command cmd;

void systemSetup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println(F("# p4p_arduino starting"));

  // Motors first and unconditionally, so four servo pins are never left
  // floating while the banner prints.
  drive.begin();

  // What the drive can actually do, straight off the speed map. Worth saying
  // out loud at boot: the limits are well below what a V may ask for, and a
  // saturated command is not visible in telemetry.
  Serial.print(F("# speed range, forward "));
  Serial.print(SpeedMap::minForwardMps(), 3);
  Serial.print(F(" to "));
  Serial.print(SpeedMap::maxForwardMps(), 3);
  Serial.print(F(" m/s, strafe "));
  Serial.print(SpeedMap::minStrafeMps(), 3);
  Serial.print(F(" to "));
  Serial.print(SpeedMap::maxStrafeMps(), 3);
  Serial.println(F(" m/s"));
  Serial.println(F("# outside that: faster saturates, slower snaps up, zero is zero"));

  Serial.println(F("# DISARMED -- send E to enable"));
}
