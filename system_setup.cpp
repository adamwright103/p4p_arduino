// system_setup.cpp
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "system_setup.h"
#include "speedmap.h"

Imu imu(IMU_RESET_PIN, IMU_YAW_SIGN);
Drive drive;
Command cmd;

void systemSetup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println(F("# p4p_arduino starting"));

  // Motors first and unconditionally: the IMU bring-up below can block for a
  // long time, and four unattached servo pins must not be left floating while
  // it does.
  drive.begin();

#if defined(WIRE_HAS_TIMEOUT)
  // A hung I2C bus resets itself after 3 ms instead of freezing the loop. Set
  // before the first transaction, so the begin loop is covered too.
  Wire.setWireTimeout(3000, true);
#endif

  // The BNO needs ~100 ms after power-up before it answers on I2C. Without
  // this the first attempt always fails and every boot log carries a spurious
  // "not found".
  delay(150);

  while (!imu.begin(IMU_I2C_ADDR, IMU_REPORT_US)) {
    Serial.println(F("# BNO085 not found: check SDA=20, SCL=21, power, address"));
    delay(1000);
  }

  // Fast mode, *after* imu.begin(): Adafruit_I2CDevice::begin() calls
  // Wire.begin(), which resets the prescaler to 100 kHz, so setting the clock
  // any earlier would be undone. A spontaneous BNO reset re-enables reports
  // without touching Wire, so the clock survives that.
  Wire.setClock(IMU_I2C_HZ);

  Serial.println(F("# BNO085 ready"));

  // What the drive can actually do, straight off the speed map. Worth saying
  // out loud at boot: the limits are well below what a V command may ask for,
  // and a saturated command is no longer visible in telemetry.
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
  Serial.println(F("t_ms,heading_rad,ax,ay,yaw_rate_dps"));
}
