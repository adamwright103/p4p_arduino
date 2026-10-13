// system_setup.cpp
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "system_setup.h"

Imu imu(IMU_RESET_PIN, IMU_YAW_SIGN);
Drive drive;
Command cmd;

void systemSetup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println(F("# p4p_arduino starting"));

  // Motors first and unconditionally: the IMU bring-up below can block for
  // about a second, and four unattached servo pins must not be left floating
  // while it does.
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

  // The BNO085 now sits on the Pi's I2C bus, so a Mega without one is normal.
  // Look for it briefly, then carry on regardless: waiting for ever would leave
  // the bot deaf to E, S and V, with no telemetry, and the motors need no IMU.
  bool imu_ok = imu.begin(IMU_I2C_ADDR, IMU_REPORT_US);
  for (uint8_t i = 1; i < IMU_BEGIN_ATTEMPTS && !imu_ok; i++) {
    delay(250);
    imu_ok = imu.begin(IMU_I2C_ADDR, IMU_REPORT_US);
  }

  if (imu_ok) {
    Serial.println(F("# BNO085 ready"));
  } else {
    Serial.println(F("# BNO085 not found: running without it, heading and yaw rate read 0"));
  }
  Serial.println(F("# DISARMED -- send E to enable"));
  Serial.println(F("t_ms,heading_rad,yaw_rate_dps,vx,vy,wz,flags,resets,bad,"
                   "req_vx,req_vy,req_wz,us_lf,us_rf,us_lr,us_rr"));
}
