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

  Serial.println(F("# BNO085 ready"));
  Serial.println(F("# DISARMED -- send E to enable"));
  Serial.println(F("t_ms,heading_rad,yaw_rate_dps,vx,vy,wz,flags,resets,bad"));
}
