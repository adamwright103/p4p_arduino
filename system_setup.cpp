// system_setup.cpp
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "system_setup.h"

Imu imu(IMU_RESET_PIN, IMU_YAW_SIGN);

void systemSetup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println(F("# p4p_arduino starting"));

  while (!imu.begin(IMU_I2C_ADDR, IMU_REPORT_US)) {
    Serial.println(F("# BNO085 not found: check SDA=20, SCL=21, power, address"));
    delay(1000);
  }

#if defined(WIRE_HAS_TIMEOUT)
  // A hung I2C bus resets itself after 3 ms instead of freezing the loop.
  Wire.setWireTimeout(3000, true);
#endif

  Serial.println(F("# BNO085 ready"));
  Serial.println(F("t_ms,heading_rad,yaw_rate_dps,resets"));
}
