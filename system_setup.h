// system_setup.h
// Everything that has to exist before loop() runs. Add new peripherals here
// (motors, serial parser, ...) and bring them up inside systemSetup().
#pragma once
#include "imu.h"

extern Imu imu;

void systemSetup();
