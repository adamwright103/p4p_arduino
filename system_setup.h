// system_setup.h
// Everything that has to exist before loop() runs. Add new peripherals here
// and bring them up inside systemSetup().
#pragma once
#include "imu.h"
#include "drive.h"
#include "command.h"

extern Imu imu;
extern Drive drive;
extern Command cmd;

void systemSetup();
