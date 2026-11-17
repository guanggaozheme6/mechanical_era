#pragma once

#include "device/drive_control.hpp"
#include "device/arm_control.hpp"

void init_outputs();
void apply_motors(DriveCommand command);
void apply_servos(ArmCommand command);
