#pragma once
#include "api.h"

extern pros::Controller controller;

// The driver's brake choice from the controller menu. Autons switch to hold,
// so opcontrol() puts this back.
extern bool driverBrakeHold;
void applyDriverBrakes();

// Call every loop in opcontrol().
void driveControl();
