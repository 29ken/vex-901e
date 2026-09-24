#pragma once
#include "api.h"

namespace controls {

extern pros::Controller controller;

// Call every loop in opcontrol().
void driveControl();

} // namespace controls
