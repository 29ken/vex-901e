#pragma once
#include "lemlib/api.hpp"

namespace drive {

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern lemlib::Chassis chassis;

void init();

} // namespace drive
