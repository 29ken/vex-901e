#pragma once
#include "lemlib/api.hpp"
#include <vector>

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern lemlib::Chassis chassis;

struct MotorTemp {
    int port;
    double celsius;
    bool plugged;
};

void initDrive();

// One entry per motor in the group.
std::vector<MotorTemp> motorTemps(pros::MotorGroup& motors);
