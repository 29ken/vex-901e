#pragma once
#include <vector>
#include "main.h"
#include "rw-config.h"   // RW free functions: setPose, moveToPoint, turnToAngle, driveTo, etc.

// The drive motor groups that RW uses are declared in rw/devices.h as
// left_chassis / right_chassis. These aliases keep the rest of our code working.
extern pros::MotorGroup& leftMotors;
extern pros::MotorGroup& rightMotors;

struct MotorTemp {
    int port;
    double celsius;
    bool plugged;
};

void initDrive();

// Safety limits. Moves normally finish well before these.
inline constexpr int DRIVE_TIMEOUT_MS = 3000;
inline constexpr int TURN_TIMEOUT_MS = 2000;

// Relative moves for autons. driveInches(10) goes 10 in the way the robot is
// facing (negative backs up); turnDegrees(90) turns 90 clockwise. Both wait
// until they're done unless async is true.
void driveInches(float inches, int timeout = DRIVE_TIMEOUT_MS, bool async = false);
void turnDegrees(float degrees, int timeout = TURN_TIMEOUT_MS, bool async = false);

// One entry per motor in the group.
std::vector<MotorTemp> motorTemps(pros::MotorGroup& motors);
