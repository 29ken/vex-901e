#pragma once
#include <vector>
#include "main.h"
#include "rw-config.h"  // RW: left_chassis, right_chassis, controller_1, moveToPoint, turnToAngle, ...

// Puts the drive in coast. Call once from initialize().
void initDrive();

// Arcade drive from the sticks, shaped by the driving-feel settings in
// config.hpp. Call every loop in opcontrol().
void driveControl();

// The driver's brake choice from the controller menu. Autons switch to hold,
// so opcontrol() puts this back.
extern bool driverBrakeHold;
void applyDriverBrakes();

// Safety limits. Moves normally finish well before these.
inline constexpr int DRIVE_TIMEOUT_MS = 3000;
inline constexpr int TURN_TIMEOUT_MS = 2000;

// Relative moves for autons. driveInches(10) goes 10 in the way the robot is
// facing (negative backs up); turnDegrees(90) turns 90 clockwise. Both wait
// until they're done unless async is true.
void driveInches(float inches, int timeout = DRIVE_TIMEOUT_MS, bool async = false);
void turnDegrees(float degrees, int timeout = TURN_TIMEOUT_MS, bool async = false);

struct MotorTemp {
    int port;
    double celsius;
    bool plugged;
};
// One entry per motor in the group, for the screens.
std::vector<MotorTemp> motorTemps(pros::MotorGroup& motors);
