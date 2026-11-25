#pragma once

// The claw piston and its autograb sensor (ports in config.hpp), shared by
// driver control and autons.

// Starts the autograb task. Call once from initialize(). The claw starts open.
void initClaw();

void clawGrab();     // close
void clawRelease();  // open
void clawToggle();
bool clawClosed();

// While armed, the claw closes the first time something is closer than
// AUTOGRAB_MM to the sensor, then disarms itself.
void autograbArm(bool armed);
bool autograbArmed();

// For grabbing on the move: arms autograb, and also closes the claw once the
// robot's centre (RW's odometry) is within radius_in of (x, y) in field
// coordinates. The sensor normally fires first; the odometry trigger is the
// backup for a piece the sensor misses (a cup's clear half), and the only one
// the simulator has (its sensor can't see pieces).
// lift_after: if 0 or more, the lift starts for that height as soon as the claw has shut.
void autograbArmNear(double x, double y, double radius_in, double lift_after = -1);
// How the last armed grab closed: 's' the sensor, 'o' odometry, 0 not yet. And when (pros::millis()).
char autograbHow();
unsigned autograbWhenMs();

// For autons: arms autograb and waits until it grabs or timeout_ms passes.
// Returns true if it grabbed; on a timeout it disarms and leaves the claw as is.
bool clawGrabWhenSeen(int timeout_ms);
// For autons that armed autograb before driving in: waits up to timeout_ms for
// it to fire (no wait if it already has), then disarms. True if the claw is closed.
bool autograbWait(int timeout_ms);
