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

// For autons: arms autograb and waits until it grabs or timeout_ms passes.
// Returns true if it grabbed; on a timeout it disarms and leaves the claw as is.
bool clawGrabWhenSeen(int timeout_ms);
// For autons that armed autograb before driving in: waits up to timeout_ms for
// it to fire (no wait if it already has), then disarms. True if the claw is closed.
bool autograbWait(int timeout_ms);
