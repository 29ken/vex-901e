#pragma once

// Distance-sensor watch (step 2 of the localisation plan), WATCH ONLY: while
// the robot drives, it checks each distance sensor against the wall it should
// be seeing and works out how far off the robot's position looks. It never
// changes the position. It prints what it would have done to the PROS
// terminal, so a practice run shows whether its corrections can be trusted
// before anything is switched on.
//
// A reading is believed only when all of these hold (the numbers are in watch.cpp):
//  - the beam meets its wall within 12 degrees of square, under 30 in away
//    (12 in for the back sensor, which starts catching the toggle past that);
//  - no goal, wall stack, loose Cup+Pin or loader that should still be there
//    is inside the beam's cone;
//  - it isn't the front sensor with the claw closed (the piece blocks it);
//  - the robot isn't turning fast (a reading is about 40 ms old);
//  - it is within 3 in of what the robot's own position predicts.
// The reading is compared with where the robot was 40 ms earlier.

// Starts the background task. Call once from initialize().
void initWatch();
// On for an auton, off after it. Turning it on forgets earlier runs.
void watchEnable(bool on);
// OFF by default. On: each believed reading moves the robot's position 20% of
// the way toward what the sensor says, instead of only logging. Don't turn it
// on for a match until practice logs show the watch is right.
void watchApply(bool on);
// A piece the robot has taken (field coordinates): no longer in the way of a beam.
void watchTaken(double x, double y);
