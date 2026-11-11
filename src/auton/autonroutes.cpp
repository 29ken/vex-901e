// Autonomous routes using RW-Template functions.
//
// RW API reference (from include/rw/motor-control.h):
//   setPose(x, y, heading_deg)          — set odometry position
//   moveToPoint(x, y, dir, timeout_ms)  — PID drive to field coordinate
//                                          dir: 1 = forwards, -1 = reverse
//   turnToPoint(x, y, dir, timeout_ms)  — turn to face a coordinate
//   turnToAngle(angle, timeout_ms)      — turn to absolute heading
//   driveTo(distance_in, timeout_ms)    — drive straight by distance
//   boomerang(x, y, dir, heading, dlead, timeout_ms)

#include "autons.hpp"
#include "drivetrain.hpp"

static void none() {}

static void toggle() {
    moveToPoint(-64, -8, 1, 1000);
    moveToPoint(-63, -8, 1, 1000);
}

static void redRight() {
    setPose(-63, -8, 270);
    moveToPoint(-63, -8, 1, 2000);
}

static void redRight2pin() {
    // starting position
    setPose(-7, -64, 180);

    // toggle
    moveToPoint(-7, -65, 1, 1000);

    // head to red goal + score 1 pin
    moveToPoint(-16, -56, 1, 1000);
    moveToPoint(-24, -56, 1, 1000);

    // get pin + cup
    moveToPoint(-24, -62, 1, 1000);

    // head to red goal + score 2nd pin, +1 pin + cup
    moveToPoint(-24, -59, 1, 1000);
}

static void redLeft4pin() {
    // initial pos
    setPose(-63, -8, 270);

    // toggle
    moveToPoint(-64, -8, 1, 1000);
    moveToPoint(-56, -16, 1, 1000);

    // head to red goal + score 1 pin
    moveToPoint(-56, -24, 1, 1000);

    // grab pin + cup
    moveToPoint(-61, -24, 1, 1000);

    // head to red goal + score 2nd pin + 1 pin + cup
    moveToPoint(-58, -24, 1, 1000);

    // head to other side (black goal left)
    moveToPoint(-57, 24, 1, 1000);

    // collect pin + cup
    moveToPoint(-61, 24, 1, 1000);

    // score black goal + 1 pin + cup
    moveToPoint(-59, 24, 1, 1000);

    // go to other side collect pin + cup
    moveToPoint(-29, -17, 1, 1000);

    // score red goal + 1 pin + cup
    moveToPoint(-38, -20, 1, 1000);
}

static void redLeft5pin() {
    // initial pos
    setPose(-63, -8, 270);

    // Toggle
    moveToPoint(-66, -8, 1, 1000);
    moveToPoint(-63, -8, 1, 1000);

    // head to red goal
    moveToPoint(-54, -17, 1, 1000);
    moveToPoint(-62, -24, 1, 1000);

    // release claw
    moveToPoint(-57, -24, 1, 1000);

    // head to match load side's pin + claw
    moveToPoint(-52, -41, 1, 1000);

    // red goal + unclaw
    moveToPoint(-50, -33, 1, 1000);

    // head to other outside pin + claw
    moveToPoint(-31, -26, 1, 1000);

    // head to black goal + unclaw
    moveToPoint(-46, 16, 1, 1000);

    // head to left pin
    moveToPoint(-63, 22, 1, 1000);

    // head to black goal + unclaw
    moveToPoint(-57, 23, 1, 1000);
}

static void blueLeft() {}

static void blueRight() {}

static void skills() {
    setPose(-63, -9, 270);

    moveToPoint(-64, -9, 1, 1000);
    moveToPoint(-57, -16, 1, 1000);
    moveToPoint(-58, -24, 1, 1000);
    moveToPoint(-61, -24, 1, 1000);
    moveToPoint(-56, -24, 1, 1000);
    moveToPoint(-56, -60, 1, 1000);
    moveToPoint(-61, -60, 1, 1000);
    moveToPoint(-48, -35, 1, 1000);
    moveToPoint(-61, -60, 1, 1000);
    moveToPoint(-35, -51, 1, 1000);
    moveToPoint(-40, -50, 1, 1000);
    moveToPoint(-35, -49, 1, 1000);
    moveToPoint(-60, -59, 1, 1000);
    moveToPoint(-24, -62, 1, 1000);
    moveToPoint(-24, -59, 1, 1000);
} // X 3 pin matchload
