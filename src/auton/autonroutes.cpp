// Autonomous routes for the V4 robot.
//
// Coordinates are RW's field-centred inches (the walls are at +-70.25) with
// compass headings (0 = +y, 90 = +x). Every route is written for red. Red's two
// Quadrants are the left one (x < 0, its toggle in the middle of the left wall)
// and the bottom one (y < 0, toggle in the middle of the bottom wall), and red
// stays below the diagonal Autonomous Line (x + y < 0). Blue's Quadrants are
// red's turned 180 degrees about the field centre, so the blue autons run the
// same routes turned that way (see at()); turns stay the same direction.
//
// Goal, wall-stack and loader positions are RW's Override field model
// (sim/sim-engine.cpp). The loose Cup+Pin stacks were measured off an overhead
// match video (youtube.com/shorts/VCsaetCuP-s, mapped onto the field with the
// goals as reference points, about +-3 in) and turned 180 degrees to red's side.
// Robot numbers are from the V4 CAD.
//
// What these routes can't do yet:
//  - The toggle flapper isn't wired (see toggle.hpp): setToggle() only waits.
//  - Every lift height is a placeholder (config.hpp) until practice.
//  - The simulator can't show the wall-stack move properly: it has no robot
//    squaring itself on a wall. It has to be proven on a field.
// Check at practice:
//  - The start: back against the wall at the toggle. A robot may not touch a
//    toggle at the start (<SG1>), and in the CAD the flapper's sprockets sit
//    about 0.07 in in front of it.
//  - The loose Cup+Pin positions (video measurement, +-3 in).
//  - Scoring: in the simulator the claw lands up to ~0.9 in to the side of a
//    goal's centre and ~0.7 in short. The front aligner has to take that.
//
// Try them in the simulator (--start is where each route's setPose puts it):
//   sh sim/run-sim.sh "Red Left 3"   --start -62.61,-6,90
//   sh sim/run-sim.sh "Red Right 2"  --start -6,-62.61,0
//   sh sim/run-sim.sh "Red Left 4"   --start -62.61,-6,90
//   sh sim/run-sim.sh "Red Left 5"   --start -62.61,-6,90
//   sh sim/run-sim.sh "Blue Left 3"  --start 62.61,6,270     (Blue Left 4/5 the same)
//   sh sim/run-sim.sh "Blue Right 2" --start 6,62.61,180
//   sh sim/run-sim.sh "Skills"     --start -62.61,-6,90 --time 60

#include "autons.hpp"
#include "drivetrain.hpp"
#include "claw.hpp"
#include "config.hpp"
#include "lift.hpp"
#include "resets.hpp"
#include "toggle.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#ifdef RW_SIM_PROS
#include "rw/auton-selector.h"
#endif

namespace {

struct Pt {
    double x, y;
};

// ---- V4 robot (CAD, 2026-11-25) --------------------------------------------
// Robot centre (the middle of the four drive wheels, where RW tracks) to the
// centre of a pin, cup or goal in the claw. From the goal, pin and cup the CAD's
// field reference puts in the claw.
constexpr double REACH = 6.25;
constexpr double BACK = 7.64;      // robot centre to the back of the frame
// The front end braces: what touches a wall first, 7.14 in ahead. With them flat
// on a wall the claw is 0.89 in from it, 0.69 in past a wall stack's centre
// (1.58 in out), so the stack sits deep in the claw. Check at practice that the
// claw still closes on it.
constexpr double FRONT = 7.14;
constexpr double HALF_WIDTH = 6.14;  // the drive frame's sides
// The back corners are 9.8 in from the centre, so with the back on a wall the
// robot drives this far out before it can turn.
constexpr double OFF_WALL = 4;
constexpr double LINEUP = 8;       // the last bit into a goal or piece goes straight and slow
// Backs away this far after scoring. The front corners are 9.42 in from the
// centre: at 6 in back they'd clear the goal by 0.02 in when turning, at 8 by 2.
constexpr double BACK_OFF = 8;
constexpr int CLAW_OPEN_MS = 150;  // time for the piece to drop after opening. TODO: practice
constexpr int CLAW_CLOSE_MS = 150; // time for the claw to close on a stack. TODO: practice

// ---- Field (red's side) -------------------------------------------------------
constexpr double WALL = 70.25;
constexpr Pt RED_GOAL_LEFT{-47.09, -23.55};    // red goal, left Quadrant (3.25 in tall)
constexpr Pt RED_GOAL_BOTTOM{-23.55, -47.09};  // red goal, bottom Quadrant
constexpr Pt BLACK_GOAL_LEFT{-47.09, 23.55};   // black goal, left Quadrant (5.8 in tall)
// Cup+Pin stacks against the left wall, each with a goal 21.6 in in front of it.
constexpr Pt STACK_BY_RED_GOAL{-68.7, -23.6};
constexpr Pt STACK_BY_BLACK_GOAL{-68.7, 23.6};
// Loose Cup+Pin stacks, measured off the overhead video (+-3 in):
constexpr Pt DIAG_NEAR{-23.4, -25.3};  // yellow pin, on the diagonal by the Midfield corner
constexpr Pt DIAG_FAR{-45.9, -47.6};   // yellow pin, on the diagonal toward the corner
constexpr Pt MID_EDGE{-22.8, -1.9};    // red/blue pin, middle of the Midfield's left edge

// What the robot drives around: all nine goals, the eight wall stacks and the
// four loaders, in field coordinates (the same set for red and blue), and how
// far the robot's centre has to pass from each: half the robot's width (6.14),
// the thing's radius, and 1.5 in spare. Goal radius 2.81;
// the centre goal's size isn't in the manual's text, so 4.4 is a guess. Stacks
// are cups (1.58), and a loader's 4 x 5 in footprint is treated as a 3.2 in circle.
struct Keepout {
    Pt c;
    double clear;
};
constexpr double GOAL_CLEAR = HALF_WIDTH + 2.81 + 1.5;
constexpr double STACK_CLEAR = HALF_WIDTH + 1.58 + 1.5;
constexpr double LOADER_CLEAR = HALF_WIDTH + 3.2 + 1.5;
constexpr Keepout KEEPOUTS[] = {
    {{0, 0}, HALF_WIDTH + 4.4 + 1.5},
    {{-47.09, -23.55}, GOAL_CLEAR}, {{-47.09, 23.55}, GOAL_CLEAR},
    {{47.09, -23.55}, GOAL_CLEAR},  {{47.09, 23.55}, GOAL_CLEAR},
    {{-23.55, -47.09}, GOAL_CLEAR}, {{-23.55, 47.09}, GOAL_CLEAR},
    {{23.55, -47.09}, GOAL_CLEAR},  {{23.55, 47.09}, GOAL_CLEAR},
    {{-68.7, -23.6}, STACK_CLEAR},  {{-68.7, 23.6}, STACK_CLEAR},
    {{68.7, -23.6}, STACK_CLEAR},   {{68.7, 23.6}, STACK_CLEAR},
    {{-23.6, -68.7}, STACK_CLEAR},  {{-23.6, 68.7}, STACK_CLEAR},
    {{23.6, -68.7}, STACK_CLEAR},   {{23.6, 68.7}, STACK_CLEAR},
    {{-68.5, -58.7}, LOADER_CLEAR}, {{-68.5, 58.7}, LOADER_CLEAR},
    {{68.5, -58.7}, LOADER_CLEAR},  {{68.5, 58.7}, LOADER_CLEAR},
};

// ---- Red or blue ------------------------------------------------------------
bool blue = false;  // the blue autons set this and run the red route

Pt at(Pt p) { return blue ? Pt{-p.x, -p.y} : p; }
double turned(double heading) { return blue ? std::fmod(heading + 180, 360) : heading; }

// Safety time limits. Moves end when they arrive; these only stop a stuck one.
int msFor(double inches) { return static_cast<int>(600 + 30 * inches); }

double distanceTo(Pt fieldPoint) { return std::hypot(fieldPoint.x - x_pos, fieldPoint.y - y_pos); }

// Where the robot centre stops so the claw is on `target` (field coordinates),
// coming at it from where the robot is now, `extra` inches further back.
Pt clawStop(Pt target, double extra = 0) {
    double d = std::max(distanceTo(target), 0.01);
    double ux = (target.x - x_pos) / d, uy = (target.y - y_pos) / d;
    return {target.x - ux * (REACH + extra), target.y - uy * (REACH + extra)};
}

// Prints how long each step took, to the PROS terminal (and the simulator's).
std::uint32_t stepStart = 0;
void step(const char* what) {
    std::uint32_t now = pros::millis();
    std::printf("  %5.2f s  +%4u ms  %s\n", now / 1000.0, now - stepStart, what);
    stepStart = now;
}

void aimAt(Pt fieldPoint) { turnToPoint(fieldPoint.x, fieldPoint.y, 1, 800); }

void driveToField(Pt p, double volts = 12) {
    moveToPoint(p.x, p.y, 1, msFor(distanceTo(p)), true, volts);
}

// With the robot's back against the wall at a toggle and holding the preload:
// sets the pose (red coordinates), grips the preload, flips the toggle, and
// drives off the wall (the toggle only counts once no robot touches it).
void startAtToggle(Pt centre, double heading) {
    Pt f = at(centre);
    setPose(f.x, f.y, turned(heading));
    stepStart = pros::millis();
    clawGrab();  // the preload goes in the open claw before the match
    setToggle(blue);
    step("toggle");
    driveTo(OFF_WALL, msFor(OFF_WALL));
    step("off the wall");
}

// Turns on the spot only when `t` is more than maxOff degrees off the nose.
// moveToPoint steers on the way, but backs up while facing more than 90 away,
// and stops steering for its last 8 in.
void aimIfNeeded(Pt t, double maxOff) {
    double bearing = std::atan2(t.x - x_pos, t.y - y_pos) * 180 / M_PI;
    if (std::fabs(std::remainder(bearing - getInertialHeading(), 360.0)) > maxOff) aimAt(t);
}

// If the straight line from the robot to `to` (field coordinates) passes too
// close to anything in KEEPOUTS, gives a point beside it to drive through first.
bool detour(Pt to, Pt& via) {
    double dx = to.x - x_pos, dy = to.y - y_pos, len2 = dx * dx + dy * dy;
    if (len2 < 1) return false;
    double worst = 0;
    for (const Keepout& k : KEEPOUTS) {
        double along = ((k.c.x - x_pos) * dx + (k.c.y - y_pos) * dy) / len2;
        if (along <= 0.05 || along >= 0.95) continue;  // not beside this stretch
        Pt p{x_pos + along * dx, y_pos + along * dy};
        double d = std::hypot(p.x - k.c.x, p.y - k.c.y);
        if (d >= k.clear || k.clear - d <= worst) continue;
        double ux = d > 0.01 ? (p.x - k.c.x) / d : -dy / std::sqrt(len2);
        double uy = d > 0.01 ? (p.y - k.c.y) / d : dx / std::sqrt(len2);
        via = {k.c.x + ux * (k.clear + 1), k.c.y + uy * (k.clear + 1)};
        worst = k.clear - d;
    }
    return worst > 0;
}

// Drives so the claw ends on `t` (field coordinates): around any goal in the
// way, fast to LINEUP short, a distance reset if a wall is close, then straight
// and slow the rest.
void clawTo(Pt t, double slowVolts, bool armAutograb, bool waitForLift) {
    Pt via;
    for (int i = 0; i < 2 && detour(clawStop(t, LINEUP), via); i++) {
        aimIfNeeded(via, 30);
        moveToPoint(via.x, via.y, 1, msFor(distanceTo(via)), false);  // keeps rolling into the next move
        step("around a goal");
    }
    aimIfNeeded(t, 30);
    if (distanceTo(t) > REACH + LINEUP + 2) {
        driveToField(clawStop(t, LINEUP));
        step("drive");
    }
    if (resetNearWalls() > 0) step("reset");
    if (waitForLift) {
        liftWait(1500);  // the piece has to be above the goal before the claw gets there
        step("lift");
    }
    aimIfNeeded(t, 2);  // the slow part goes dead straight
    if (armAutograb) autograbArm(true);  // closes as soon as the sensor sees it
    driveToField(clawStop(t), slowVolts);
    step("in");
}

// Picks up a free-standing pin or cup (red coordinates).
void grab(Pt piece, const char* name) {
    clawRelease();
    liftTo(LIFT_GRAB);  // already down after score(), unless something stopped it
    clawTo(at(piece), 6, true, false);
    bool seen = autograbWait(150);
    clawGrab();  // close anyway: a cup's clear half can hide it from the sensor
    liftTo(LIFT_CARRY);
    std::printf("Grab %s: %s\n", name, seen ? "sensor saw it" : "closed without the sensor seeing it");
}

// Drives to a point (field coordinates), around anything in the way.
void travelTo(Pt f) {
    Pt via;
    for (int i = 0; i < 2 && detour(f, via); i++) {
        aimIfNeeded(via, 30);
        moveToPoint(via.x, via.y, 1, msFor(distanceTo(via)), false);
        step("around a goal");
    }
    aimIfNeeded(f, 30);
    driveToField(f);
}

// With the front flat on a wall the robot's distance from it is exact: sets
// that coordinate, if the robot really is square (red coordinates for the wall).
void wallContactReset(double faceWall) {
    double heading = getInertialHeading();
    if (std::fabs(std::remainder(heading - turned(faceWall), 360.0)) > 4) {
        std::printf("Wall contact: not square (heading %.1f), no reset\n", heading);
        return;
    }
    Pt red = at({x_pos, y_pos});  // at() is its own inverse
    double before = 0, after = WALL - FRONT;
    if (faceWall == 270) { before = red.x; red.x = -after; after = red.x; }
    else if (faceWall == 90) { before = red.x; red.x = after; }
    else if (faceWall == 180) { before = red.y; red.y = -after; after = red.y; }
    else { before = red.y; red.y = after; }
    Pt f = at(red);
    setPose(f.x, f.y, heading);
    std::printf("Wall contact: %.2f -> %.2f\n", before, after);
}

// Takes a Cup+Pin stack against a wall (red coordinates), the way the robot in
// the overhead video does: from WALL_ENTRY out and along the wall on the side
// `fromSide` (+1 or -1 along the wall), it drives in at 45 degrees until a
// front corner meets the wall, turns square against the wall (the corner
// slides; with all-omni wheels the robot slides sideways easily), pushes
// gently, grabs, sets its distance from the wall from the contact, and backs
// out the same way. faceWall: the heading that faces the wall (270 left wall,
// 180 bottom wall). Touching the wall and brushing the goal beside the stack
// are both allowed; only the opponent's alliance goals are off limits.
// Where the robot lines up: this far out from the wall and along it. The 45 degree
// line in is never closer than 10.9 in to the goal beside the stack, so the robot
// can only turn on the spot (corners 9.8 in out) far enough along it: at 12.8 it
// clears the goal by 0.5 in, and the straight run in passes it with 1.9 to spare.
constexpr double WALL_ENTRY = 12.8;
constexpr double WALL_AIM_SHIFT = 0;  // along the wall, if the stack ends off-centre in the claw. TODO: practice
void grabWallStack(Pt stack, double faceWall, int fromSide, const char* name) {
    double a = faceWall * M_PI / 180;
    Pt toWall{std::sin(a), std::cos(a)};
    Pt along{fromSide * std::cos(a), -fromSide * std::sin(a)};
    Pt final{stack.x - toWall.x * REACH - along.x * WALL_AIM_SHIFT, stack.y - toWall.y * REACH - along.y * WALL_AIM_SHIFT};
    Pt entry{final.x - toWall.x * WALL_ENTRY + along.x * WALL_ENTRY, final.y - toWall.y * WALL_ENTRY + along.y * WALL_ENTRY};
    clawRelease();
    liftTo(LIFT_GRAB);
    travelTo(at(entry));
    step("to the wall stack");
    aimAt(at(final));
    autograbArm(true);
    driveToField(at(final), 8);   // meets the wall at 45 degrees, a front corner first
    turnToAngle(turned(faceWall), 700);  // squares up, sliding along the wall
    driveChassis(3, 3);           // a gentle push to seat both front corners on the wall
    pros::delay(300);
    driveChassis(0, 0);
    bool seen = autograbWait(100);
    clawGrab();
    pros::delay(CLAW_CLOSE_MS);
    step("grabbed at the wall");
    wallContactReset(faceWall);
    liftTo(LIFT_CARRY);
    Pt out = at(entry);
    moveToPoint(out.x, out.y, -1, msFor(WALL_ENTRY * 1.5), true, 8);  // back out the way it came
    step("wall stack");
    std::printf("Grab %s: %s\n", name, seen ? "sensor saw it" : "closed without the sensor seeing it");
}

// Puts what the claw holds on a goal (red coordinates), backs off, lowers the lift.
void score(Pt goal, double liftHeight, const char* name) {
    liftTo(liftHeight);  // rises while the robot drives
    clawTo(at(goal), 7, false, true);
    clawRelease();
    pros::delay(CLAW_OPEN_MS);
    driveTo(-BACK_OFF, msFor(BACK_OFF));
    liftTo(LIFT_GRAB);
    step("release + back off");
    std::printf("Scored on %s\n", name);
}

// ---- Routes (red) --------------------------------------------------------------

void none() {}

// Left Quadrant: toggle, preload and one more on the red goal, one on the black goal.
void redLeft() {
    startAtToggle({-WALL + BACK, -6}, 90);  // flapper arms (+-5.5 in) both on the 25.8 in toggle
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL, "alliance goal (preload)");
    grab(DIAG_FAR, "diagonal piece by the corner");
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE, "alliance goal");
    grab(DIAG_NEAR, "diagonal piece by the Midfield");
    score(BLACK_GOAL_LEFT, LIFT_NEUTRAL_GOAL, "black goal");
}

// The overhead video's route (blue robot, 4 pins), on red's side: toggle;
// preload onto the red goal; the wall stack in front of it onto the red goal;
// the wall stack in front of the black goal onto the black goal; the yellow
// Cup+Pin by the Midfield onto the red goal. The video's robot took 14.9 s.
void redLeft4() {
    startAtToggle({-WALL + BACK, -6}, 90);
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL, "alliance goal (preload)");
    grabWallStack(STACK_BY_RED_GOAL, 270, +1, "wall stack by the alliance goal");
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE, "alliance goal");
    grabWallStack(STACK_BY_BLACK_GOAL, 270, -1, "wall stack by the black goal");
    score(BLACK_GOAL_LEFT, LIFT_NEUTRAL_GOAL, "black goal");
    grab(DIAG_NEAR, "yellow Cup+Pin by the Midfield");
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL + 2 * LIFT_PER_PIECE, "alliance goal");
}

// Red Left 4 plus the Cup+Pin at the Midfield's edge, which the video's robot
// left: preload and the wall stack onto the red goal; then about 90 degrees
// left, the Midfield-edge Cup+Pin onto the black goal (one pin at a time, so it
// has to be scored before the next wall stack); about 90 degrees left again, the
// wall stack in front of the black goal onto the black goal; last, the yellow
// Cup+Pin by the Midfield onto the red goal.
void redLeft5() {
    startAtToggle({-WALL + BACK, -6}, 90);
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL, "alliance goal (preload)");
    grabWallStack(STACK_BY_RED_GOAL, 270, +1, "wall stack by the alliance goal");
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE, "alliance goal");
    grab(MID_EDGE, "red/blue Cup+Pin at the Midfield edge");
    score(BLACK_GOAL_LEFT, LIFT_NEUTRAL_GOAL, "black goal");
    grabWallStack(STACK_BY_BLACK_GOAL, 270, -1, "wall stack by the black goal");
    score(BLACK_GOAL_LEFT, LIFT_NEUTRAL_GOAL + LIFT_PER_PIECE, "black goal");
    grab(DIAG_NEAR, "yellow Cup+Pin by the Midfield");
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL + 2 * LIFT_PER_PIECE, "alliance goal");
}

// Bottom Quadrant: toggle, preload and one more on the red goal.
void redRight() {
    startAtToggle({-6, -WALL + BACK}, 0);
    score(RED_GOAL_BOTTOM, LIFT_ALLIANCE_GOAL, "alliance goal (preload)");
    grab(DIAG_FAR, "diagonal piece by the corner");
    score(RED_GOAL_BOTTOM, LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE, "alliance goal");
}

// Both red toggles plus Red Left 3's scoring. Only the start is planned: the rest
// of the minute needs piece positions from the manual's field drawing, and the
// loaders, which aren't planned.
void skills() {
    redLeft();
    // Back into the bottom wall at its toggle: drive to a point in front of it,
    // face away from the wall, back in slowly, flip, drive off.
    Pt front = at({-6, -WALL + BACK + 12});
    aimAt(front);
    driveToField(front);
    turnToAngle(turned(0), 1000);
    step("to the bottom toggle");
    Pt wall = at({-6, -WALL + BACK + 0.5});
    moveToPoint(wall.x, wall.y, -1, msFor(12), true, 6);
    resetNearWalls();
    setToggle(blue);
    step("toggle");
    driveTo(OFF_WALL, msFor(OFF_WALL));
    step("off the wall");
}

template <void (*Route)()>
void asBlue() {
    blue = true;
    Route();
    blue = false;
}

}  // namespace

// The brain screen's selector lists these in order; runAuton() runs the one picked.
// The number in a name is how many pins that auton puts on goals.
const Auton autons[] = {
    {"None", none},
    {"Red Left 3", redLeft},
    {"Red Left 4", redLeft4},
    {"Red Left 5", redLeft5},
    {"Red Right 2", redRight},
    {"Blue Left 3", asBlue<redLeft>},
    {"Blue Left 4", asBlue<redLeft4>},
    {"Blue Left 5", asBlue<redLeft5>},
    {"Blue Right 2", asBlue<redRight>},
    {"Skills", skills},
};
const int autonCount = sizeof(autons) / sizeof(autons[0]);
int selectedAuton = 0;

void runAuton() {
    // Hold so the robot stops dead at the end of each move instead of rolling on.
    stopChassis(pros::MotorBrake::hold);
    setPose(0, 0, 0);  // autons can override this
#ifdef RW_SIM_PROS
    // In the simulator, RW's auton list picks (see registerSimAutons() in sim_checks.cpp).
    runSelectedAuton();
#else
    autons[selectedAuton].run();
#endif
}
