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
// Add `--frame 7.14,7.64,6.14` so the wall meets V4's real frame, and
// `--wall stick` or `--wall slide` (default) for a wall that grips a corner
// or lets it slide: the wall stacks in Red Left 4/5 depend on which.

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
// Backs away this far after scoring: the front corners are 9.42 in from the
// centre, so 6.5 in back (12.75 in from the goal's centre) they clear the goal
// by 0.5 in when turning.
constexpr double BACK_OFF = 6.5;
constexpr double BACK_OFF_VOLTS = 7;  // in the gap beside a goal the robot has 0.6 in to the wall when it turns
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

// ---- How the robot moves ------------------------------------------------------
// Moves are chained where it's safe: RW's exit=false ends a move without
// stopping, so the next one keeps the speed and the path curves from one into
// the next. A chained move leaves the motors running, so the next thing after
// one must always be another move, never a wait or a sensor reset; every helper
// below keeps to that, and every route ends with a stop (runAuton()).
// Every number here is a starting guess until practice.
constexpr double TURN_FIRST = 60;  // turn on the spot first only past this many degrees; under it the drive curves while moving
constexpr double FINAL_RUN = 6;    // the last stretch into a goal or piece, straight
constexpr double FINAL_VOLTS = 9;
constexpr double FINAL_AIM_DEG = 3;  // re-aim before the last stretch when more off than this
constexpr double EARLY_EXIT = 1;     // moveToPoint stops about this far short: aim this far past
constexpr double DRIVE_IPS = 35;   // rough average speed, to guess whether the lift will be up in time. TODO: practice
constexpr double LIFT_DPS = 400;   // rough lift speed with a load, sensor degrees per second. TODO: practice
// Close enough to drive in: the lift finishes the last bit on the way. Scoring
// only needs the piece above the goal; grabbing needs the claw right down.
constexpr double LIFT_NEAR_SCORE = 40;  // sensor degrees. TODO: practice
constexpr double LIFT_NEAR_GRAB = 10;

// With the robot's back against the wall at a toggle and holding the preload:
// sets the pose (red coordinates), grips the preload, flips the toggle, starts
// the lift up for the first goal, and rolls off the wall into the next move
// (the toggle only counts once no robot touches it).
void startAtToggle(Pt centre, double heading, double firstLift = LIFT_ALLIANCE_GOAL) {
    Pt f = at(centre);
    setPose(f.x, f.y, turned(heading));
    stepStart = pros::millis();
    clawGrab();  // the preload goes in the open claw before the match
    setToggle(blue);
    step("toggle");
    liftTo(firstLift);  // not during the flip: the ratchet motor drives both the winch and the flapper
    driveTo(OFF_WALL, msFor(OFF_WALL), false);
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
// A point to pass beside something is only good if the robot can turn there:
// its corners reach 9.8 in, so 10.3 in from the walls, and clear of everything else.
bool roomToTurn(Pt p, const Keepout* besides) {
    if (WALL - std::fabs(p.x) < 10.3 || WALL - std::fabs(p.y) < 10.3) return false;
    for (const Keepout& k : KEEPOUTS) {
        double size = k.clear - HALF_WIDTH - 1.5;  // the thing's own radius
        if (&k != besides && std::hypot(p.x - k.c.x, p.y - k.c.y) < 9.8 + size + 0.5) return false;
    }
    return true;
}

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
        Pt other{k.c.x - ux * (k.clear + 1), k.c.y - uy * (k.clear + 1)};
        if (!roomToTurn(via, &k) && roomToTurn(other, &k)) via = other;  // e.g. not between a goal and the wall
        worst = k.clear - d;
    }
    return worst > 0;
}

// Rolls through the points beside anything in the way of `to` (chained).
void aroundObstacles(Pt to) {
    Pt via;
    for (int i = 0; i < 3 && detour(to, via); i++) {
        Pt first;
        if (detour(via, first)) via = first;  // a point on the far side: the way there goes around too
        aimIfNeeded(via, TURN_FIRST);
        moveToPoint(via.x, via.y, 1, msFor(distanceTo(via)), false);
        step("around a goal");
    }
}

// How far the lift still has to go to be within `near` of `target` (0 without its sensor).
double liftLeft(double target, double near) {
    LiftStatus s = liftStatus();
    if (!s.sensorOk || !s.zeroed) return 0;
    return std::max(0.0, std::fabs(target - s.height) - near);
}

// Waits until the lift is within `near` of `target`, or has stopped, or timeout_ms.
void waitLiftNear(double target, double near, int timeout_ms) {
    std::uint32_t start = pros::millis();
    while (liftLeft(target, near) > 0 && liftMoving() && pros::millis() - start < static_cast<std::uint32_t>(timeout_ms)) {
        pros::delay(10);
    }
}

// Drives so the claw ends on `t` (field coordinates): around anything in the
// way, then at full speed to FINAL_RUN short and on into the last stretch
// without stopping, when the lift will be at `liftTarget` by then. If it won't,
// it stops there instead, tries a distance reset, waits for the lift and goes on.
void clawTo(Pt t, bool armAutograb, double liftTarget, double liftNear) {
    aroundObstacles(clawStop(t, FINAL_RUN));
    aimIfNeeded(t, TURN_FIRST);
    double d = distanceTo(t) - REACH - FINAL_RUN;
    // The piece only meets the goal 1.5 in into the last stretch (piece and goal radii).
    bool flowing = liftLeft(liftTarget, liftNear) / LIFT_DPS <= (std::max(d, 0.0) + 1.5) / DRIVE_IPS;
    if (d > 2) {
        Pt lineup = clawStop(t, FINAL_RUN);
        moveToPoint(lineup.x, lineup.y, 1, msFor(d), !flowing);
        step(flowing ? "drive" : "drive, then wait for the lift");
    }
    if (!flowing) {
        if (resetNearWalls() > 0) step("reset");
        waitLiftNear(liftTarget, liftNear, 1500);
        step("lift");
        aimIfNeeded(t, 3);
    }
    if (armAutograb) autograbArm(true);  // closes as soon as the sensor sees it
    // The last stretch goes straight (moveToPoint stops steering 8 in out), so
    // face the target first: coming out of a chained move the robot can be
    // 15-20 degrees off, which put pieces 2.7-3.5 in beside the claw.
    aimIfNeeded(t, FINAL_AIM_DEG);
    Pt stop = clawStop(t, -EARLY_EXIT);
    moveToPoint(stop.x, stop.y, 1, msFor(FINAL_RUN + 2), true, FINAL_VOLTS);
    step("in");
}

// Picks up a free-standing Cup+Pin (red coordinates) and starts the lift up
// to `nextLift`, the height of the goal it goes to next.
void grab(Pt piece, const char* name, double nextLift = LIFT_CARRY) {
    clawRelease();
    liftTo(LIFT_GRAB);
    clawTo(at(piece), true, LIFT_GRAB, LIFT_NEAR_GRAB);  // the lift has to be down before the claw gets there
    bool seen = autograbWait(80);
    clawGrab();  // close anyway: a cup's clear half can hide it from the sensor
    pros::delay(CLAW_CLOSE_MS);
    liftTo(nextLift);  // rises on the way to the goal
    std::printf("Grab %s: %s\n", name, seen ? "sensor saw it" : "closed without the sensor seeing it");
}

// Takes a Cup+Pin stack against a wall (red coordinates), the way the robot in
// the overhead video does, fitted to V4. V4 (14.8 in long, corners 9.4-9.8 in
// from its centre) can't turn to face the wall in the gap between the stack and
// the goal in front of it: turning on the spot there it hits the goal or the
// stack, so it can't come in square the way 10B's robot does.
//  1. It drives in at 45 degrees from the side `fromSide` (+1 or -1 along the
//     wall) until the front corner on that side touches the wall.
//  2. It turns square while pressing that corner on the wall: a curve whose
//     centre is a little past the corner (WALL_TURN_RADIUS, half the frame
//     width is 6.14), so the corner pushes into the wall and friction holds
//     it while the robot slides sideways on its omni rollers. If the corner
//     stays put, the stack moves into the claw's opening (checked: 0.67 in
//     from the arms, and still inside if the corner slides 1 in along the
//     wall). How far it really slides depends on the wall's grip, which only
//     the field can show: WALL_AIM_SHIFT corrects a slide that repeats.
//  3. A gentle push seats the front on the wall and the claw grabs. Holding
//     the stack, whose place on the field is known, the robot knows where it
//     is: its distance from the wall and its place along it.
//  4. It backs out to GAP_TURN from the wall, where it can spin to the goal
//     behind it: 0.37 in from the goal and 0.39 from the wall. That's the
//     tightest part of the route; brushing our own goal or the wall is allowed.
// faceWall: the heading that faces the wall (270 left wall, 180 bottom wall).
// The 45 degree run-in passes the goal 5.6 in clear, and the turn about the corner 1.9 in.
constexpr double STACK_FROM_WALL = 1.55;  // a stack's centre from the wall's face (RW's field model)
constexpr double WALL_LINEUP = 12;        // how far back along the 45 degree line it lines up
constexpr double GAP_TURN = 10.1;         // centre from the wall when spinning to leave: corners 9.8 + 0.3
constexpr double GAP_VOLTS = 5;           // backing out to GAP_TURN: at 8 V it went an inch too far
constexpr double WALL_PAST = 1;           // aims this far past the contact point: moveToPoint stops about 1 in early
constexpr double WALL_TURN_RADIUS = 7.5;  // the squaring curve's radius. TODO: practice
constexpr double WALL_AIM_SHIFT = 0;      // along the wall, toward the side it comes from, if the stack ends off-centre in the claw. TODO: practice
void grabWallStack(Pt stack, double faceWall, int fromSide, const char* name, double nextLift = LIFT_CARRY) {
    double a = faceWall * M_PI / 180;
    Pt toWall{std::sin(a), std::cos(a)};
    Pt along{fromSide * std::cos(a), -fromSide * std::sin(a)};  // toward the side it comes from
    // Square on the wall: front flat on it, the stack in the claw.
    double back = FRONT - STACK_FROM_WALL;
    Pt square{stack.x - toWall.x * back - along.x * WALL_AIM_SHIFT, stack.y - toWall.y * back - along.y * WALL_AIM_SHIFT};
    // The front corner on the come-from side, on the wall: what it turns about.
    Pt pivot{square.x + toWall.x * FRONT + along.x * HALF_WIDTH, square.y + toWall.y * FRONT + along.y * HALF_WIDTH};
    // The square pose turned 45 degrees back about that corner: where the corner first touches.
    double contactHeading = faceWall - fromSide * 45;
    Pt v{square.x - pivot.x, square.y - pivot.y};
    double r = -fromSide * 45 * M_PI / 180;  // compass turn, clockwise positive
    Pt contact{pivot.x + v.x * std::cos(r) + v.y * std::sin(r), pivot.y - v.x * std::sin(r) + v.y * std::cos(r)};
    double c = contactHeading * M_PI / 180;
    Pt lineup{contact.x - std::sin(c) * WALL_LINEUP, contact.y - std::cos(c) * WALL_LINEUP};
    Pt past{contact.x + std::sin(c) * WALL_PAST, contact.y + std::cos(c) * WALL_PAST};

    clawRelease();
    liftTo(LIFT_GRAB);
    Pt l = at(lineup), k = at(contact), kp = at(past);
    // Straight to the contact pose when already on the 45 degree line, else through the line-up point.
    double bearing = std::atan2(k.x - x_pos, k.y - y_pos) * 180 / M_PI;
    bool direct = std::fabs(std::remainder(bearing - turned(contactHeading), 360.0)) <= 6 && distanceTo(k) <= WALL_LINEUP + 8;
    if (!direct) {
        aroundObstacles(l);
        aimIfNeeded(l, TURN_FIRST);
        moveToPoint(l.x, l.y, 1, msFor(distanceTo(l)), true);  // stops on the 45 degree line
        turnToAngle(turned(contactHeading), 700);  // and faces along it: the corner has to meet the wall where planned
    }
    step(direct ? "to the wall stack (straight in)" : "to the wall stack");
    aimIfNeeded(k, TURN_FIRST);
    autograbArm(true);
    moveToPoint(kp.x, kp.y, 1, static_cast<int>(200 + 28 * distanceTo(kp)), true, FINAL_VOLTS);
    step("corner on the wall");
    curveCircle(turned(faceWall), fromSide * WALL_TURN_RADIUS, 700, true, 8);  // squares up, pressing the corner on the wall
    driveChassis(3, 3);  // a gentle push to seat the front on the wall
    pros::delay(150);
    driveChassis(0, 0);
    bool seen = autograbWait(50);
    clawGrab();
    pros::delay(CLAW_CLOSE_MS);
    step("grabbed at the wall");
    // Square on the wall, the front is on it: that gives the distance from the
    // wall. Holding the stack, whose place is known, gives the place along the
    // wall too, but only if the stack really is in the claw: a missed grab
    // would set it ~2 in wrong and every later grab would miss (seen in the
    // simulator). The claw's sensor says whether it's there; the simulator's
    // sensor can't see pieces, so there it's always trusted.
#ifdef RW_SIM_PROS
    bool holding = true;
#else
    bool holding = seen;
#endif
    double heading = getInertialHeading();
    if (std::fabs(std::remainder(heading - turned(faceWall), 360.0)) <= 4) {
        Pt before = at({x_pos, y_pos});
        double off = (before.x * toWall.x + before.y * toWall.y) - (WALL - FRONT);  // how far past the wall RW thinks the front is
        Pt now{before.x - toWall.x * off, before.y - toWall.y * off};
        if (holding) now = {stack.x - toWall.x * back, stack.y - toWall.y * back};
        Pt f = at(now);
        setPose(f.x, f.y, heading);
        std::printf("Wall contact%s: (%.2f, %.2f) -> (%.2f, %.2f)\n", holding ? " + stack" : "", before.x, before.y, now.x, now.y);
    } else {
        std::printf("Wall contact: not square (heading %.1f), no reset\n", heading);
    }
    liftTo(nextLift);  // rises while it backs out and turns
    driveTo(-(GAP_TURN - FRONT), msFor(GAP_TURN - FRONT) + 300, true, GAP_VOLTS);  // slow: an inch too far and it can't turn
    step("wall stack");
    std::printf("Grab %s: %s\n", name, seen ? "sensor saw it" : "closed without the sensor seeing it");
}

// Puts what the claw holds on a goal (red coordinates), then backs off (to a
// stop) and lowers the lift.
void score(Pt goal, double liftHeight, const char* name) {
    liftTo(liftHeight);  // usually already on its way up
    clawTo(at(goal), false, liftHeight, LIFT_NEAR_SCORE);
    clawRelease();
    pros::delay(CLAW_OPEN_MS);
    driveTo(-BACK_OFF, msFor(BACK_OFF), true, BACK_OFF_VOLTS);  // stops: at full power it went 1.5 in too far
    liftTo(LIFT_GRAB);
    step("release + back off");
    std::printf("Scored on %s\n", name);
}

// ---- Routes (red) --------------------------------------------------------------

void none() {}

// Left Quadrant: toggle, preload and the corner Cup+Pin on the red goal, the
// Midfield Cup+Pin on the black goal.
void redLeft() {
    startAtToggle({-WALL + BACK, -6}, 90);  // flapper arms (+-5.5 in) both on the 25.8 in toggle
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL, "alliance goal (preload)");
    grab(DIAG_FAR, "yellow Cup+Pin by the corner", LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE);
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE, "alliance goal");
    grab(DIAG_NEAR, "yellow Cup+Pin by the Midfield", LIFT_NEUTRAL_GOAL);
    score(BLACK_GOAL_LEFT, LIFT_NEUTRAL_GOAL, "black goal");
}

// The overhead video's route (blue robot, 4 pins), on red's side: toggle;
// preload onto the red goal; the wall stack in front of it onto the red goal;
// the wall stack in front of the black goal onto the black goal; the yellow
// Cup+Pin by the Midfield onto the red goal. The video's robot took 14.9 s.
void redLeft4() {
    startAtToggle({-WALL + BACK, -6}, 90);
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL, "alliance goal (preload)");
    grabWallStack(STACK_BY_RED_GOAL, 270, +1, "wall stack by the alliance goal", LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE);
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE, "alliance goal");
    grabWallStack(STACK_BY_BLACK_GOAL, 270, -1, "wall stack by the black goal", LIFT_NEUTRAL_GOAL);
    score(BLACK_GOAL_LEFT, LIFT_NEUTRAL_GOAL, "black goal");
    grab(DIAG_NEAR, "yellow Cup+Pin by the Midfield", LIFT_ALLIANCE_GOAL + 2 * LIFT_PER_PIECE);
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL + 2 * LIFT_PER_PIECE, "alliance goal");
}

// Ken's order from the route labeler (2026-11-25): preload, the wall stack by
// the red goal and the yellow Cup+Pin by the corner onto the red goal; the
// yellow Cup+Pin by the Midfield, then the wall stack by the black goal, onto
// the black goal.
void redLeft5() {
    startAtToggle({-WALL + BACK, -6}, 90);
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL, "alliance goal (preload)");
    grabWallStack(STACK_BY_RED_GOAL, 270, +1, "wall stack by the alliance goal", LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE);
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE, "alliance goal");
    grab(DIAG_FAR, "yellow Cup+Pin by the corner", LIFT_ALLIANCE_GOAL + 2 * LIFT_PER_PIECE);
    score(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL + 2 * LIFT_PER_PIECE, "alliance goal");
    grab(DIAG_NEAR, "yellow Cup+Pin by the Midfield", LIFT_NEUTRAL_GOAL);
    score(BLACK_GOAL_LEFT, LIFT_NEUTRAL_GOAL, "black goal");
    grabWallStack(STACK_BY_BLACK_GOAL, 270, -1, "wall stack by the black goal", LIFT_NEUTRAL_GOAL + LIFT_PER_PIECE);
    score(BLACK_GOAL_LEFT, LIFT_NEUTRAL_GOAL + LIFT_PER_PIECE, "black goal");
}

// Bottom Quadrant: toggle, preload and the corner Cup+Pin on the red goal.
void redRight() {
    startAtToggle({-6, -WALL + BACK}, 0);
    score(RED_GOAL_BOTTOM, LIFT_ALLIANCE_GOAL, "alliance goal (preload)");
    grab(DIAG_FAR, "yellow Cup+Pin by the corner", LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE);
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
    // Routes end on a chained move, which leaves the motors running: stop.
    driveChassis(0, 0);
    stopChassis(pros::MotorBrake::hold);
}
