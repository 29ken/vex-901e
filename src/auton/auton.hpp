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
//  - The ratchet motor has no port yet (see toggle.hpp): setToggle() only waits.
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
//
// The auton code is split by job (all in src/auton/):
//   auton.hpp      this file: the robot's and field's numbers, and what each file offers the others
//   basics.cpp     red/blue, small helpers, step timing
//   stopgo.cpp     the older stop-and-turn moves (Red Left 3, Red Right 2, Skills use them)
//   wallstack.cpp  taking a Cup+Pin stack off a wall
//   flow.cpp       curved moves: take a piece rolling, score, back away, the goal reset
//   toggles.cpp    flipping a second toggle (backing in, or side-on)
//   routes.cpp     the routes themselves, the auton list, runAuton()
//   practice.cpp   set-up and test runs (Cal: ..., Test: ...)
//   lab.cpp        simulator only: routes from a text script
#pragma once

#include "autons.hpp"
#include "runlog.hpp"
#include "drivetrain.hpp"
#include "claw.hpp"
#include "watch.hpp"
#include "config.hpp"
#include "lift.hpp"
#include "resets.hpp"
#include "toggle.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace auton {

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
// Backs away about this far after scoring: the front corners are 9.42 in from
// the centre, so 6.5 in back (12.75 in from the goal's centre) they clear the
// goal by 0.5 in when turning. Between a goal and a wall see backOffToTurn().
constexpr double BACK_OFF = 6.5;
constexpr double BACK_OFF_VOLTS = 7;
constexpr double FRONT_CORNER = 9.42, BACK_CORNER = 9.8;  // corner distances from the centre (CAD)
constexpr double GOAL_RADIUS = 2.81;
constexpr int SETTLE_MS = 100;  // holding still before a turn in a tight spot
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
constexpr Pt MID_BOTTOM{-2.4, -24.2};  // loose Cup+Pin by the Midfield's bottom edge (off the video, +-3 in)
constexpr Pt BLACK_GOAL_BOTTOM{23.55, -47.09};  // black goal, bottom Quadrant: starts with a yellow pin on it
constexpr Pt STACK_BY_RED_GOAL_BOTTOM{-23.6, -68.7};
constexpr Pt STACK_BY_BLACK_GOAL_BOTTOM{23.6, -68.7};  // on the bottom wall, in front of the bottom black goal

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

// ---- Numbers more than one file uses ---------------------------------------------
constexpr double TURN_FIRST = 60;  // turn on the spot first only past this many degrees; under it the drive curves while moving
constexpr double FINAL_RUN = 6;    // the last stretch into a goal or piece, straight
constexpr double FINAL_VOLTS = 9;
constexpr double FINAL_AIM_DEG = 3;  // re-aim before the last stretch when more off than this
constexpr double EARLY_EXIT = 1;     // moveToPoint stops about this far short: aim this far past
constexpr double DRIVE_IPS = 35;   // rough average speed, to guess whether the lift will be up in time. TODO: practice
constexpr double LIFT_DPS = 550;   // rough lift speed, sensor degrees per second (the simulator unloaded lift does ~600). TODO: measure at practice
// Close enough to drive in: the lift finishes the last bit on the way. Scoring
// only needs the piece above the goal; grabbing needs the claw right down.
constexpr double LIFT_NEAR_SCORE = 40;  // sensor degrees. TODO: practice
constexpr double LIFT_NEAR_GRAB = 10;
constexpr double GAP_VOLTS = 5;           // backing out to GAP_TURN: at 8 V it went an inch too far
constexpr double LIFT_CLEAR = 2;     // the open claw is clear of a piece it let go this far back. TODO: practice

// A point on a curved leg (see "Curved moves" in flow.cpp): red coordinates, and the
// top speed passing it as a fraction of TOP_IPS.
struct Via {
    double x, y;
    double speed = 1;
};

// ---- basics.cpp ------------------------------------------------------------------
extern bool blue;  // the blue autons set this and run the red route
double knob(const char* name, double fallback);
Pt at(Pt p);
double turned(double heading);
int msFor(double inches);
double distanceTo(Pt fieldPoint);
Pt clawStop(Pt target, double extra = 0);
extern std::uint32_t stepStart;  // when the last step ended (step() prints the time since)
void step(const char* what);
void aimAt(Pt fieldPoint);
void driveToField(Pt p, double volts = 12);

// ---- stopgo.cpp ------------------------------------------------------------------
extern bool turnOnSpotNext;
void startAtToggle(Pt centre, double heading, double firstLift = LIFT_ALLIANCE_GOAL, bool rollOff = true);
void aimIfNeeded(Pt t, double maxOff);
bool roomToTurn(Pt p, const Keepout* besides);
bool detour(Pt to, Pt& via);
void aroundObstacles(Pt to);
double liftLeft(double target, double near);
void waitLiftNear(double target, double near, int timeout_ms);
void clawTo(Pt t, bool armAutograb, double liftTarget, double liftNear);
void grab(Pt piece, const char* name, double nextLift = LIFT_CARRY);
void settle();
void backOffToTurn(Pt goal);
void score(Pt goal, double liftHeight, const char* name);

// ---- wallstack.cpp ---------------------------------------------------------------
extern double wallAimNext, wallApproachNext, wallLineupSkipNext;  // for the next wall stack only
extern bool stackMayBeBehind;
void grabWallStack(Pt stack, double faceWall, int fromSide, const char* name, double nextLift = LIFT_CARRY,
                   bool flow = false, std::vector<Via> via = {}, bool backOut = true, bool approach = true);

// ---- flow.cpp --------------------------------------------------------------------
extern double TOP_IPS;  // top path speed, in/s
// Faster drive limits for one route, put back when the route ends. TOP_IPS is
// the top path speed and RW's profile_max_accel how hard it speeds up and
// slows down (100 in/s^2 by default). The drive's free speed is about 65 in/s,
// so these are within reach on paper, but nothing above 45 / 100 has been run
// on the robot: check them against the drive log before trusting a route that
// needs them. The simulator can still override both (TOP_IPS, ACCEL).
struct DriveLimits {
    double top = TOP_IPS, accel = profile_max_accel;
    DriveLimits(double topIps, double accelIps2) {
        TOP_IPS = knob("TOP_IPS", topIps);
        profile_max_accel = knob("ACCEL", accelIps2);
    }
    ~DriveLimits() {
        TOP_IPS = top;
        profile_max_accel = accel;
    }
};

double curve(const std::vector<Via>& pts, bool reverse, bool stop, double topIps);
double topSpeedForLift(double length, double liftTarget, double near);
double legLength(Pt fromRed, const std::vector<Via>& pts);
void take(Pt piece, const char* name, double nextLift, std::vector<Via> via = {});
int resetAtStop();
double clawFromGoal(Pt goalRed);
void seatReset(Pt goalRed, double miss);
void scoreOn(Pt goal, double liftHeight, const char* name, std::vector<Via> via = {});
void backAway(double straight, std::vector<Via> then = {});
void gapBack(Pt goal);
void faceTo(Pt p);

// ---- toggles.cpp -----------------------------------------------------------------
void flipToggleAt(Pt centre, double heading, std::vector<Via> via = {}, bool reverseIn = false, bool rollOff = true);
void sideToggle(Pt where, const std::vector<Via>& pts, bool ram = false);

// ---- routes.cpp ------------------------------------------------------------------
void none();
void redLeft();
void redLeft4();
void redLeft5();
void redSoloAwp();
void redRight();
void skills();

// ---- practice.cpp ----------------------------------------------------------------
void calFeedforward();
void calDriveLog();
void testTurn();
void testDrive();

#ifdef RW_SIM_PROS
// ---- lab.cpp (simulator only) ----------------------------------------------------
void lab();
#endif

}  // namespace auton
