// The routes, the auton list, and runAuton(). See auton.hpp.
#include "auton.hpp"
#ifdef RW_SIM_PROS
#include "rw/auton-selector.h"
#endif

namespace auton {

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
    // preload -> RG, W1 -> RG, W2 -> BG, YN -> RG. Simulator: all 4 by 13.4 s, 20 of 20 runs.
    startAtToggle({-WALL + BACK, -6}, 90, LIFT_ALLIANCE_GOAL, false);
    scoreOn(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL, "alliance goal (preload)", {{-55.8, -9}});  // one bend off the toggle, into RG from the NNW (~152 deg)
    backAway(4, {{-51, -8.25}});  // 4 in straight off the goal, then reverse to just NE of W1's 45-degree line-up (rolling)
    wallAimNext = 0.25;
    grabWallStack(STACK_BY_RED_GOAL, 270, +1, "wall stack by the alliance goal", LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE, true);  // reverses straight into the forward run: one curve down the 45 line, no stop at the line-up
    turnToAngle(turned(90), 900);  // spin to face RG (turnToAngle settles faster than face)
    scoreOn(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE, "alliance goal");
    backAway(4, {{-59.2, -23.5}});  // straight off RG, then a short reverse to just short of the gap's turn spot...
    settle();  // ...hold still (stops ~0.1 in past the aim, 0.3 in from RG / 0.8 in from the wall when turning)
    turnToAngle(turned(25), 900);  // aim NNE, clear of RG
    wallAimNext = 1;
    grabWallStack(STACK_BY_BLACK_GOAL, 270, -1, "wall stack by the black goal", LIFT_NEUTRAL_GOAL, true, {{-52, -3, 0.8}, {-50, 4, 0.6}});  // sweep north, then bend left onto W2's 45 line at 27 in/s
    turnToAngle(turned(90), 900);
    scoreOn(BLACK_GOAL_LEFT, LIFT_NEUTRAL_GOAL, "black goal");
    backAway(4, {{-59.2, 23.5}});  // same gap move beside BG
    settle();
    turnToAngle(turned(143), 900);  // face straight down the line to YN (passes BG 1.6 in clear)
    take({-22.8, -24.85}, "yellow Cup+Pin by the Midfield", LIFT_ALLIANCE_GOAL + 2 * LIFT_PER_PIECE, {{-33.7, -11.66}});  // full speed until ~17 in out, decelerate into the piece, grab rolling
    scoreOn(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL + 2 * LIFT_PER_PIECE, "alliance goal", {{-25.5, -33.5}, {-31.8, -37.2}, {-37.8, -34.8}});  // roll on round a right-hand loop (lift rising), last 16 in straight into RG from the SE
}

// Ken's order from the route labeler (2026-11-25): preload, the wall stack by
// the red goal and the yellow Cup+Pin by the corner onto the red goal; the
// yellow Cup+Pin by the Midfield, then the wall stack by the black goal, onto
// the black goal.
void redLeft5() {
    // preload -> RG, W1 -> RG, YF -> RG, YN -> BG, W2 -> BG. Simulator: all 5 by 14.6 s, ends 14.8 s, 20 of 20 runs.
    startAtToggle({-WALL + BACK, -6}, 90, LIFT_ALLIANCE_GOAL, false);
    scoreOn(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL, "alliance goal (preload)", {{-55.8, -9}});  // one bend off the toggle into RG from the NNW (a faster via makes the W1 grab a knife edge)
    backAway(4, {{-51, -8.25}});  // ends within 0.75 in of W1's line-up, so the code's 0.6 cap there is dropped
    wallAimNext = 0.25;
    grabWallStack(STACK_BY_RED_GOAL, 270, +1, "wall stack by the alliance goal", LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE, true);
    turnToAngle(turned(90), 900);
    scoreOn(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE, "alliance goal");
    backAway(4, {{-59.2, -23.5}});  // straight off RG to the gap's spin spot
    settle();
    turnToAngle(turned(148), 900);
    take({-45.03, -47.1}, "yellow Cup+Pin by the corner", LIFT_ALLIANCE_GOAL + 2 * LIFT_PER_PIECE, {{-52.9, -35.7}});
    scoreOn(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL + 2 * LIFT_PER_PIECE, "alliance goal", {{-42, -48}, {-36, -41}, {-37, -35}});  // lift-bound leg; aim centred between the slide and stick wall models
    backAway(4, {{-29, -33.4, 0.8}, {-17.3, -37.9, 0.7}, {-14.6, -43.3}});  // judge: first point 0.2 in further from RGB (RGB pass 0.28 -> 0.47 in, same time)
    take({-22.21, -25.39}, "yellow Cup+Pin by the Midfield", LIFT_NEUTRAL_GOAL, {{-17.3, -37.9}});  // from c: aim 0.6 in short of YN along the approach (slide grab depth 4.4 -> 5.0 in, limit 3.92)
    scoreOn(BLACK_GOAL_LEFT, LIFT_NEUTRAL_GOAL, "black goal", {{-45, -5}, {-50.55, 8.55}});  // BG scored at ~20 deg, mirror of the preload's 157
    backAway(4, {{-52, 8.6}});  // cusp on W2's line-up (mirror of W1's)
    wallAimNext = 1;
    grabWallStack(STACK_BY_BLACK_GOAL, 270, -1, "wall stack by the black goal", LIFT_NEUTRAL_GOAL + LIFT_PER_PIECE, true, {{-52.4, 9.2, 1}});  // usually dropped with the code's 0.6-capped line-up point (0.75 in rule); when kept, W2 is ~0.15 s slower
    turnToAngle(turned(90), 900);
    scoreOn(BLACK_GOAL_LEFT, LIFT_NEUTRAL_GOAL + LIFT_PER_PIECE, "black goal");  // aim centred after W2's reset
}

// Solo Autonomous Win Point, Worlds-qualifying criteria: 7 visible pin halves,
// 3 goals with 2 or more each, neither robot touching the wall at the end.
// Each black goal starts with a yellow pin (1 half once its quadrant's toggle
// is ours), so both toggles are flipped: the left one at the start, the bottom
// one by backing in. Red goal 2 (preload + a Cup+Pin), left black goal 2, bottom
// black goal 3 (two Cup+Pins).
// The partner robot starts on the bottom wall too. This route assumes it sits
// on the LEFT half of the bottom toggle (x from about -21.5 to -3.5) and rolls
// forward off the wall during autonomous; the robot passes 1.5 in from where a
// partner that rolled 4 in forward would be. It backs onto the right half.
// It needs faster drive limits than the other routes (DriveLimits below): at
// 52 in/s and 150 in/s^2 it ends at 15.1 s, at 45 and 100 well over 16.
// The pickup before the toggle is a hairpin and the least forgiving grab: its
// aim point is tuned to the simulator and must be re-found on the field.
void redSoloAwp() {
    // Partner on the left half of the bottom toggle; this robot backs in on the right half.
    // preload -> RG, ME -> BG, YN -> RG, M2 picked up and carried into the bottom toggle,
    // M2 -> BK2, W4 -> BK2. Simulator at these drive limits: ends 14.95 s, 20 of 20 runs.
    DriveLimits fast(55, 175);
    startAtToggle({-WALL + BACK, -6}, 90, LIFT_ALLIANCE_GOAL, false);
    scoreOn(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL, "alliance goal (preload)", {{-55.8, -9}});
    backAway(4, {{-55, -10}});
    take({-22.4, -3}, "Cup+Pin at the Midfield's edge", LIFT_NEUTRAL_GOAL, {{-36, -6}});
    scoreOn(BLACK_GOAL_LEFT, LIFT_NEUTRAL_GOAL, "black goal", {{-31, 8}, {-35.8, 12.3}});
    curve({{-40, 14}}, true, true, TOP_IPS);
    turnToAngle(turned(168), 900);
    take({-24.6, -25.5}, "yellow Cup+Pin by the Midfield", LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE, {{-27, -10}});
    scoreOn(RED_GOAL_LEFT, LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE, "alliance goal", {{-26, -33}, {-34, -31}});
    backAway(5, {{-38, -33}, {-40, -36}});
    take({-7.2, -25.7}, "Cup+Pin", LIFT_NEUTRAL_GOAL, {{-22, -36}, {2, -39.5}});
    flipToggleAt({6, -62.61}, 0, {{4, -40}, {6, -48}}, true, false);
    scoreOn(BLACK_GOAL_BOTTOM, LIFT_NEUTRAL_GOAL, "bottom black goal", {{9, -55.8}});
    backAway(4, {{8.25, -51}});
    wallAimNext = 0.25;
    wallLineupSkipNext = 4;
    grabWallStack(STACK_BY_BLACK_GOAL_BOTTOM, 180, +1, "wall stack by the bottom black goal", LIFT_NEUTRAL_GOAL + LIFT_PER_PIECE, true);
    turnToAngle(turned(0), 900);
    scoreOn(BLACK_GOAL_BOTTOM, LIFT_NEUTRAL_GOAL + LIFT_PER_PIECE, "bottom black goal");
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

}  // namespace auton

using namespace auton;

// The brain screen's selector lists these in order; runAuton() runs the one picked.
// The number in a name is how many pins that auton puts on goals.
const Auton autons[] = {
    {"None", none},
    {"Red Left 3", redLeft},
    {"Red Left 4", redLeft4},
    {"Red Left 5", redLeft5},
    {"Red Right 2", redRight},
    {"Red Solo AWP", redSoloAwp},
    {"Blue Left 3", asBlue<redLeft>},
    {"Blue Left 4", asBlue<redLeft4>},
    {"Blue Left 5", asBlue<redLeft5>},
    {"Blue Right 2", asBlue<redRight>},
    {"Blue Solo AWP", asBlue<redSoloAwp>},
    {"Skills", skills},
#ifndef RW_SIM_PROS
    // Practice set-up runs, not match autons (the simulator's list has no room for them).
    {"Cal: feedforward", calFeedforward},
    {"Cal: drive log", calDriveLog},
    {"Test: turn", testTurn},
    {"Test: drive", testDrive},
#endif
};
const int autonCount = sizeof(autons) / sizeof(autons[0]);
int selectedAuton = 0;

bool blueAlliance() {
    return std::strncmp(autons[selectedAuton].name, "Blue", 4) == 0;
}

void runAuton() {
    // Hold so the robot stops dead at the end of each move instead of rolling on.
    stopChassis(pros::MotorBrake::hold);
    setPose(0, 0, 0);  // autons can override this
#ifdef RW_SIM_PROS
    // In the simulator, RW's auton list picks (see registerSimAutons() in sim_checks.cpp).
    runLogStart("simulator run");  // only writes if RUN_LOG_DIR is set
    runSelectedAuton();
#else
    // Every run is saved to the SD card, if there is one (runlog.hpp). Not the
    // set-up runs: "Cal: drive log" writes its own file.
    bool logged = std::strncmp(autons[selectedAuton].name, "Cal:", 4) != 0 && std::strcmp(autons[selectedAuton].name, "None") != 0;
    if (logged) runLogStart(autons[selectedAuton].name);
    autons[selectedAuton].run();
#endif
    // Routes end on a chained move, which leaves the motors running: stop.
    watchEnable(false);
    driveChassis(0, 0);
    stopChassis(pros::MotorBrake::hold);
    runLogStop();
}
