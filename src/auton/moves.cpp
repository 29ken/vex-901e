// How the robot does each kind of move: the building blocks the routes (routes.cpp) are made of.
// The robot's and field's numbers are in auton.hpp.
#include "auton.hpp"
#ifdef RW_SIM_PROS
#include <cstdlib>
#endif

namespace auton {

// ============================================================================
// Red or blue, small helpers, step timing
// ============================================================================

// ---- Red or blue ------------------------------------------------------------
bool blue = false;  // the blue autons set this and run the red route

// A tuning value. On the robot it is always `fallback`; the simulator can
// override it by name from the environment, to compare settings without a rebuild.
double knob(const char* name, double fallback) {
#ifdef RW_SIM_PROS
    if (const char* e = std::getenv(name)) return std::atof(e);
#endif
    (void)name;
    return fallback;
}

Pt at(Pt p) { return blue ? Pt{-p.x, -p.y} : p; }
double turned(double heading) { return blue ? std::fmod(heading + 180, 360) : heading; }

// Safety time limits. Moves end when they arrive; these only stop a stuck one.
int msFor(double inches) { return static_cast<int>(600 + 30 * inches); }

double distanceTo(Pt fieldPoint) { return std::hypot(fieldPoint.x - x_pos, fieldPoint.y - y_pos); }

// Where the robot centre stops so the claw is on `target` (field coordinates),
// coming at it from where the robot is now, `extra` inches further back.
Pt clawStop(Pt target, double extra) {
    double d = std::max(distanceTo(target), 0.01);
    double ux = (target.x - x_pos) / d, uy = (target.y - y_pos) / d;
    return {target.x - ux * (REACH + extra), target.y - uy * (REACH + extra)};
}

// Prints how long each step took, to the PROS terminal (and the simulator's).
std::uint32_t stepStart = 0;
void step(const char* what) {
    std::uint32_t now = pros::millis();
    say("  %5.2f s  +%4u ms  %s\n", now / 1000.0, now - stepStart, what);
    stepStart = now;
}

void aimAt(Pt fieldPoint) { turnToPoint(fieldPoint.x, fieldPoint.y, 1, 800); }

void driveToField(Pt p, double volts) {
    moveToPoint(p.x, p.y, 1, msFor(distanceTo(p)), true, volts);
}

// ============================================================================
// Stop-and-turn moves (Red Left 3, Red Right 2 and Skills use them)
// ============================================================================

// ---- How the robot moves ------------------------------------------------------
// Moves are chained where it's safe: RW's exit=false ends a move without
// stopping, so the next one keeps the speed and the path curves from one into
// the next. A chained move leaves the motors running, so the next thing after
// one must always be another move, never a wait or a sensor reset; every helper
// below keeps to that, and every route ends with a stop (runAuton()).
// Every number here is a starting guess until practice.
// With the robot's back against the wall at a toggle and holding the preload:
// sets the pose (red coordinates), grips the preload, flips the toggle, starts
// the lift up for the first goal, and rolls off the wall into the next move
// (the toggle only counts once no robot touches it). rollOff false: the next
// move is a curve that leaves the wall by itself.
void startAtToggle(Pt centre, double heading, double firstLift, bool rollOff) {
    Pt f = at(centre);
    setPose(f.x, f.y, turned(heading));
    watchEnable(true);  // the distance-sensor watch: logs only
    stepStart = pros::millis();
    clawGrab();  // the preload goes in the open claw before the match
    setToggle(blue);
    step("toggle");
    liftTo(firstLift);  // not during the flip: the ratchet motor drives both the winch and the flapper
    if (!rollOff) return;
    driveTo(OFF_WALL, msFor(OFF_WALL), false);
    step("off the wall");
}

// Turns on the spot only when `t` is more than maxOff degrees off the nose.
// moveToPoint steers on the way, but backs up while facing more than 90 away,
// and stops steering for its last 8 in.
// Set after backing off between a goal and a wall: there a forward curve
// drives a corner into the goal (0.56 in in the simulator), so the next move
// turns on the spot first, however small the turn.
bool turnOnSpotNext = false;

void aimIfNeeded(Pt t, double maxOff) {
    if (turnOnSpotNext) {
        maxOff = std::min(maxOff, 3.0);
        turnOnSpotNext = false;
    }
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
void grab(Pt piece, const char* name, double nextLift) {
    clawRelease();
    liftTo(LIFT_GRAB);
    clawTo(at(piece), true, LIFT_GRAB, LIFT_NEAR_GRAB);  // the lift has to be down before the claw gets there
    bool seen = autograbWait(80);
    clawGrab();  // close anyway: a cup's clear half can hide it from the sensor
    pros::delay(CLAW_CLOSE_MS);
    liftTo(nextLift);  // rises on the way to the goal
    say("Grab %s: %s\n", name, seen ? "sensor saw it" : "closed without the sensor seeing it");
}

// Stops dead and holds for a moment. Every RW move starts by letting the
// motors coast, so a robot still rolling from a slow back-out kept going about
// 1 in into the next turn, enough to clip the goal beside a wall stack.
void settle() {
    driveChassis(0, 0);
    stopChassis(pros::MotorBrake::hold);
    pros::delay(SETTLE_MS);
}

// Puts what the claw holds on a goal (red coordinates), then backs off (to a
// stop) and lowers the lift.
// Backs straight away from the goal it just scored on (field coordinates) to
// where it can turn: front corners clear of that goal, back corners clear of
// the walls. In the open that's BACK_OFF. Between a goal and the wall there is
// only ~1 in to spare in all (at 7 V it went 0.9 in too far and its corner hit
// the wall on the next turn), so there it goes to the middle, slowly, and settles.
void backOffToTurn(Pt goal) {
    double h = getInertialHeading() * M_PI / 180;
    Pt away{-std::sin(h), -std::cos(h)};
    auto room = [&](double b) {
        Pt p{x_pos + away.x * b, y_pos + away.y * b};
        double fromGoal = std::hypot(p.x - goal.x, p.y - goal.y) - FRONT_CORNER - GOAL_RADIUS;
        double fromWall = std::min(WALL - std::fabs(p.x), WALL - std::fabs(p.y)) - BACK_CORNER;
        return std::min(fromGoal, fromWall);
    };
    double best = BACK_OFF, most = -1e9;
    for (double b = 4; b <= 10.001; b += 0.05) {
        if (room(b) > most) { most = room(b); best = b; }
    }
    if (most >= 1.5) {  // open field: the usual back-off is enough
        driveTo(-BACK_OFF, msFor(BACK_OFF), true, BACK_OFF_VOLTS);
        return;
    }
    driveTo(-best, msFor(best) + 300, true, GAP_VOLTS);
    settle();
    turnOnSpotNext = true;
    say("Back off %.2f in: %.2f in to spare when it turns\n", best, most);
}

void score(Pt goal, double liftHeight, const char* name) {
    liftTo(liftHeight);  // usually already on its way up
    clawTo(at(goal), false, liftHeight, LIFT_NEAR_SCORE);
    clawRelease();
    pros::delay(CLAW_OPEN_MS);
    backOffToTurn(at(goal));
    liftTo(LIFT_GRAB);
    step("release + back off");
    say("Scored on %s\n", name);
}

// ============================================================================
// Taking a Cup+Pin stack off a wall
// ============================================================================

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
const double GAP_TURN = knob("GAP_TURN", 10.1);         // centre from the wall when spinning to leave: corners 9.8 + 0.3
constexpr double WALL_PAST = 1;           // aims this far past the contact point: moveToPoint stops about 1 in early
constexpr double WALL_TURN_RADIUS = 7.5;  // the squaring curve's radius. TODO: practice
// Moves where the claw ends along the wall: positive is away from the side the
// robot comes from (the code aims at stack - along * shift), and the second wall
// stack gets the same shift mirrored. Set it if a stack ends off-centre the same
// way every time. TODO: practice
constexpr double WALL_AIM_SHIFT = 0;
// An extra shift for the next wall stack only (same sign), set by a route just before it.
double wallAimNext = 0;
// The next wall stack only: the angle off square at which the corner meets the
// wall (45 unless a route sets it). Steeper suits a stack that is right beside
// the robot along the wall: it turns on the spot and drives a few inches.
double wallApproachNext = 45;
// The next wall stack only (flow approach): how close to the line-up point the
// robot may start for that point to be left out of the curve.
double wallLineupSkipNext = 0.75;
// How far off square the robot is when it grabs a wall stack, degrees. The claw
// point is 0.89 in inside the frame's front and a stack is 1.55 in off the
// wall, so square on the wall the stack is 0.66 in too deep in the claw. With
// one front corner on the wall and the robot this far off square, the claw
// point is 0.89 cos(a) + 6.14 sin(a) from the wall: 1.55 in at 6 degrees. So
// it stops turning there and grabs, with no push. 0 = square up and push.
// The lab can override it (env WALL_GRAB_ANGLE) to compare.
constexpr double WALL_GRAB_ANGLE_DEFAULT = 0;
double wallGrabAngle() {
#ifdef RW_SIM_PROS
    if (const char* e = std::getenv("WALL_GRAB_ANGLE")) return std::atof(e);
#endif
    return WALL_GRAB_ANGLE_DEFAULT;
}
// Set after a wall stack the claw's sensor didn't confirm (see resetAtStop()).
bool stackMayBeBehind = false;

// The squaring curve stops as soon as the robot is within this many degrees of
// its target heading (0 = let the curve run to its time limit, as before).
constexpr double WALL_SQUARE_EXIT_DEG = 3;
constexpr int WALL_PUSH_MS = 0;              // a push to seat the front on the wall after the turn (150 before the turn was cut short; 0 = grab straight away)
constexpr double WALL_GRAB_DURING_PUSH = 0;  // 1: the claw may close during the push
double squareAngle = 0, squareRadius = 0;
void squareMotion() { curveCircle(squareAngle, squareRadius, 700, true, 8); }

// flow: the approach is one curve through `via`, then down the 45 degree line
// to the wall, instead of stopping at the line-up point to turn.
void grabWallStack(Pt stack, double faceWall, int fromSide, const char* name, double nextLift, bool flow,
                   std::vector<Via> via, bool backOut, bool approach) {
    double a = faceWall * M_PI / 180;
    Pt toWall{std::sin(a), std::cos(a)};
    Pt along{fromSide * std::cos(a), -fromSide * std::sin(a)};  // toward the side it comes from
    // Square on the wall: front flat on it, the stack in the claw.
    double back = FRONT - STACK_FROM_WALL;
    double aimShift = WALL_AIM_SHIFT + wallAimNext;
    wallAimNext = 0;
    Pt square{stack.x - toWall.x * back - along.x * aimShift, stack.y - toWall.y * back - along.y * aimShift};
    // The front corner on the come-from side, on the wall: what it turns about.
    Pt pivot{square.x + toWall.x * FRONT + along.x * HALF_WIDTH, square.y + toWall.y * FRONT + along.y * HALF_WIDTH};
    // The square pose turned 45 degrees back about that corner: where the corner first touches.
    double approachDeg = wallApproachNext;  // how far off square the robot is when its corner meets the wall
    wallApproachNext = 45;
    double contactHeading = faceWall - fromSide * approachDeg;
    Pt v{square.x - pivot.x, square.y - pivot.y};
    double r = -fromSide * approachDeg * M_PI / 180;  // compass turn, clockwise positive
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
    if (!approach) {
        // Already alongside the wall, a front corner at it (after a side-on
        // toggle pass): the turn about that corner is 90 degrees, not 45.
        step("alongside the wall");
    } else if (flow) {
        // Starting on (or near) the line-up point, the slow-down there is skipped:
        // the robot is already where that point would put it.
        if (distanceTo(l) > wallLineupSkipNext) via.push_back({lineup.x, lineup.y, 0.6});
        wallLineupSkipNext = 0.75;
        via.push_back({past.x, past.y, 0.25});
        curve(via, false, true, 45);
        step("to the wall stack");
    } else if (!direct) {
        aroundObstacles(l);
        aimIfNeeded(l, TURN_FIRST);
        moveToPoint(l.x, l.y, 1, msFor(distanceTo(l)), true);  // stops on the 45 degree line
        settle();  // still rolling ~16 in/s here, it coasted 1.8 in past the line-up point during the turn
        turnToAngle(turned(contactHeading), 700);  // and faces along it: the corner has to meet the wall where planned
    }
    if (!flow && approach) {
        step(direct ? "to the wall stack (straight in)" : "to the wall stack");
        aimIfNeeded(k, FINAL_AIM_DEG);  // the drive-in is straight: arriving 8 degrees off moved the claw ~2 in
        moveToPoint(kp.x, kp.y, 1, static_cast<int>(200 + 28 * distanceTo(kp)), true, FINAL_VOLTS);
    }
    step("corner on the wall");
    double grabAngle = wallGrabAngle();
    double exitDeg = knob("WS_EXIT_DEG", WALL_SQUARE_EXIT_DEG);
    squareAngle = turned(faceWall - fromSide * grabAngle);
    squareRadius = fromSide * WALL_TURN_RADIUS;
    if (exitDeg > 0) {
        // The curve itself runs to its time limit even after the robot is
        // square (0.4 s of waiting in the simulator), so watch the heading and
        // stop it as soon as the robot is nearly there.
        std::uint32_t began = pros::millis();
        runAsync(squareMotion);
        while (pros::millis() - began < 750) {
            double off = std::fabs(std::remainder(getInertialHeading() - squareAngle, 360.0));
            if (off <= exitDeg && pros::millis() - began > 60) break;
            if (!isMotionRunning() && pros::millis() - began > 60) break;
            pros::delay(5);
        }
        cancelMotion();
    } else {
        squareMotion();
    }
    int pushMs = static_cast<int>(knob("WS_PUSH_MS", WALL_PUSH_MS));
    bool early = knob("WS_GRAB_EARLY", WALL_GRAB_DURING_PUSH) > 0;
    bool seen = false;
    if (early) autograbArm(true);  // the claw may close while the push seats the front
    if (grabAngle == 0 && pushMs > 0) {
        driveChassis(3, 3);  // a gentle push to seat the front on the wall
        pros::delay(pushMs);
    }
    driveChassis(0, 0);
    // Without `early`: only now look for the stack. Armed during the curve,
    // the claw could close and drag the stack. So `seen` means it's in the claw now.
    if (!early) autograbArm(true);
    seen = autograbWait(50);
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
    // With a front corner on the wall and the robot `tilt` off square, the
    // centre is FRONT cos(tilt) + HALF_WIDTH sin(tilt) from the wall.
    double tilt = std::fabs(std::remainder(heading - turned(faceWall), 360.0)) * M_PI / 180;
    double centreOut = FRONT * std::cos(tilt) + HALF_WIDTH * std::sin(tilt);
    if (tilt * 180 / M_PI <= grabAngle + 4) {
        Pt before = at({x_pos, y_pos});
        double off = (before.x * toWall.x + before.y * toWall.y) - (WALL - centreOut);  // how far past the wall RW thinks the corner is
        Pt now{before.x - toWall.x * off, before.y - toWall.y * off};
        if (holding) {
            // The stack is in the claw: REACH ahead of the centre along the heading.
            double hr = (blue ? heading - 180 : heading) * M_PI / 180;
            now = {stack.x - std::sin(hr) * REACH, stack.y - std::cos(hr) * REACH};
            // The wall still gives the distance from it exactly.
            double offNow = (now.x * toWall.x + now.y * toWall.y) - (WALL - centreOut);
            now = {now.x - toWall.x * offNow, now.y - toWall.y * offNow};
        }
        Pt f = at(now);
        setPose(f.x, f.y, heading);
        say("Wall contact%s: (%.2f, %.2f) -> (%.2f, %.2f)\n", holding ? " + stack" : "", before.x, before.y, now.x, now.y);
    } else {
        say("Wall contact: not square (heading %.1f), no reset\n", heading);
    }
    { Pt gone = at(stack); watchTaken(gone.x, gone.y); }
    stackMayBeBehind = !seen;  // the simulator's sensor never sees it, so there the back sensor sits this stop out
    liftTo(nextLift);  // rises while it backs out and turns
    if (backOut) {
        driveTo(-(GAP_TURN - centreOut), msFor(GAP_TURN - centreOut) + 300, true, knob("GAP_VOLTS", GAP_VOLTS));  // slow: an inch too far and it can't turn
        settle();  // the next move spins here
    }
    step("wall stack");
    say("Grab %s: %s\n", name, seen ? "sensor saw it" : "closed without the sensor seeing it");
}

// ============================================================================
// Curved moves: take a piece without stopping, score, back away, the goal reset
// ============================================================================

// ---- Curved moves (RW's followPath) ---------------------------------------------
// A leg is one smooth curve through a few points, driven with RW's pure pursuit
// follower: it slows for bends by itself and each point can cap the speed there.
// The robot doesn't stop to pick a piece up: the curve slows to GRAB_IPS through
// the piece, the claw closes on the way (its sensor, or the robot's own position
// as the backup), and the next curve carries on from that speed. It stops only
// where it has to: on a goal, to let go of the piece, and at a wall stack.
// RW's path follower needs the drive's feedforward measured on the real robot
// (measureDriveFeedforward(), rw-config.cpp) before these speeds mean anything.
double TOP_IPS = knob("TOP_IPS", 45);       // top path speed, in/s (RW's default for this drive: 80% of free speed)
constexpr double GRAB_IPS = 12;      // speed through a piece while the claw closes. 15 saved ~0.05 s a grab in the simulator but left grabs 1.34 in off-centre (limit 1.42). TODO: practice
constexpr double GRAB_SLOW = 4;      // already at GRAB_IPS this far before the piece
constexpr double GRAB_LEAD = 1.5;    // the position backup closes the claw this far before the piece is at the claw point (about GRAB_IPS x CLAW_CLOSE_MS)
constexpr double GRAB_ON = 1.0;      // keeps rolling this far past the piece's place while the claw closes
constexpr double SCORE_SLOW = 8;     // the last stretch into a goal starts this far out...
constexpr double SCORE_SLOW_SPEED = 0.45;  // ...at this fraction of TOP_IPS
constexpr double SCORE_PAST = 0.5;   // aims this far past the stop: RW's follower stops up to 1 in short
// Drives a smooth curve from where the robot is through `pts`. reverse: driving
// backwards. stop: stops at the last point (otherwise it arrives rolling and the
// next move must be another drive). Returns the curve's length.
double curve(const std::vector<Via>& pts, bool reverse, bool stop, double topIps) {
    std::vector<Waypoint> w;
    w.emplace_back(x_pos, y_pos, 1.0);
    for (const Via& v : pts) {
        Pt f = at({v.x, v.y});
        if (std::hypot(f.x - w.back().x, f.y - w.back().y) < 0.75) continue;  // on top of the last point
        w.emplace_back(f.x, f.y, v.speed);
    }
    if (w.size() < 2) return 0;
    Path path = pathFromPoints(w);
    followPath(path, 1500 + path.length / 15 * 1000, reverse, stop, topIps);
    return path.length;
}

// A top speed for a leg of `length` inches that gets there as the lift gets to
// `liftTarget`: the robot slows down for the lift instead of stopping to wait.
double topSpeedForLift(double length, double liftTarget, double near) {
    double need = liftLeft(liftTarget, near) / LIFT_DPS;  // seconds the lift still needs
    if (need <= 0.05) return TOP_IPS;
    return std::clamp(1.2 * length / need, 10.0, TOP_IPS);  // a leg averages about 80% of its top speed
}

double legLength(Pt fromRed, const std::vector<Via>& pts) {
    double total = 0;
    Pt last = fromRed;
    for (const Via& v : pts) {
        total += std::hypot(v.x - last.x, v.y - last.y);
        last = {v.x, v.y};
    }
    return total;
}

// Picks up a free-standing Cup+Pin (red coordinates) without stopping: a curve
// through `via` that runs straight through the piece at GRAB_IPS, the claw
// closing on the way. The piece is met coming from the last `via` point (or
// from the robot, with none). Starts the lift up to `nextLift` once it has it.
void take(Pt piece, const char* name, double nextLift, std::vector<Via> via) {
    clawRelease();
    liftTo(LIFT_GRAB);
    Pt here = at({x_pos, y_pos});  // red coordinates
    Pt from = via.empty() ? here : Pt{via.back().x, via.back().y};
    double d = std::max(std::hypot(piece.x - from.x, piece.y - from.y), 0.01);
    Pt u{(piece.x - from.x) / d, (piece.y - from.y) / d};
    double slow = GRAB_IPS / TOP_IPS;
    via.push_back({piece.x - u.x * (REACH + GRAB_SLOW), piece.y - u.y * (REACH + GRAB_SLOW), slow});
    via.push_back({piece.x - u.x * (REACH - GRAB_ON), piece.y - u.y * (REACH - GRAB_ON), slow});
    Pt f = at(piece);
    autograbArmNear(f.x, f.y, REACH + GRAB_LEAD, nextLift);  // the lift starts up as soon as the claw has shut
    // The lift has to be down before the claw gets there.
    curve(via, false, false, topSpeedForLift(legLength(here, via), LIFT_GRAB, LIFT_NEAR_GRAB));
    char how = autograbHow();
    double when = how ? autograbWhenMs() / 1000.0 : pros::millis() / 1000.0;
    clawGrab();  // close anyway, if neither trigger fired
    autograbArm(false);
    watchTaken(f.x, f.y);
    liftTo(nextLift);  // rises on the way to the goal
    step("took it rolling");
    say("Grab %s: %s at %.2f s\n", name, how == 's' ? "sensor saw it" : how == 'o' ? "closed by position" : "closed at the end of the curve", when);
}

// Distance-sensor resets at the stops a route already makes (on a goal, the
// claw takes CLAW_OPEN_MS to open anyway). Tries the sensors that should see a
// wall from here; each is used only if the robot is within 12 degrees of
// square to that wall and the reading agrees with where it thinks it is
// (resets.hpp). Never the front sensor: a goal or piece is in front of it.
// Not the back one right after a wall stack unless the claw's sensor saw the
// stack: a stack left at the wall would be read as the wall.
int resetAtStop() {
    std::uint32_t start = pros::millis();
    int fixed = 0;
    for (ResetSensor sensor : {ResetSensor::Back, ResetSensor::Left, ResetSensor::Right}) {
        if (sensor == ResetSensor::Back && stackMayBeBehind) continue;
        ResetPreview p = previewReset(sensor);
        if (p.installed && p.ok && resetFrom(sensor)) fixed++;
    }
    if (fixed) say("Reset at stop: %d sensor(s), %u ms\n", fixed, static_cast<unsigned>(pros::millis() - start));
    return fixed;
}

// Seated on a goal, the robot knows where it is: the claw opening's arms sit
// either side of the goal (0.19 in to spare each side) and the goal aligner's
// notch stops against its base, so the goal's centre is GOAL_SEAT ahead of the
// robot's centre, on its centre line. The goal's place on the field is known,
// so that gives x and y (the IMU gives the heading). Skipped if RW thinks the
// claw is further than SEAT_TRUST from the goal: then it probably isn't seated.
// TODO: practice. Check the robot really ends pressed onto the goal, and measure
// GOAL_SEAT (the notch is 3.24 in radius about the claw point and a goal's base
// is 2.81 in across the flats, 3.22 on the diagonals: 5.82 to 6.23 in).
constexpr double GOAL_SEAT = 6.0;
constexpr double SEAT_TRUST = 3.0;
constexpr double GOAL_SEAT_RESET = 1;  // 0 turns it off
constexpr double DIST_CHECK_AT_SCORE = 1;  // 0 turns the distance-sensor check at each score off
constexpr double DIST_KEEP_IN = 1.5;       // a sensor within this of the seat only confirms it
constexpr double SEAT_PUSH_VOLTS = 3;  // the light press onto the goal while the claw opens
// How far RW has the claw point from a goal (red coordinates) right now.
double clawFromGoal(Pt goalRed) {
    Pt g = at(goalRed);
    double h = getInertialHeading() * M_PI / 180;
    return std::hypot(x_pos + std::sin(h) * REACH - g.x, y_pos + std::cos(h) * REACH - g.y);
}
// `miss`: clawFromGoal() when the robot arrived, before it pressed on the goal
// (the wheels can creep while it presses, which RW counts as driving).
void seatReset(Pt goalRed, double miss) {
    Pt g = at(goalRed);
    double heading = getInertialHeading(), h = heading * M_PI / 180;
    if (miss > SEAT_TRUST) {
        say("Goal seat: RW had the claw %.1f in from the goal, no reset\n", miss);
        return;
    }
    double nx = g.x - std::sin(h) * GOAL_SEAT, ny = g.y - std::cos(h) * GOAL_SEAT;
    say("Goal seat: (%.2f, %.2f) -> (%.2f, %.2f)\n", x_pos, y_pos, nx, ny);
    setPose(nx, ny, heading);
}

// Puts what the claw holds on a goal (red coordinates): a curve through `via`
// that slows into the goal and stops with the claw over it, then lets go. The
// goal is met coming from the last `via` point (or from the robot, with none).
void scoreOn(Pt goal, double liftHeight, const char* name, std::vector<Via> via) {
    liftTo(liftHeight);  // usually already on its way up
    Pt here = at({x_pos, y_pos});
    Pt from = via.empty() ? here : Pt{via.back().x, via.back().y};
    double d = std::max(std::hypot(goal.x - from.x, goal.y - from.y), 0.01);
    Pt u{(goal.x - from.x) / d, (goal.y - from.y) / d};
    if (d > REACH + SCORE_SLOW + 2) {
        via.push_back({goal.x - u.x * (REACH + SCORE_SLOW), goal.y - u.y * (REACH + SCORE_SLOW), SCORE_SLOW_SPEED});
    }
    via.push_back({goal.x - u.x * (REACH - SCORE_PAST), goal.y - u.y * (REACH - SCORE_PAST), SCORE_SLOW_SPEED});
    curve(via, false, true, topSpeedForLift(legLength(here, via), liftHeight, LIFT_NEAR_SCORE));
    if (liftLeft(liftHeight, LIFT_NEAR_SCORE) > 0) {
        waitLiftNear(liftHeight, LIFT_NEAR_SCORE, 1500);
        step("lift");
    }
    step("in");
    clawRelease();
    std::uint32_t opened = pros::millis();
    // While the claw opens, press lightly onto the goal: the follower can stop
    // up to an inch short, and the seat reset is only right once the aligner
    // is against the goal's base. Costs no time.
    bool seat = knob("SEAT_RESET", GOAL_SEAT_RESET) > 0;
    double arrived = clawFromGoal(goal);
    if (seat) driveChassis(SEAT_PUSH_VOLTS, SEAT_PUSH_VOLTS);
    else resetAtStop();  // the distance sensors want the robot still
    std::uint32_t took = pros::millis() - opened;
    // Pressed on the goal, read the distance sensors in what is left of the
    // claw's opening time (no extra time): the same moment the no-tracker
    // teams reset at. They are checked against the seat below.
    bool check = seat && knob("DIST_CHECK", DIST_CHECK_AT_SCORE) > 0;
    const std::uint32_t SAMPLE_MS = 90;  // 3 readings 40 ms apart, with 10 ms to spare
    if (check && took + SAMPLE_MS < static_cast<std::uint32_t>(CLAW_OPEN_MS)) pros::delay(CLAW_OPEN_MS - SAMPLE_MS - took);
    StopReadings readings{};
    if (check) readings = sensorSampleAtStop();
    took = pros::millis() - opened;
    if (took < static_cast<std::uint32_t>(CLAW_OPEN_MS)) pros::delay(CLAW_OPEN_MS - took);
    if (seat) {
        driveChassis(0, 0);
        seatReset(goal, arrived);
        // The seat is the better reference (a fraction of an inch), so a sensor
        // that agrees with it changes nothing. One that disagrees by more than
        // DIST_KEEP_IN says the robot isn't really seated: then the sensor wins.
        if (check) sensorCheckAtStop(readings, name, DIST_KEEP_IN, distance_reset_max_jump);
    }
    stackMayBeBehind = false;
    step("release");
    say("Scored on %s\n", name);
}

// Backs away from the goal it just scored on: `straight` inches straight back
// (the claw's arms are around the goal), then a reverse curve through `then`,
// which swings the nose toward the next piece. The lift comes down once the
// claw is clear of the goal. Ends rolling: the next move must be a drive.
void backAway(double straight, std::vector<Via> then) {
    double h = getInertialHeading() * M_PI / 180;
    // The lift starts down once the open claw is LIFT_CLEAR back from the piece it left.
    double first = std::min(LIFT_CLEAR, straight);
    Pt c = at({x_pos - std::sin(h) * first, y_pos - std::cos(h) * first});
    Pt r = at({x_pos - std::sin(h) * straight, y_pos - std::cos(h) * straight});  // red coordinates
    curve({{c.x, c.y}}, true, false, TOP_IPS);
    liftTo(LIFT_GRAB);
    curve({{r.x, r.y, then.empty() ? 0.3 : 1.0}}, true, false, TOP_IPS);
    if (!then.empty()) {
        then.back().speed = std::min(then.back().speed, 0.3);  // slow at the end: the next drive goes forwards
        curve(then, true, false, TOP_IPS);
    }
    step("back away");
}

// For the gap between a goal and the wall, where there's no room for a reverse
// curve: the slow straight back-off to where the robot can turn, then stopped.
void gapBack(Pt goal) {
    driveTo(-LIFT_CLEAR, msFor(LIFT_CLEAR), false, GAP_VOLTS);  // the open claw clears the piece, then the lift starts down
    liftTo(LIFT_GRAB);
    backOffToTurn(at(goal));
    step("back off (gap)");
}

// Turns on the spot to face a point (red coordinates). Only from a stop.
void faceTo(Pt p) {
    turnOnSpotNext = false;
    aimAt(at(p));
    step("turn");
}

// ============================================================================
// Flipping a second toggle: backing onto it, or side-on
// ============================================================================

// Flips a second toggle (red coordinates for its place on the wall, as in
// startAtToggle): a curve through `via` to a point TOGGLE_LINEUP in front of
// it, turns its back to the wall, backs in slowly, flips, and rolls off the
// wall (a toggle only counts once no robot touches it, and for the Autonomous
// Win Point the robot must not end touching the wall).
constexpr double TOGGLE_LINEUP = 10;
constexpr double TOGGLE_LINEUP_REVERSE = 7;  // coming tail first it is already lined up: a shorter slow stretch
constexpr double TOGGLE_PRESS_VOLTS = 4;     // the press onto the wall after reversing in
constexpr int TOGGLE_PRESS_MS = 40;         // pressing before the flapper starts; the press then holds through the flip
constexpr double TOGGLE_ROLL_OFF = 2;  // off the wall and off the toggle afterwards. TODO: practice (does the flapper clear it?)
// reverseIn: the robot comes tail first (a reverse curve through `via`), so it
// keeps rolling backwards onto the wall with no stop and no turn in front of it.
// With its back flat on the wall it also knows its distance from that wall.
void flipToggleAt(Pt centre, double heading, std::vector<Via> via, bool reverseIn, bool rollOff) {
    double h = heading * M_PI / 180;
    double lineup = reverseIn ? knob("TOG_LINEUP", TOGGLE_LINEUP_REVERSE) : TOGGLE_LINEUP;
    Pt front{centre.x + std::sin(h) * lineup, centre.y + std::cos(h) * lineup};
    if (reverseIn) {
        via.push_back({front.x, front.y, 0.45});
        curve(via, true, false, TOP_IPS);
    } else {
        via.push_back({front.x, front.y, 0.5});
        curve(via, false, true, TOP_IPS);
        // Already close to square (a reverse curve can arrive that way): no turn needed.
        if (std::fabs(std::remainder(getInertialHeading() - turned(heading), 360.0)) > 5) turnToAngle(turned(heading), 900);
    }
    step("to the toggle");
    Pt wall = at({centre.x - std::sin(h) * 0.5, centre.y - std::cos(h) * 0.5});
    moveToPoint(wall.x, wall.y, -1, msFor(lineup), true, 9);
    if (reverseIn) {
        // The move can stop up to an inch short, so press the back onto the wall
        // and keep a light press on through the flip: the flapper has to reach
        // the toggle and its push must not walk the robot off the wall. The
        // wheels may creep while pressing, so the position is set afterwards.
        driveChassis(-TOGGLE_PRESS_VOLTS, -TOGGLE_PRESS_VOLTS);
        pros::delay(static_cast<int>(knob("TOG_PRESS_MS", TOGGLE_PRESS_MS)));
        setToggle(blue);
        driveChassis(0, 0);
        if (std::fabs(std::remainder(getInertialHeading() - turned(heading), 360.0)) <= 4) {
            // Back flat on the wall: the centre is BACK from it. Set that coordinate, keep the other.
            Pt b = at({x_pos, y_pos});  // red coordinates
            Pt fixed = std::fabs(std::sin(h)) > 0.7 ? Pt{centre.x, b.y} : Pt{b.x, centre.y};
            Pt f = at(fixed);
            say("Back on the wall: (%.2f, %.2f) -> (%.2f, %.2f)\n", x_pos, y_pos, f.x, f.y);
            setPose(f.x, f.y, getInertialHeading());
        }
    } else {
        setToggle(blue);
    }
    step("toggle");
    if (!rollOff) return;  // the next move drives forward off the wall anyway
    driveTo(TOGGLE_ROLL_OFF, msFor(TOGGLE_ROLL_OFF));
    step("off the wall");
}

// Flips a toggle side-on while driving past it. V4 has a side flap on each
// side of the toggle shaft (CAD: 6.05-7.55 in from the centre line, so 1.35 in
// past the frame's side, and 6.4-7.1 in behind the robot's centre). With the
// robot alongside the wall and its side within about an inch of it, that flap
// is over the toggle. The curve through `pts` (red coordinates) should run
// along the wall, slowly, about SIDE_GAP off it; the flip starts when the
// robot's centre comes within 3 in of `where` and takes TOGGLE_FLIP_MS, during
// which the flap (SIDE_FLAP_BEHIND behind the centre) has to stay over the
// toggle (25.8 in long, centred on the wall). Ends rolling. Untested on the
// robot: how close the side has to be, and whether it can flip while moving.
constexpr double SIDE_FLAP_BEHIND = 6.75;
Pt sideFlipWhere{0, 0};
std::atomic<bool> sideFlipBusy{false};
// ram: the curve is aimed a little into the wall, so the robot meets it at a
// shallow angle and slides along it on its side sleds (the wall also turns it
// parallel). When the flip starts and the robot is within 6 degrees of
// parallel, its side is taken to be on the wall, which gives its distance
// from that wall exactly (HALF_WIDTH). Keep the angle shallow (10-20 degrees):
// at 30 in/s and 15 degrees only 8 in/s goes into the wall.
bool sideFlipRam = false;
void sideFlipTask(void*) {
    std::uint32_t began = pros::millis();
    while (distanceTo(sideFlipWhere) > 3 && pros::millis() - began < 6000) pros::delay(10);
    if (sideFlipRam) {
        double h = getInertialHeading();
        bool alongX = std::fabs(std::remainder(h - 90, 180.0)) <= 6;   // heading east or west: beside the bottom or top wall
        bool alongY = std::fabs(std::remainder(h, 180.0)) <= 6;        // heading north or south: beside the left or right wall
        if (alongX && std::fabs(y_pos) > WALL - HALF_WIDTH - 4) {
            double y = std::copysign(WALL - HALF_WIDTH, y_pos);
            say("Side on the wall: y %.2f -> %.2f\n", y_pos, y);
            setPose(x_pos, y, h);
        } else if (alongY && std::fabs(x_pos) > WALL - HALF_WIDTH - 4) {
            double x = std::copysign(WALL - HALF_WIDTH, x_pos);
            say("Side on the wall: x %.2f -> %.2f\n", x_pos, x);
            setPose(x, y_pos, h);
        } else {
            say("Side on the wall: not parallel or not near it (heading %.1f), no reset\n", h);
        }
    }
    setToggle(blue);
    step("toggle");
    sideFlipBusy = false;
}
void sideToggle(Pt where, const std::vector<Via>& pts, bool ram) {
    sideFlipWhere = at(where);
    sideFlipRam = ram;
    sideFlipBusy = true;
    pros::Task flip(sideFlipTask, nullptr, "Side toggle");
    curve(pts, false, false, TOP_IPS);
    step(ram ? "along the toggle (on the wall)" : "along the toggle");
}

}  // namespace auton
