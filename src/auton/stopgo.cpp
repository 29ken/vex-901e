// The older stop-and-turn moves (Red Left 3, Red Right 2 and Skills use them). See auton.hpp.
#include "auton.hpp"

namespace auton {

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

}  // namespace auton
