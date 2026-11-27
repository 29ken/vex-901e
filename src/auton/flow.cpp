// Curved moves: take a piece without stopping, score, back away, and the goal reset. See auton.hpp.
#include "auton.hpp"

namespace auton {

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

}  // namespace auton
