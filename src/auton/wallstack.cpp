// Taking a Cup+Pin stack off a wall. See auton.hpp.
#include "auton.hpp"
#ifdef RW_SIM_PROS
#include <cstdlib>
#endif

namespace auton {

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

}  // namespace auton
