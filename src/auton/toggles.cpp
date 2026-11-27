// Flipping a second toggle: backing onto it, or side-on. See auton.hpp.
#include "auton.hpp"

namespace auton {

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
