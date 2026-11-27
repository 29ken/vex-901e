// Red or blue, small helpers, and step timing. See auton.hpp.
#include "auton.hpp"
#ifdef RW_SIM_PROS
#include <cstdlib>
#endif

namespace auton {

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

}  // namespace auton
