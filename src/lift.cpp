#include "lift.hpp"
#include "config.hpp"
#include <cstdio>

pros::MotorGroup lift({LIFT_LEFT, LIFT_RIGHT});

static LiftTop top;

void moveLift(int power) {
    bool knewTop = top.found;
    lift.move(top.limit(power, lift.get_position(), lift.get_actual_velocity(), pros::millis()));
    if (top.found && !knewTop) std::printf("Lift max height found at %.0f motor degrees\n", top.position);
}
