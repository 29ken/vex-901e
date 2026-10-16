#pragma once
#include "api.h"
#include <algorithm>
#include <cstdint>

extern pros::MotorGroup lift;

// Runs the lift at `power` (-127 to 127, + is up). The first time it stalls
// against the top, that spot becomes the max height. After that it slows down
// near the top and stops just under it instead of grinding into the stop.
void moveLift(int power);

// Stalled = pushing up but barely turning, for this long.
inline constexpr double LIFT_STALL_RPM = 5;
inline constexpr std::uint32_t LIFT_STALL_MS = 250;
// In motor degrees. Raise LIFT_SLOW_ZONE if it still hits the top hard.
inline constexpr double LIFT_SLOW_ZONE = 120;  // start slowing this far below the top
inline constexpr double LIFT_TOP_MARGIN = 15;  // stop this far below it
inline constexpr int LIFT_SLOW_POWER = 40;

// The max height logic, kept apart from the motors so it can be tested off
// the robot. Feed it what the motor is doing; it returns the power to send.
// ponytail: the first stall wins, so if the lift snags on the way up, that
// becomes the top until the program restarts. Add a re-learn button if it bites.
struct LiftTop {
    bool found = false;
    double position = 0;
    std::uint32_t stalledSince = 0;

    int limit(int power, double at, double rpm, std::uint32_t now) {
        if (power <= 0) {
            stalledSince = 0;
            return power;
        }
        if (found) {
            double left = position - at;
            if (left <= LIFT_TOP_MARGIN) return 0;
            if (left < LIFT_SLOW_ZONE) return std::min(power, LIFT_SLOW_POWER);
            return power;
        }
        if (rpm > LIFT_STALL_RPM) {
            stalledSince = 0;
            return power;
        }
        if (!stalledSince) stalledSince = now;
        if (now - stalledSince >= LIFT_STALL_MS) {
            found = true;
            position = at;
            return 0;  // stop grinding into the stop
        }
        return power;
    }
};
