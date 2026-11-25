#include "toggle.hpp"
#include "main.h"
#include <cstdio>

// Placeholder until the ratchet motor and optical sensor have ports (see toggle.hpp).
void setToggle(bool blueAlliance) {
    std::printf("Toggle: flapper not wired yet, waiting %d ms instead (%s)\n", TOGGLE_FLIP_MS,
                blueAlliance ? "blue" : "red");
    pros::delay(TOGGLE_FLIP_MS);
}

ToggleColor toggleColor() {
    return ToggleColor::Unknown;
}
