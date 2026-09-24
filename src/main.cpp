#include "main.h"
#include "drivetrain.hpp"
#include "controls.hpp"
#include "autons.hpp"
#include "ui.hpp"

void initialize() {
    drive::init();
    ui::init();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    autons::run();
}

void opcontrol() {
    while (true) {
        controls::driveControl();
        pros::delay(10);
    }
}
