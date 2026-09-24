#include "main.h"
#include "drivetrain.hpp"
#include "controls.hpp"
#include "autons.hpp"
#include "ui.hpp"

void initialize() {
    initDrive();
    initUI();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    runAuton();
}

void opcontrol() {
    while (true) {
        driveControl();
        pros::delay(10);
    }
}
