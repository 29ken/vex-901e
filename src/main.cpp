#include "main.h"
#include "drivetrain.hpp"
#include "controls.hpp"
#include "autons.hpp"
#include "screens.hpp"

void initialize() {
    initDrive();
    startBrainScreen();
    startControllerScreen();
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
