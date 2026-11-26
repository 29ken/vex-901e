#include "main.h"
#include "drivetrain.hpp"
#include "autons.hpp"
#include "config.hpp"
#include "lift.hpp"
#include "tuner.hpp"
#include "screens.hpp"
#include "claw.hpp"
#include "watch.hpp"
#include "toggle.hpp"
#include <cstring>

void initialize() {
    initDrive();
    initLift();
    initClaw();
    initToggle();
    initWatch();
    startBrainScreen();
    startControllerScreen();
#ifdef RW_SIM_PROS
    registerSimAutons();  // so sh sim/run-sim.sh can pick autons by name
#endif
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    runAuton();
}


void opcontrol() {

    heading_correction = false; //possibly remove
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    // The claw keeps whatever auton left it holding (it starts open at power-on).
    // An autograb armed during auton doesn't carry over.
    autograbArm(false);

    applyDriverBrakes();

    while (true) {
        // 1. Drivetrain
        if (tuning == TUNE_NONE) {
            driveControl();
        }

        // 2. Lift: R2 up, R1 down.
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            liftManual(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            liftManual(-127);
        } else {
            liftManual(0);
        }

        // 3. A: manual toggle of the claw. While the controller menu is open
        // (X), A is the menu's + button instead.
        if (!controllerMenuOpen() && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            clawToggle();
        }

        // 4. L1: arm/disarm autograb. The claw task (src/sub/claw.cpp) watches
        // the sensor and grabs once, then disarms itself.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            autograbArm(!autograbArmed());
        }

        // 5. L2: let go of the pin
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            clawRelease();
        }

        // 6. B: flip the toggle next to the flapper to our colour. Our colour is
        // the selected auton's ("Blue ..." = blue, anything else = red).
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            setToggleAsync(std::strncmp(autons[selectedAuton].name, "Blue", 4) == 0);
        }

        pros::delay(10);  // inside da loop
    }
}
