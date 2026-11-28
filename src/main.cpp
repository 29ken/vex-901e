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
    autograbAuto(false);  // autons arm autograb themselves
    runAuton();
}


void opcontrol() {

    heading_correction = false; //possibly remove
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    // The claw keeps whatever auton left it holding (it starts open at power-on).
    // From here on it closes by itself whenever a piece sits in it.
    autograbAuto(true);

    applyDriverBrakes();

    while (true) {
        // 1. Drivetrain
        if (tuning == TUNE_NONE) {
            driveControl();
        }

        // 2. Cascade: L1 up, L2 down, while held.
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            liftManual(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            liftManual(-127);
        } else {
            liftManual(0);
        }

        // 3. B: the claw, by hand. Closed: opens it (lets go). Open: closes it.
        // Closing is usually automatic: the claw task (src/sub/claw.cpp) shuts
        // it when the distance sensor sees a piece in place, and after a
        // release waits for the claw to be empty first. B is for when the
        // sensor misses a piece. (A is only the controller menu's + button.)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            clawToggle();
        }

        // 4. Toggle (include/toggle.hpp). R1, one press: flip the toggle next
        // to the flapper to our colour. R2, held: turn the flapper by hand.
        toggleManual(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2));
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            setToggleAsync(blueAlliance());
        }

        pros::delay(10);  // inside da loop
    }
}
