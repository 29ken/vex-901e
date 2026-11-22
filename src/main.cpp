#include "main.h"
#include "drivetrain.hpp"
#include "autons.hpp"
#include "config.hpp"
#include "lift.hpp"
#include "tuner.hpp"
#include "screens.hpp"

void initialize() {
    initDrive();
    initLift();
    startBrainScreen();
    startControllerScreen();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    runAuton();
}


void opcontrol() {

    heading_correction = false; //possibly remove
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::adi::DigitalOut claw('B');
    pros::Distance distance_sensor(3);

    bool claw_extended = false;
    bool autograb_armed = false;

    // pistons starting variables
    claw.set_value(claw_extended);

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

        // 3. A: manual toggle of the claw
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            claw_extended = !claw_extended;
            claw.set_value(claw_extended);
        }

        // 4. L1: arm/disarm the sensor + autograb
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            autograb_armed = !autograb_armed;
        }

        // 5. L2: let go of the pin
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            claw_extended = false;
            claw.set_value(claw_extended);
        }

        // 6. Autograb: only runs while armed, grabs once, then disarms itself
        if (autograb_armed) {
            int32_t d = distance_sensor.get();
            if (d > 0 && d < 51) {
                claw_extended = true;
                claw.set_value(claw_extended);
                autograb_armed = false;
            }
        }

        pros::delay(10);  // inside da loop
    }
}
