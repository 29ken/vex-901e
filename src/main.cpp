#include "main.h"
#include "drivetrain.hpp"
#include "controls.hpp"
#include "autons.hpp"
#include "config.hpp"
#include "lift.hpp"
#include "pros/adi.hpp"
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

 
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::Motor intake(INTAKE);
    pros::adi::DigitalOut wrist ('A');
    bool wrist_retracted = false;
    pros::adi::DigitalOut claw ('B');
    bool claw_extended = false;
    

    applyDriverBrakes();


    while (true) {
        // 1. Arcade Drivetrain
        if (tuning == TUNE_NONE) {
            driveControl();
        }

        // 2. Lift: R2 up and R1 down by hand. L1 steps up through the preset
        // heights. Tap L2 to go to the bottom; hold it to re-zero (see lift.hpp).
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            liftManual(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            liftManual(-127);
        } else {
            liftManual(0);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) liftNextPreset();
        liftDownButton(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));

        // 3. Intake: hold B to pull in, Y to push out
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            intake.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            intake.move(-127);
        } else {
            intake.move(0);
        }

        // 4. Piston control with A button. new_press only fires once per
        // press, so holding A can't toggle it over and over.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            wrist_retracted = !wrist_retracted;
            wrist.set_value(wrist_retracted);

            claw_extended = !claw_extended;
            claw.set_value(claw_extended);
        }

        pros::delay(10); // Keep just this one delay at the bottom
    }
}
