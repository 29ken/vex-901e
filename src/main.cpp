#include "main.h"
#include "drivetrain.hpp"
#include "controls.hpp"
#include "autons.hpp"
#include "config.hpp"
#include "pros/adi.hpp"
#include "tuner.hpp"
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

 
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::MotorGroup lift({LIFT_LEFT, LIFT_RIGHT});
    pros::adi::DigitalOut wrist ('A');
    bool wrist_retracted = false;
    pros::adi::DigitalOut claw ('B');
    bool claw_extended = false;
    

    applyDriverBrakes();
    // Set brake mode ONCE here before the loop begins:
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);


    while (true) {
        // 1. Arcade Drivetrain
        if (tuning == TUNE_NONE) {
            driveControl();
        }

        // 2. Lift control: R2 up, R1 down
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            lift.move(127);       // Move UP
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            lift.move(-127);      // Move DOWN
        } else {
            lift.move(0);         // Hold still
        }

        // 3. Piston control with A button. new_press only fires once per
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
