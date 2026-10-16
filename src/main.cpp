#include "main.h"
#include "drivetrain.hpp"
#include "controls.hpp"
#include "autons.hpp"
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
    pros::MotorGroup lift({-16, 17}); // Use -17 if one motor lifts backwards
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

        // 2. Lift control with R1 and R2
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            lift.move(127);       // Move UP
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            lift.move(-127);      // Move DOWN
        } else {
            lift.move(0);         // Hold still
        }

        pros::delay(10); // Keep just this one delay at the bottom


// 3. Piston control with A button
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        wrist_retracted = !wrist_retracted;
        wrist.set_value(wrist_retracted);
        
        pros::delay(100000); // Delay to prevent rapid toggling
        
        claw_extended = !claw_extended;
        claw.set_value(claw_extended);

    }

    pros::delay(10);
}

    }





