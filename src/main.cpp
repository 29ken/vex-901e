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
    pros::adi::DigitalOut claw ('C');
    bool claw_extended = false;
    pros::Distance distance_sensor(1); //<-- port for distance sensor
    bool auto_lowering = false;  // this R2 press was handed to liftDown()


    applyDriverBrakes();


    while (true) {
        // 1. Arcade Drivetrain
        if (tuning == TUNE_NONE) {
            driveControl();
        }

        // 2. Lift: hold R1 to raise it. The bottom is wherever the lift was
        // when the program started. With the wrist up, one tap of R2 takes the
        // lift all the way down on its own (R1 stops it partway); with the
        // wrist down, R2 lowers it only while held.
        bool wrist_up = wrist_retracted;  // if this is backwards on the robot, make it !wrist_retracted
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            auto_lowering = wrist_up && liftDown();
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            liftManual(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !auto_lowering) {
            liftManual(-127);
        } else {
            liftManual(0);
        }

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

        // Claw on its own with L1 (temporary, for testing the claw).
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            claw_extended = !claw_extended;
            claw.set_value(claw_extended);
        }

        //5. Distance sensor. Autograb if the distance sensor reads less than 50mm. 
        // The autograb is initated when the distance is less than 51mm, which is 2 inches.
        int distance_mm = distance_sensor.get();
        if (distance_mm > 0 && distance_mm < 51) { 
            if (!claw_extended) {
                claw_extended = true;
                claw.set_value(claw_extended);
            }

        }
        pros::delay(10); // Keep just this one delay at the bottom
    }
}



