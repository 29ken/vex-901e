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
    pros::adi::DigitalOut wrist ('A', true);
    bool wrist_retracted = true;
    pros::adi::DigitalOut claw ('C');
    bool claw_closed = false;
    bool claw_armed = false;  // waiting for the distance sensor to see something
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

        // 4. Wrist: L2 flips it. It starts retracted. new_press only fires once
        // per press, so holding the button can't toggle it over and over.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            wrist_retracted = !wrist_retracted;
            wrist.set_value(wrist_retracted);
        }

        // 5. Claw on L1. It starts open. First press turns on the distance
        // sensor, and the claw closes by itself once something is closer than
        // 51 mm (2 inches). Once it's closed, L1 opens it. Pressing L1 while
        // it's still waiting closes it by hand, in case the sensor misses.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (claw_closed) {
                claw_closed = false;
            } else if (claw_armed) {
                claw_closed = true;
                claw_armed = false;
            } else {
                claw_armed = true;
            }
        }
        int distance_mm = distance_sensor.get();
        if (claw_armed && distance_mm > 0 && distance_mm < 51) {
            claw_closed = true;
            claw_armed = false;
        }
        claw.set_value(claw_closed);  // if the claw opens when it should close, make this !claw_closed
        pros::delay(10); // Keep just this one delay at the bottom
    }
}



