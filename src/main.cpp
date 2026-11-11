#include "main.h"
#include "drivetrain.hpp"
#include "controls.hpp"
#include "autons.hpp"
#include "config.hpp"
#include "lift.hpp"
#include "pto.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "tuner.hpp"
#include "screens.hpp"

void initialize() {
    initDrive();
    initPto();
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
    pros::adi::DigitalOut wrist('C');
    pros::adi::DigitalOut claw('B');
    pros::Distance distance_sensor(3);

    bool wrist_retracted = true;
    bool claw_extended = false;
    bool autograb_armed = false;

    // pistons starting variables
    wrist.set_value(wrist_retracted);
    claw.set_value(claw_extended);

    applyDriverBrakes();

    while (true) {
        // 1. Drivetrain
        if (tuning == TUNE_NONE) {
            driveControl();
        }

        // 2. Lift: R2 up, R1 down. If the PTO is in INTAKE, it shifts to LIFT
        // first (~0.35 s), then the cascade moves on all three motors.
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            liftManual(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            liftManual(-127);
        } else {
            liftManual(0);
        }

        // 3. Intake: B in, Y out. If the PTO is in LIFT, it shifts to INTAKE first
        // (~0.35 s), then the intake runs while the two lift motors take the
        // cascade to clamp height (PTO_CLAMP_HEIGHT in pto.hpp).
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            intakeMove(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            intakeMove(-127);
        } else {
            intakeMove(0);
        }

        // 3b. DOWN: shift the PTO now (INTAKE <-> LIFT). R2 shifts by itself;
        // DOWN on the way to the goal just saves the ~0.35 s pause when lifting.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            ptoToggle();
        }

        // 4. A: manual toggle of wrist + claw
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            wrist_retracted = !wrist_retracted;
            claw_extended = !claw_extended;
            wrist.set_value(wrist_retracted);
            claw.set_value(claw_extended);
        }

        // 4b. The cascade reached clamp height after a switch to INTAKE: get the
        // claw ready for what the intake brings in. Wrist down, claw open,
        // autograb armed, as if the driver had set it up.
        if (ptoClawReady()) {
            wrist_retracted = false;
            claw_extended = false;
            wrist.set_value(wrist_retracted);
            claw.set_value(claw_extended);
            autograb_armed = true;
        }

        // 5. L1: arm/disarm the sensor + autograb
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            autograb_armed = !autograb_armed;
        }

        // 6. L2: let go of the pin (claw only)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            claw_extended = false;
            claw.set_value(claw_extended);
        }

        // 7. Autograb: only runs while armed, grabs once, then disarms itself
        if (autograb_armed) {
            int32_t d = distance_sensor.get();
            if (d > 0 && d < 51) {
                claw_extended = true;
                wrist_retracted = true;
                claw.set_value(claw_extended);
                wrist.set_value(wrist_retracted);
                autograb_armed = false;
            }
        }

        pros::delay(10);  // inside da loop
    }
}

// void opcontrol() {

 
//     pros::Controller master(pros::E_CONTROLLER_MASTER);
//     pros::Motor intake(INTAKE);
//     pros::adi::DigitalOut wrist ('C');
//     bool wrist_retracted = true;
//     pros::adi::DigitalOut claw ('B');
//     bool claw_extended = false;
//     pros::Distance distance_sensor(3); //<-- port for distance sensor
//     bool distance_sensor_triggered = false; // Flag to track if the distance sensor has been triggered
//     // Add this with your other declarations (before the loop)
//     bool auto_grab_done = false;
    

//     applyDriverBrakes();


//     while (true) {
//         // 1. Arcade Drivetrain
//         if (tuning == TUNE_NONE) {
//             driveControl();
//         }

//         // 2. Lift: R2 up and R1 down by hand. L1 steps up through the preset
//         // heights. Tap L2 to go to the bottom; hold it to re-zero (see lift.hpp).
//         if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
//             liftManual(127);
//         } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
//             liftManual(-127);
//         } else {
//             liftManual(0);
//         }
//         // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) liftNextPreset();
//         // liftDownButton(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));

//         // 3. Intake: hold B to pull in, Y to push out
//         if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
//             intake.move(127);
//         } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
//             intake.move(-127);
//         } else {
//             intake.move(0);
//         }

//         // 4. Piston control with A button. new_press only fires once per
//         // press, so holding A can't toggle it over and over.
//         if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
//             wrist_retracted = !wrist_retracted;
//             wrist.set_value(wrist_retracted);

//             claw_extended = !claw_extended;
//             claw.set_value(claw_extended);
//         }

//         // 5. Claw on its own with L1 (temporary, for testing the claw).
//         // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
//         //     wrist_retracted = !wrist_retracted;
//         //     wrist.set_value(wrist_retracted);
//         // }

//         // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
//         //     claw_extended = !claw_extended;
//         //     claw.set_value(claw_extended);
//         // }
//                // 5. L1 toggles sensor on/off AND wrist

//         }

    

//     if (distance_sensor_triggered) {
//         int distance_mm = distance_sensor.get();
//             if (distance_mm > 0 && distance_mm < 51) {

//             (distance_sensor_triggered = !distance_sensor_triggered); 
//             // Only auto grab ONCEEEEEE
//             if (!auto_grab_done) {
//                 auto_grab_done = true;
//                 claw_extended = true;
//                 claw.set_value(claw_extended);
//                 wrist_retracted = true;
//                 wrist.set_value(wrist_retracted);
//             }
//             // Press L1 to release while pin is still close
//             if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
//                 claw_extended = false;
//                 claw.set_value(claw_extended);
//                 wrist_retracted = false;
//                 wrist.set_value(wrist_retracted);
//             }
//         } else {
//             // Pin moved away ---> reset so auto-grab is ready for next time
//             auto_grab_done = false;
//         }
//             pros::delay(10); // Keep just this one delay at the bottom 
                  
//                 }
// }
        
        





