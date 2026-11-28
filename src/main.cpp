#include "main.h"
#include "drivetrain.hpp"
#include "autons.hpp"
#include "config.hpp"
#include "lift.hpp"
#include "tuner.hpp"
#include "screens.hpp"
#include "toggle.hpp"

void initialize() {
    initDrive();
    initLift();
    initToggle();
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
    // Autograb is always on in driver control: the claw closes by itself when
    // the sensor sees a piece in it. After a release it re-arms only once the
    // sensor has seen nothing closer than AUTOGRAB_CLEAR_MM for
    // AUTOGRAB_REARM_MS (the piece has left), so it doesn't grab the same piece again.
    const int AUTOGRAB_MM = 51, AUTOGRAB_CLEAR_MM = 90, AUTOGRAB_REARM_MS = 250;
    bool autograb_armed = false;
    bool claw_clear = false;
    std::uint32_t clear_since = 0;

    // pistons starting variables
    claw.set_value(claw_extended);

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
        // Closing is usually automatic (step 4); B is for when the sensor
        // misses a piece. (A is only the controller menu's + button.)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            claw_extended = !claw_extended;
            if (!claw_extended) autograb_armed = false;  // the piece just let go of is still in front of the sensor
            claw.set_value(claw_extended);
        }

        // 4. Autograb. An unplugged sensor reads PROS_ERR (huge): it arms, and never grabs.
        int32_t d = distance_sensor.get();
        if (!autograb_armed && !claw_extended) {
            if (d <= AUTOGRAB_CLEAR_MM) {
                claw_clear = false;
            } else if (!claw_clear) {
                claw_clear = true;
                clear_since = pros::millis();
            } else if (pros::millis() - clear_since >= static_cast<std::uint32_t>(AUTOGRAB_REARM_MS)) {
                autograb_armed = true;
            }
        } else {
            claw_clear = false;
        }
        if (autograb_armed && d > 0 && d < AUTOGRAB_MM) {
            claw_extended = true;
            claw.set_value(claw_extended);
            autograb_armed = false;
        }

        // 5. Toggle (include/toggle.hpp). R1, one press: flip the toggle next
        // to the flapper to our colour. R2, held: turn the flapper by hand.
        toggleManual(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2));
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            setToggleAsync(blueAlliance());
        }

        pros::delay(10);  // inside da loop
    }
}
