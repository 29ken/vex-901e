#include "autons.hpp"
#include "autonroutes.cpp"
#include "drivetrain.hpp"


const Auton autons[] = {
    {"None", none},
    {"Red Left", redLeft5pin},
    {"Red Right", redRight},
    {"Red Right 2pin", redRight2pin},
    {"Red Left 4pin", redLeft4pin},
    {"Blue Left", blueLeft},
    {"Blue Right", blueRight},
    {"Skills", skills},
};
const int autonCount = sizeof(autons) / sizeof(autons[0]);
int selectedAuton = 0;

void runAuton() {
    // Hold so the robot stops dead at the end of each move instead of rolling on.
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(0, 0, 0);  // autons can override this
    autons[selectedAuton].run();
}
