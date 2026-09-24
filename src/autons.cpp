#include "autons.hpp"
#include "drivetrain.hpp"

static void none() {}

static void redLeft() {
    // chassis.setPose(-48, -60, 90);
    // chassis.moveToPoint(-24, -60, 2000);
}

static void redRight() {}

static void blueLeft() {}

static void blueRight() {}

static void skills() {}

const Auton autons[] = {
    {"None", none},
    {"Red Left", redLeft},
    {"Red Right", redRight},
    {"Blue Left", blueLeft},
    {"Blue Right", blueRight},
    {"Skills", skills},
};
const int autonCount = sizeof(autons) / sizeof(autons[0]);
int selectedAuton = 0;

void runAuton() {
    chassis.setPose(0, 0, 0);  // autons can override this
    autons[selectedAuton].run();
}
