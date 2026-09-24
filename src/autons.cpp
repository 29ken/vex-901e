#include "autons.hpp"
#include "drivetrain.hpp"

using drive::chassis;

namespace autons {
namespace {

void none() {}

void redLeft() {
    // chassis.setPose(-48, -60, 90);
    // chassis.moveToPoint(-24, -60, 2000);
}

void redRight() {}

void blueLeft() {}

void blueRight() {}

void skills() {}

} // namespace

const Routine list[] = {
    {"None", none},
    {"Red Left", redLeft},
    {"Red Right", redRight},
    {"Blue Left", blueLeft},
    {"Blue Right", blueRight},
    {"Skills", skills},
};
const int count = sizeof(list) / sizeof(list[0]);
int selected = 0;

void run() {
    chassis.setPose(0, 0, 0);  // routines can override this
    list[selected].run();
}

} // namespace autons
