#include "autons.hpp"
#include "drivetrain.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

static void none() {}

static void redRight() {
    // chassis.setPose(-48, -60, 90);
    // chassis.moveToPoint(-24, -60, 2000);
}


static void redLeft5pin() {
//initial pos
chassis.setPose(-63, -8, 270);

//Toggle 
chassis.turnToPoint(-66, -8, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-66, -8, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-63, -8, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-63, -8, 1000, {.forwards=true, .maxSpeed=127}, false);

//head to red goal
chassis.turnToPoint(-54, -17, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-54, -17, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-62, -24, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-62, -24, 1000, {.forwards=true, .maxSpeed=127}, false);

//release claw
chassis.turnToPoint(-57, -24, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-57, -24, 1000, {.forwards=true, .maxSpeed=127}, false);

//head to match load side's pin + claw 
chassis.turnToPoint(-52, -41, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-52, -41, 1000, {.forwards=true, .maxSpeed=127}, false);

//red goal + unclaw 
chassis.turnToPoint(-50, -33, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-50, -33, 1000, {.forwards=true, .maxSpeed=127}, false);

//head to other outside pin + claw
chassis.turnToPoint(-31, -26, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-31, -26, 1000, {.forwards=true, .maxSpeed=127}, false);

//head to black goal + unclaw
chassis.turnToPoint(-46, 16, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-46, 16, 1000, {.forwards=true, .maxSpeed=127}, false);

//head to left pin 
chassis.turnToPoint(-63, 22, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-63, 22, 1000, {.forwards=true, .maxSpeed=127}, false);

//head to black goal + unclaw
chassis.turnToPoint(-57, 23, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-57, 23, 1000, {.forwards=true, .maxSpeed=127}, false);
}

static void blueLeft() {}

static void blueRight() {}

static void skills() {} //X 3 pin matchload
