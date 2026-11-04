#include "autons.hpp"
#include "drivetrain.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

static void none() {}


static void toggle() {
    chassis.turnToPoint(-64, -8, 500, {.forwards=true, .maxSpeed=127}, false);
    chassis.moveToPoint(-64, -8, 1000, {.forwards=true, .maxSpeed=127}, false);

    chassis.turnToPoint(-63, -8, 500, {.forwards=true, .maxSpeed=127}, false);
    chassis.moveToPoint(-63, -8, 1000, {.forwards=true, .maxSpeed=127}, false);
}
static void redRight() {
    chassis.setPose(-63, -8, 270);
    chassis.moveToPoint(-63, -8, 2000);
}

static void redRight2pin(){
//starting position
chassis.setPose(-7, -64, 180);

//toggle 
chassis.turnToPoint(-7, -65, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-7, -65, 1000, {.forwards=true, .maxSpeed=127}, false);
//head to red goal + score 1 pin 
chassis.turnToPoint(-16, -56, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-16, -56, 1000, {.forwards=true, .maxSpeed=127}, false);


chassis.turnToPoint(-24, -56, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-24, -56, 1000, {.forwards=true, .maxSpeed=127}, false);

//get pin + cup
chassis.turnToPoint(-24, -62, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-24, -62, 1000, {.forwards=true, .maxSpeed=127}, false);

//head to red goal + score 2nd pin, +1 pin + cup
chassis.turnToPoint(-24, -59, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-24, -59, 1000, {.forwards=true, .maxSpeed=127}, false);


}
static void redLeft4pin() {

//initial pos 
chassis.setPose(-63, -8, 270);

//toggle
chassis.turnToPoint(-64, -8, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-64, -8, 1000, {.forwards=true, .maxSpeed=127}, false);


chassis.turnToPoint(-56, -16, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-56, -16, 1000, {.forwards=true, .maxSpeed=127}, false);


//head to red goal + score 1 pin
chassis.turnToPoint(-56, -24, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-56, -24, 1000, {.forwards=true, .maxSpeed=127}, false);


//grab pin + cup
chassis.turnToPoint(-61, -24, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-61, -24, 1000, {.forwards=true, .maxSpeed=127}, false);


// head to red goal + score 2nd pin + 1 pin + cup
chassis.turnToPoint(-58, -24, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-58, -24, 1000, {.forwards=true, .maxSpeed=127}, false);


//head to other side (black goal left)
chassis.turnToPoint(-57, 24, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-57, 24, 1000, {.forwards=true, .maxSpeed=127}, false);

//collect pin + cup
chassis.turnToPoint(-61, 24, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-61, 24, 1000, {.forwards=true, .maxSpeed=127}, false);

//score black goal + 1 pin + cup
chassis.turnToPoint(-59, 24, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-59, 24, 1000, {.forwards=true, .maxSpeed=127}, false);

//go to other side collect pin + cup
chassis.turnToPoint(-29, -17, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-29, -17, 1000, {.forwards=true, .maxSpeed=127}, false);
//score red goal + 1 pin + cup
chassis.turnToPoint(-38, -20, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-38, -20, 1000, {.forwards=true, .maxSpeed=127}, false);
  
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

static void skills() {

chassis.setPose(-63, -9, 270);

chassis.turnToPoint(-64, -9, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-64, -9, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-57, -16, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-57, -16, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-58, -24, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-58, -24, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-61, -24, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-61, -24, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-56, -24, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-56, -24, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-56, -60, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-56, -60, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-61, -60, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-61, -60, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-48, -35, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-48, -35, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-61, -60, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-61, -60, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-35, -51, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-35, -51, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-40, -50, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-40, -50, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-35, -49, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-35, -49, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-60, -59, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-60, -59, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-24, -62, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-24, -62, 1000, {.forwards=true, .maxSpeed=127}, false);

chassis.turnToPoint(-24, -59, 500, {.forwards=true, .maxSpeed=127}, false);
chassis.moveToPoint(-24, -59, 1000, {.forwards=true, .maxSpeed=127}, false);} //X 3 pin matchload
