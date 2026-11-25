// Simulator only: this whole file is empty on the robot. sh sim/run-sim.sh
// builds with RW_SIM_PROS and runs autons by name, e.g.
//   sh sim/run-sim.sh "Red Left 3" --start -62.61,-6,90
//   sh sim/run-sim.sh "Check: resets" --start -46,-46,180
// --start is where the simulated robot really is. Give the auton's own
// setPose(), or the simulator treats the auton's coordinates as relative and
// the walls won't be where it thinks. The replay is written to sim/out/.
#ifdef RW_SIM_PROS
#include "autons.hpp"
#include "drivetrain.hpp"
#include "claw.hpp"
#include "lift.hpp"
#include "resets.hpp"
#include "screens.hpp"
#include <cstdio>
#include <cstdlib>

static void printPose(const char* when) {
    std::printf("%-28s x %6.2f  y %6.2f  heading %6.1f\n", when, x_pos, y_pos, getInertialHeading());
}

// Run with --start -46,-46,180. The code is told it's 2 in off in X and Y, as
// if odometry had drifted, and the resets should pull it back to -46, -46.
static void checkResets() {
    setPose(-44, -48, 180);
    printPose("start (2 in off on purpose)");

    resetFrom({ResetSensor::Front, ResetSensor::Right});  // facing the -y wall, right side to the -x wall
    printPose("after front + right");

    turnToAngle(270, 1500);  // face the -x wall
    resetFrom({ResetSensor::Front, ResetSensor::Left});
    printPose("after turning, front + left");

    turnToAngle(250, 1500);  // 20 degrees off square: should be refused
    resetFrom(ResetSensor::Front);

    turnToAngle(270, 1500);
    clawGrab();  // with the claw closed the front sensor isn't trusted
    resetFrom(ResetSensor::Front);
    clawRelease();
    printPose("end (should be -46, -46)");
}

// Run with any --start. The lift should rise while the robot drives.
static void checkLiftAndClaw() {
    std::uint32_t t0 = pros::millis();
    liftTo(600);
    std::printf("lift moving after liftTo(600): %s\n", liftMoving() ? "yes" : "no");
    driveTo(24, 2000);
    std::printf("drove 24 in by %u ms, lift at %.0f, still moving: %s\n", pros::millis() - t0,
                liftStatus().height, liftMoving() ? "yes" : "no");
    bool there = liftWait();
    std::printf("liftWait: %s at %.0f, %u ms\n", there ? "there" : "NOT there", liftStatus().height, pros::millis() - t0);

    // The simulator doesn't simulate the autograb sensor, so this should time out.
    bool grabbed = clawGrabWhenSeen(300);
    std::printf("clawGrabWhenSeen(300): %s, claw %s\n", grabbed ? "grabbed" : "timed out", clawClosed() ? "closed" : "open");
    clawGrab();
    std::printf("clawGrab: claw %s\n", clawClosed() ? "closed" : "open");

    liftTo(0);
    there = liftWait();
    std::printf("liftTo(0) + liftWait: %s at %.0f, %u ms\n", there ? "there" : "NOT there", liftStatus().height,
                pros::millis() - t0);
}

static void printPreviews() {
    for (ResetSensor s : {ResetSensor::Front, ResetSensor::Back, ResetSensor::Left, ResetSensor::Right}) {
        ResetPreview p = previewReset(s);
        if (p.ok) {
            std::printf("  preview %-5s %5.1f in -> %c %.2f\n", resetSensorName(s), p.readingIn, p.axis, p.newPos);
        } else {
            std::printf("  preview %-5s %5.1f in    %s\n", resetSensorName(s), p.readingIn, p.why);
        }
    }
}

// Run with --start -46,-43.34,180: the front sensor's lens is then exactly 24 in
// from the -y wall. The simulator's sensors read 1.2% long, so the Distance
// page should print SCALE_FRONT close to 0.988. The code is told it's 1.5 in off
// in X and Y, so the previews should show the corrections.
static void checkDistancePage() {
    setPose(-44.5, -44.84, 180);
    printPreviews();
    showDistancePage(24);
    pros::delay(3000);  // the brain screen averages 2 s, then prints the scale to the terminal
    turnToAngle(195, 1000);  // 15 degrees off square: every preview should say so
    printPreviews();
}

void registerSimAutons() {
    // Leaves out "None" (index 0): RW's list holds 12, and ours plus the checks would be 13.
    for (int i = 1; i < autonCount; i++) addAuton(autons[i].name, autons[i].run);
    // RW's list holds 12. These go last so the robot's autons keep their numbers.
    addAuton("Check: resets", checkResets);
    addAuton("Check: lift and claw", checkLiftAndClaw);
    // RW's list is full: the route lab takes the last place when a script is given.
    if (std::getenv("ROUTE_SCRIPT")) addAuton("Lab", labRoute);
    else addAuton("Check: distance page", checkDistancePage);
}
#endif
