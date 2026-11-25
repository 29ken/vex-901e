// Simulator only: this whole file is empty on the robot. sh sim/run-sim.sh
// builds with RW_SIM_PROS and runs autons by name, e.g.
//   sh sim/run-sim.sh "Red Left" --start -62.61,-6,90
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
#include <cstdio>

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

void registerSimAutons() {
    for (int i = 0; i < autonCount; i++) addAuton(autons[i].name, autons[i].run);
    // RW's list holds 12. These two go last so the robot's autons keep their numbers.
    addAuton("Check: resets", checkResets);
    addAuton("Check: lift and claw", checkLiftAndClaw);
}
#endif
