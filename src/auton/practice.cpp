// Set-up and test runs picked like an auton: Cal: ... and Test: ... See auton.hpp.
#include "auton.hpp"

namespace auton {

// Practice set-up, picked like an auton. Both need about 4 ft of clear floor
// in front of and behind the robot (the drive log: on every side).
// Measures the drive's feedforward for RW's curve follower: copy the three
// numbers it shows on the brain into drive_kS, drive_kV and drive_kA in
// src/rw-config.cpp. It never returns: stop the program when it's done.
void calFeedforward() { measureDriveFeedforward(); }
// Records about 15 s of driving for the simulator's fitter (sim/fit-profile.sh):
// saves rw_drive_log.csv to the SD card, or prints it to the terminal without one.
void calDriveLog() { calibrateDrive(); }

// ---- PID test moves (practice only) -------------------------------------------
// One clean move at a time, so a PID can be judged in a few seconds without a
// full route: how far off it ended, how far it overshot, and how long it took
// to settle. A background task records the heading (or the distance driven)
// every 10 ms while the move runs and for a moment after it.
constexpr int TRACE_MAX = 600;            // 6 s
constexpr int TEST_AFTER_MS = 400;        // keeps recording this long after the move returns
constexpr double TEST_TURN_BAND = 1.0;    // degrees: "settled" once it stays inside this
constexpr double TEST_DRIVE_BAND = 0.5;   // inches
double traceValue[TRACE_MAX];
std::atomic<int> traceCount{0};
std::atomic<bool> traceOn{false};
double (*traceRead)() = nullptr;
bool traceTaskStarted = false;
double testX0 = 0, testY0 = 0, testH0 = 0;

void traceTask(void*) {
    while (true) {
        if (traceOn && traceRead) {
            int n = traceCount;
            if (n < TRACE_MAX) {
                traceValue[n] = traceRead();
                traceCount = n + 1;
            }
        }
        pros::delay(10);
    }
}

double readHeading() { return getInertialHeading(); }
// Inches driven from where the test move started, along the way the robot faced then.
double readDriven() {
    double h = testH0 * M_PI / 180;
    return (x_pos - testX0) * std::sin(h) + (y_pos - testY0) * std::cos(h);
}

// Runs `move`, which should take the reading from `from` to `to`, and prints what happened.
template <typename Move>
void testMove(const char* what, const char* unit, double from, double to, double band, double (*read)(), Move move) {
    if (!traceTaskStarted) {
        traceTaskStarted = true;
        pros::Task task(traceTask, nullptr, "Test trace");
    }
    traceRead = read;
    traceCount = 0;
    traceOn = true;
    std::uint32_t began = pros::millis();
    move();
    double returned = (pros::millis() - began) / 1000.0;
    pros::delay(TEST_AFTER_MS);
    traceOn = false;
    int n = traceCount;
    if (n < 2) {
        say("Test %s: nothing recorded\n", what);
        return;
    }
    double way = to >= from ? 1 : -1, goal = std::fabs(to - from);
    double peak = -1e9;
    int lastOutside = -1;
    for (int i = 0; i < n; i++) {
        double done = (traceValue[i] - from) * way;  // progress toward the target
        peak = std::fmax(peak, done);
        if (std::fabs(done - goal) > band) lastOutside = i;
    }
    double ended = (traceValue[n - 1] - from) * way - goal;  // + is past the target
    double overshoot = std::fmax(0.0, peak - goal);
    if (lastOutside == n - 1) {
        say("Test %s %+.1f %s: ended %+.2f %s off, overshot %.2f, NOT settled within %.1f %s (move returned at %.2f s)\n",
            what, goal * way, unit, ended, unit, overshoot, band, unit, returned);
    } else {
        say("Test %s %+.1f %s: ended %+.2f %s off, overshot %.2f, settled within %.1f %s after %.2f s (move returned at %.2f s)\n",
            what, goal * way, unit, ended, unit, overshoot, band, unit, (lastOutside + 1) * 0.01, returned);
    }
}

constexpr int TEST_PAUSE_MS = 700;  // standing still between test moves

// Turns `degrees` (up to 180) from where the robot faces now: + is right (clockwise).
void testTurnBy(double degrees) {
    double from = getInertialHeading();
    double to = from + degrees;
    testMove("turn", "deg", from, to, TEST_TURN_BAND, readHeading, [to] { turnToAngle(to, 2000); });
    pros::delay(TEST_PAUSE_MS);
}

void testDriveBy(double inches) {
    testX0 = x_pos;
    testY0 = y_pos;
    testH0 = getInertialHeading();
    testMove("drive", "in", 0, inches, TEST_DRIVE_BAND, readDriven, [inches] { driveTo(inches, 2500); });
    pros::delay(400);
}

// Turn PID: 90 right twice, 90 left twice, 180 right, 180 left, 45 right, 45 left,
// with a pause between. About 13 s in the simulator, close to Timed Run's 15 s,
// so the controller's Programming Skills mode (60 s) is the safe way to run it.
// Needs room to spin; no pieces.
// Tune turn_kp / turn_ki / turn_kd in rw-config.cpp from what it prints.
void testTurn() {
    say("Test: turn. Gains now: kp %.3f ki %.3f kd %.3f\n", turn_kp, turn_ki, turn_kd);
    // Each size twice or both ways, so one lucky or unlucky turn doesn't decide it.
    testTurnBy(90);
    testTurnBy(90);
    testTurnBy(-90);
    testTurnBy(-90);
    testTurnBy(180);
    testTurnBy(-180);
    testTurnBy(45);    // a smaller turn: small turns settle differently from big ones
    testTurnBy(-45);
}

// Move PID: 24 in forward and back, twice. Needs 3 ft clear in front.
// Tune distance_kp / distance_ki / distance_kd. The distance is what the robot
// measures itself, so do Cal dist first, and check one run with a tape measure.
void testDrive() {
    say("Test: drive. Gains now: kp %.3f ki %.3f kd %.3f\n", distance_kp, distance_ki, distance_kd);
    testDriveBy(24);
    testDriveBy(-24);
    testDriveBy(24);
    testDriveBy(-24);
}

}  // namespace auton
