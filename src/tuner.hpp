#pragma once
#include <atomic>

// Everything that makes "drive 10 in" land on 10 in. Do them in order:
//   1. Cal dist and Cal turn, so odom measures true inches and degrees.
//      Paste the printed values into config.hpp and re-upload.
//   2. Turn PID, then Move PID. Each finds gains, then keeps adjusting them
//      until test moves land within DRIVE_EXACT_IN / TURN_EXACT_DEG.
//      Paste the printed gains into drivetrain.cpp.

enum TuneKind { TUNE_NONE, TUNE_TURN, TUNE_MOVE, TUNE_CAL_DIST, TUNE_CAL_TURN };

inline constexpr float CAL_INCHES = 48;  // two field tiles

struct TuneResult {
    bool done = false;
    bool ok = false;
    bool exact = false;      // test moves landed within the exact tolerance
    const char* error = "";  // short enough for the controller screen
    float kP = 0;
    float kI = 0;
    float kD = 0;
    float windup = 3;        // integral only builds this close to the target
    float ku = 0;            // raw relay measurements, printed for reference
    float tu = 0;
    float miss = 0;          // worst final error in the last round of test moves
};

struct CalResult {
    bool done = false;
    const char* label = "";  // short name for the controller
    float value = 0;
};

// What the tuner is busy with. Driver control pauses while it isn't TUNE_NONE.
extern std::atomic<TuneKind> tuning;
extern std::atomic<float> calReading;  // live inches/degrees while calibrating
extern TuneResult turnTune;
extern TuneResult moveTune;
extern CalResult distCal;
extern CalResult turnCal;

// Run in the background and apply the new gains when they finish.
void startTurnTune();
void startMoveTune();

// Start, then push the robot exactly CAL_INCHES in a straight line (or spin it
// exactly one full turn, either way) by hand, then confirm.
void startDistCal();
void startTurnCal();
void confirmCal();

// Stops whatever is running. Moving either stick does the same.
void cancelTune();
