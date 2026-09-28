#pragma once
#include <atomic>

enum TuneKind { TUNE_NONE, TUNE_TURN, TUNE_MOVE };

struct TuneResult {
    bool done = false;
    bool ok = false;
    const char* error = "";  // short enough for the controller screen
    float kP = 0;
    float kD = 0;
    float ku = 0;  // raw measurements, printed for reference
    float tu = 0;
};

// Which tune is running. Driver control pauses while this isn't TUNE_NONE.
extern std::atomic<TuneKind> tuning;
extern TuneResult turnTune;
extern TuneResult moveTune;

// Each runs in the background and applies the new gains when it finishes.
// Moving either stick aborts.
void startTurnTune();
void startMoveTune();
