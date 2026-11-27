#pragma once

// Run log: saves each autonomous run to the SD card, so a run can be looked at
// afterwards without a laptop cabled to the brain while it happens.
//
// One file per run, /usd/run_NNN.csv (NNN counts up; the last number used is
// kept in /usd/run_count.txt). It only records: with no card in the brain, or
// in the simulator, nothing is written and the robot drives exactly the same.
//
// Lines in the file:
//   # ...                      header: the auton's name, battery, when it started
//   P,t,x,y,h,lift,claw,front,right,back,left
//                              20 times a second: seconds since the start, where
//                              the robot thinks it is (in, deg), lift height,
//                              claw (1 = closed), and the four distance sensors
//                              (in, -1 = sees nothing)
//   E,t,text                   every line the code prints to the terminal during
//                              the run (step times, grabs, "Goal seat", "Dist",
//                              "Wall contact", "Watch", ...)
//
// Put a FAT32 microSD card in the brain before turning it on, and don't pull it
// out while a program is running.

// Starts a log for this run and the background sampler. Safe with no card.
void runLogStart(const char* autonName);
// Ends it and closes the file. Also happens by itself RUN_LOG_MAX_S after the
// start, or when the robot is disabled, so a run cut short is still saved.
void runLogStop();
// True while a log file is open.
bool runLogging();

// Prints to the terminal like printf, and into the run log when one is open.
// The auton and subsystem code prints through this.
void say(const char* format, ...) __attribute__((format(printf, 1, 2)));

inline constexpr int RUN_LOG_SAMPLE_MS = 50;   // pose line every this often
inline constexpr int RUN_LOG_FLUSH_MS = 250;   // written through to the card this often
inline constexpr int RUN_LOG_MAX_S = 70;       // long enough for a 60 s skills run
