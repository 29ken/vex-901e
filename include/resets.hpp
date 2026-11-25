#pragma once
#include <initializer_list>

// Distance-sensor position resets for autons. Each call waits for the robot to
// stop, checks the reading before trusting it, prints what it did to the PROS
// terminal, and leaves the position alone if anything looks wrong. RW's own
// filters in rw-config.cpp (beam angle, corner margin, max jump) apply too.
//
// Use it right after a move that ends square to a wall, e.g.
//   turnToAngle(180, 800);
//   resetFrom({ResetSensor::Back, ResetSensor::Left});
// Needs field-centred coordinates (setPose() with the walls at +-70.25 in).

enum class ResetSensor { Front, Back, Left, Right };

// True if it corrected X or Y.
bool resetFrom(ResetSensor sensor);
// How many of them corrected X or Y.
int resetFrom(std::initializer_list<ResetSensor> sensors);

// Tuning. Every number here is a starting guess until it's tested on a field.

// Only trust a reading when the robot is this close to square with the walls.
// Clear (polycarbonate) field walls echo back only near square, about the
// beam's half-width (~12 degrees, a forum figure); past that the beam may
// see through the wall. Solid walls would allow more.
inline constexpr double RESET_MAX_OFF_SQUARE_DEG = 12;

// Skip readings from farther than this (inches). The beam spreads about 0.4 in
// per inch of distance, and the sensor is only ~5% accurate past 8 in.
inline constexpr double RESET_MAX_RANGE_IN = 30;
// The back sensor (lens 7.95 in up) starts catching the toggle (10.7 in up)
// past about 12 in from a wall centre.
inline constexpr double RESET_BACK_MAX_RANGE_IN = 12;

// The front sensor sits under the claw. With the claw closed on a piece, trust
// it only when the lift has the piece above the beam (lift sensor degrees).
// Not measured yet, so for now never while the claw is closed.
inline constexpr double RESET_FRONT_CLEAR_LIFT = 1e9;  // TODO: measure at practice

// Wait up to this long (ms) for the robot to be still before reading: a reading
// lags ~40 ms, so one taken while moving is off by the distance covered.
inline constexpr int RESET_SETTLE_MS = 250;
inline constexpr double RESET_STILL_IPS = 1.0;  // slower than this counts as still (in/s)
inline constexpr double RESET_STILL_DPS = 5.0;  // turning slower than this too (deg/s)

// Takes this many readings this far apart (ms; the sensor updates about every
// 33) and uses the middle one. They have to agree within RESET_MAX_SPREAD_IN,
// or something is moving in the beam. RW's simulator puts about +-0.35 in of
// noise on a reading at 20 in, so 0.75 threw out good readings there; the real
// sensor's noise is unknown until practice.
inline constexpr int RESET_SAMPLES = 3;
inline constexpr int RESET_SAMPLE_MS = 35;
inline constexpr double RESET_MAX_SPREAD_IN = 1.5;
