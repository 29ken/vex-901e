#pragma once
#include <cstdint>

// Our ports and settings. The drive, IMU and distance sensors, and RW's own
// tuning, are in src/rw-config.cpp. Negative port = motor is reversed. Port 15
// is the radio.

// Claw piston (3-wire port letter) and the autograb distance sensor under it.
inline constexpr char CLAW_PORT = 'B';
inline constexpr std::int8_t AUTOGRAB_SENSOR = 3;
// Armed autograb closes the claw once something is closer than this (mm).
inline constexpr int AUTOGRAB_MM = 51;

// Cascade lift. If the two sides fight each other, flip the sign on one.
inline constexpr std::int8_t LIFT_LEFT  = -17;
inline constexpr std::int8_t LIFT_RIGHT = 16;

// Rotation sensor on the lift. Raising the lift should make "Lift" on the
// brain screen go up; if it goes down, make this -4.
inline constexpr std::int8_t LIFT_SENSOR = 4;

// Ratchet motor (drives the toggle flapper, and the winch one way: see
// toggle.hpp), the rotation sensor on the toggle shaft, and the optical sensor
// that looks at the field toggle. 0 = not plugged in yet. TODO: practice
// Positive power on the ratchet motor must be the way its pawls catch; if a
// flip moves the lift instead of the flapper, make RATCHET_MOTOR negative.
inline constexpr std::int8_t RATCHET_MOTOR = 0;
inline constexpr std::int8_t TOGGLE_SENSOR = 0;
inline constexpr std::int8_t TOGGLE_OPTICAL = 0;

// liftNextPreset() steps up through these heights, lowest first, in sensor
// degrees above the bottom. It isn't on a button yet (L1 arms autograb). To
// find one: raise the lift with R2 until it lines up with the pin or cone,
// then read "Lift" on the brain screen. The V4 winch is direct drive, so
// heights measured on the early-season robot don't carry over.
inline constexpr double LIFT_PRESETS[] = {300, 600, 900};  // TODO: measure

// Lift heights the autons use, in the same units. TODO: every one of these is
// a placeholder until it's read off the brain screen at practice.
inline constexpr double LIFT_GRAB = 0;             // picking up from the floor or a wall stack
inline constexpr double LIFT_CARRY = 120;          // holding something while driving
inline constexpr double LIFT_ALLIANCE_GOAL = 300;  // claw over a red/blue goal (3.25 in tall)
inline constexpr double LIFT_NEUTRAL_GOAL = 450;   // claw over a black goal (5.8 in tall)
inline constexpr double LIFT_PER_PIECE = 250;      // extra for each piece already on that goal

// How close the PID tuner's test moves have to land. Tighter = more exact, but
// slower to finish.
inline constexpr float DRIVE_EXACT_IN = 0.25f;
inline constexpr float TURN_EXACT_DEG = 0.5f;

inline constexpr int STICK_DEADBAND = 8;

// true: left stick only drives forward/back, right stick only turns.
// false: left stick does both, and the right stick can also turn.
inline constexpr bool SPLIT_ARCADE = false;

// Turn the screens on or off. The brain's auton selector shows either way.
inline constexpr bool BRAIN_UI      = true;
inline constexpr bool CONTROLLER_UI = true;

// Turn authority: less when sitting still, more at speed.
inline constexpr float TURN_SCALE_MIN = 0.8f;
inline constexpr float TURN_SCALE_MAX = 0.9f;

// Max change in turn output per 10 ms loop, out of 127.
inline constexpr float TURN_RAMP_UP   = 7.0f;
inline constexpr float TURN_RAMP_DOWN = 20.0f;

// Left stick within ~17 degrees of horizontal counts as a pure spin.
inline constexpr float SIDEWAYS_ZONE = 0.3f;

// Left stick within ~11 degrees of straight up or down drives straight.
inline constexpr float STRAIGHT_ZONE = 0.2f;
