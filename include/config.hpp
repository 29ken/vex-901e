#pragma once
#include <cstdint>

// Our ports and settings. The drive, IMU, odom pods and distance sensors, and
// RW's own tuning, are in src/rw-config.cpp. Negative port = motor is reversed.
// Port 15 is the radio.

// Cascade lift. If the two sides fight each other, flip the sign on one.
inline constexpr std::int8_t LIFT_LEFT  = -17;
inline constexpr std::int8_t LIFT_RIGHT = 16;

// Rotation sensor on the lift. Raising the lift should make "Lift" on the
// brain screen go up; if it goes down, make this -4.
inline constexpr std::int8_t LIFT_SENSOR = 4;

// L1 steps up through these heights, lowest first, in sensor degrees above
// the bottom. To find one: raise the lift with R2 until it lines up with the
// pin or cone, then read "Lift" on the brain screen.
inline constexpr double LIFT_PRESETS[] = {300, 600, 900};  // TODO: measure

// Intake. Moved off port 16 to avoid clashing with LIFT_RIGHT.
inline constexpr std::int8_t INTAKE = 5;

// PTO piston's 3-wire port (the claw is B, the wrist C).
// TODO: make this the port the PTO solenoid is really plugged into.
inline constexpr char PTO_PISTON = 'A';
// The solenoid value that slides the gear onto the winch (LIFT). LIFT pulls the
// piston back, which is false on a normal solenoid. If DOWN moves the gear the
// wrong way, make this true.
inline constexpr bool PTO_PISTON_LIFT = false;

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
