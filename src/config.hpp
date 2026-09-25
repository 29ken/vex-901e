#pragma once
#include <cstdint>

// Ports and tuning constants. Negative port = motor is reversed.

// Right drive
inline constexpr std::int8_t RIGHT_11W_A = 1;
inline constexpr std::int8_t RIGHT_5W    = 2;
inline constexpr std::int8_t RIGHT_11W_B = 3;

// Left drive, all mounted backwards
inline constexpr std::int8_t LEFT_5W    = -11;
inline constexpr std::int8_t LEFT_11W_A = -12;
inline constexpr std::int8_t LEFT_11W_B = -13;

// Port 4 is the radio.

inline constexpr float TRACK_WIDTH      = 11.5f;  // TODO: measure, left wheel center to right
inline constexpr float DRIVE_RPM        = 450.0f;
inline constexpr float HORIZONTAL_DRIFT = 2.0f;   // 8 if we ever put traction wheels on

inline constexpr int STICK_DEADBAND = 8;

// Turn the screens on or off. The brain's auton selector shows either way.
inline constexpr bool BRAIN_UI      = true;
inline constexpr bool CONTROLLER_UI = true;

// How much of full turn speed you get. Turning backs off as you drive
// faster so a diagonal stick arcs forward instead of spinning.
inline constexpr float TURN_WHEN_STILL = 0.80f;  // spin in place
inline constexpr float TURN_AT_SPEED   = 0.45f;  // full throttle

// Max change in turn output per 10 ms loop, out of 127.
inline constexpr float TURN_RAMP_UP   = 7.0f;
inline constexpr float TURN_RAMP_DOWN = 20.0f;

// Left stick within ~17 degrees of horizontal counts as a pure spin.
inline constexpr float SIDEWAYS_ZONE = 0.3f;
