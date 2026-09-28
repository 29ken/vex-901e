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

inline constexpr float TRACK_WIDTH          = 11.5f;  // TODO: measure, left wheel center to right
inline constexpr float DRIVE_WHEEL_DIAMETER = 2.75f;
inline constexpr float DRIVE_RPM            = 450.0f;
inline constexpr float HORIZONTAL_DRIFT     = 2.0f;   // 8 if we ever put traction wheels on

// Odom sensors. Leave a port at 0 until it's plugged in; LemLib falls back to
// the drive encoders for anything missing.
inline constexpr std::uint8_t IMU_PORT            = 0;
inline constexpr std::int8_t  VERTICAL_POD_PORT   = 0;  // rotation sensor, negative = reversed
inline constexpr std::int8_t  HORIZONTAL_POD_PORT = 0;
inline constexpr float POD_WHEEL_DIAMETER    = 2.125f;
inline constexpr float VERTICAL_POD_OFFSET   = 0.0f;    // inches from center of rotation, left is negative
inline constexpr float HORIZONTAL_POD_OFFSET = 0.0f;    // inches from center of rotation, behind is negative

// How close driveInches() / turnDegrees() land. Tighter = more exact, but
// slower to finish.
inline constexpr float DRIVE_EXACT_IN = 0.25f;
inline constexpr float TURN_EXACT_DEG = 0.5f;

inline constexpr int STICK_DEADBAND = 8;

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

// Left stick within ~7 degrees of straight up or down drives straight.
inline constexpr float STRAIGHT_ZONE = 0.12f;
