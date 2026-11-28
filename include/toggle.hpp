#pragma once

// The ratchet motor, the toggle flapper it turns, and the optical sensor that
// looks at the field toggle (V4 CAD).
//
// How the ratchet works (from the CAD):
//  - The ratchet motor turns the ratchet axle 1:1. The axle ALWAYS turns the
//    toggle shaft, 3 turns of the axle to 1 of the flapper.
//  - Two pawls on a carrier join the axle to the winch one way only.
//    Motor forward (+): the pawls catch, so it helps the winch lift, and the
//    flapper turns too.
//    Motor reverse (-): the pawls click, so only the flapper turns. Toggles
//    are flipped this way.
//    Lift going down: the winch drags the axle backward, so the flapper turns
//    backward and the ratchet motor is spun by the lift. The motor is left to
//    coast so it doesn't fight the winch motors.
//
// NOT WIRED YET: the three ports in config.hpp are 0. With RATCHET_MOTOR 0,
// setToggle() only waits TOGGLE_FLIP_MS, so routes keep their timing.
//
// CHECK ON THE ROBOT (the lift's string isn't in the CAD, so it can't say which
// way the winch turns to lift): raise the lift by hand. The flapper should stay
// still (pawls clicking). Lower it by hand: the flapper should turn. If it's
// the other way round, the pawls are on backwards for this code.
// Then tap B: the flapper should turn and the lift should NOT move. If the lift
// moves instead, make RATCHET_MOTOR negative.

enum class ToggleColor { Unknown, Red, Blue, Yellow };

// Call once from initialize().
void initToggle();

// Turns the flapper one flip, then looks at the field toggle with the optical
// sensor. If it can see the toggle and it isn't our colour, flips again (up to
// TOGGLE_MAX_FLIPS in all). Returns true if it saw our colour, or flipped but
// couldn't see a colour to check; false if it never got our colour, the
// flapper jammed, or the motor isn't wired. Blocks until done.
bool setToggle(bool blueAlliance);
// The same, without waiting (for the driver's button). Does nothing if a flip
// is already going.
void setToggleAsync(bool blueAlliance);
// R2, called every loop with whether it is held: turns the flapper (the same
// way a flip does, so the lift doesn't move) for as long as it is. Ignored
// while an automatic flip is running.
void toggleManual(bool held);
// Turns the flapper (the flip direction) to its next rest angle. Lowering the
// lift drags the flapper round, so call this if the flaps are left in the way.
void parkToggle();

ToggleColor toggleColor();
const char* toggleColorName(ToggleColor c);

// Called by the lift's task every loop with the power it is sending the winch
// (+ up). Keeps the ratchet motor out of the lift's way, or helps it lift when
// RATCHET_ASSIST is on.
void ratchetWithLift(int liftPower);

struct ToggleStatus {
    bool motorWired, sensorWired, opticalWired;
    double angle;        // toggle shaft, 0-360
    ToggleColor color;
    bool flipping;
};
ToggleStatus toggleStatus();

// With no ratchet motor, or no rotation sensor, a flip is this long. In the
// simulator it is always this long. TODO: time a real flip at practice
inline constexpr int TOGGLE_FLIP_MS = 600;

// Degrees of the toggle shaft between one flap and the next. The back flapper
// in the CAD has a flap on each end of its arm, so 180: half a turn brings the
// next flap round. The side flaps are single (one per side, 360 apart), so a
// side-on flip may need a full turn: TODO check at practice.
inline constexpr double TOGGLE_FLAP_SPACING = 180;
// Toggle shaft angle (brain screen "Toggle") with the flaps parked clear of
// everything. TODO: read it off the brain screen at practice
inline constexpr double TOGGLE_REST_DEG = 0;
// Starting within this of a rest angle counts as parked: the flip is then one
// spacing. Starting anywhere else, it goes on to the rest angle after next.
inline constexpr double TOGGLE_PARK_TOLERANCE = 15;
// Cuts the power this many degrees before the rest angle; the flapper coasts the rest.
inline constexpr double TOGGLE_STOP_EARLY = 8;
inline constexpr int TOGGLE_FLIP_POWER = 127;
inline constexpr int TOGGLE_FLIP_TIMEOUT_MS = 2000;   // gives up: jammed, or the sensor isn't turning
inline constexpr int TOGGLE_MAX_FLIPS = 2;
// After a flip, how long the field toggle gets to settle before its colour is read.
inline constexpr int TOGGLE_SETTLE_MS = 120;

// Optical sensor: what counts as a colour. TODO: hold each toggle colour in
// front of the sensor at practice and read "Toggle" on the brain screen.
inline constexpr int TOGGLE_LED_PERCENT = 100;
inline constexpr double TOGGLE_MIN_SATURATION = 0.25;  // paler than this = can't tell

// true: the ratchet motor also runs forward whenever the lift goes up, adding
// its power. Leave off until the hand check above passes.
inline constexpr bool RATCHET_ASSIST = false;
