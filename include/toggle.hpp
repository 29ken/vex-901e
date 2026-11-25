#pragma once

// The toggle flapper on the back of the robot (V4 CAD: rear top, driven by the
// ratchet motor) and the optical sensor that looks at the field toggle.
//
// NOT WIRED YET. The ratchet motor's port, the optical sensor's port, and which
// way the ratchet turns the flapper get decided at practice. Until then
// setToggle() only waits as long as a flip should take, so routes keep their
// timing in the simulator, and toggleColor() says Unknown.

enum class ToggleColor { Unknown, Red, Blue, Yellow };

// Flips the toggle behind the robot to our alliance's colour. Call it with the
// robot's back against the wall at the toggle.
void setToggle(bool blueAlliance);
ToggleColor toggleColor();

inline constexpr int TOGGLE_FLIP_MS = 600;  // TODO: time a real flip at practice
