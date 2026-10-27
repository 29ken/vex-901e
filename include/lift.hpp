#pragma once
#include "api.h"
#include "config.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

extern pros::MotorGroup lift;

// Starts the lift's background task and calls wherever the lift is right now
// height 0. Call once from initialize(), with the lift all the way down.
void initLift();

// Hand control from R1/R2, + is up. Anything but 0 cancels a preset move.
void liftManual(int power);
// Goes to the next preset height above where the lift is, or is already headed.
void liftNextPreset();
// Goes down to height 0 and stops there. False if it can't (no sensor, not
// zeroed, or already at the bottom), so the caller can lower by hand instead.
bool liftDown();
// The L2 button, called every loop with whether it's held. A tap is
// liftDown(). Holding it lowers the lift slowly past height 0; let go when it
// touches the bottom and that becomes the new 0. Use it if the program was
// started with the lift up.
void liftDownButton(bool held);

struct LiftStatus {
    bool sensorOk;
    bool zeroed;
    bool backwards;  // the sensor counts down when the lift goes up
    double height;   // sensor degrees above the bottom
};
LiftStatus liftStatus();

// Tuning, all in lift sensor degrees.
inline constexpr double LIFT_KP = 0.5;            // power per degree off target
// Speed of the last stretch to a preset. It has to beat the weight of the lift
// plus whatever it's carrying; if a preset stops short, raise this.
inline constexpr int LIFT_MIN_POWER = 25;
inline constexpr double LIFT_TOLERANCE = 10;      // presets closer than this to the lift are skipped
inline constexpr std::uint32_t LIFT_MOVE_TIMEOUT_MS = 3000;

// The winch has no stop at the bottom: unwound too far, it winds the string
// the other way and lifts the lift back up. So nothing drives below this.
inline constexpr double LIFT_BOTTOM = 5;
// Holding L2 this long lowers slowly past the bottom limit to re-zero.
inline constexpr std::uint32_t LIFT_REZERO_HOLD_MS = 600;
inline constexpr int LIFT_REZERO_POWER = 30;

// Moved this far the wrong way while pushing at least this hard = sensor
// backwards. Low power doesn't count: a heavy load can sag while creeping.
inline constexpr double LIFT_WRONG_WAY = 20;
inline constexpr int LIFT_WRONG_WAY_POWER = 80;

// The top is a real hard stop. Pushing at near-full power but barely turning
// for this long means it's there; part power doesn't count, so a heavy load
// hanging still mid-air isn't mistaken for the top.
inline constexpr int LIFT_TOP_STALL_POWER = 100;
inline constexpr double LIFT_STALL_RPM = 5;
inline constexpr std::uint32_t LIFT_STALL_MS = 250;

// Near either end, slow down instead of slamming into it.
inline constexpr double LIFT_SLOW_ZONE = 120;
inline constexpr double LIFT_TOP_MARGIN = 15;     // stop this far under the learned top
inline constexpr int LIFT_SLOW_POWER = 40;

// Everything below is the lift's logic, kept apart from the motors so
// test/lift_check.cpp can run it against a simulated winch.

// Learns where the top is from the first full-power stall going up, then slows
// near it and stops just under it. Works in raw sensor degrees, so re-zeroing
// doesn't move it.
// ponytail: the first stall wins, so a snag on the way up becomes the top until
// the program restarts. Add a re-learn button if that bites.
struct LiftTop {
    bool found = false;
    double position = 0;
    std::uint32_t stalledSince = 0;

    int limit(int power, double raw, double rpm, std::uint32_t now) {
        if (power <= 0) {
            stalledSince = 0;
            return power;
        }
        if (found) {
            double left = position - raw;
            if (left <= LIFT_TOP_MARGIN) return 0;
            if (left < LIFT_SLOW_ZONE) return std::min(power, LIFT_SLOW_POWER);
            return power;
        }
        if (power < LIFT_TOP_STALL_POWER || std::fabs(rpm) > LIFT_STALL_RPM) {
            stalledSince = 0;
            return power;
        }
        if (!stalledSince) stalledSince = now;
        if (now - stalledSince >= LIFT_STALL_MS) {
            found = true;
            position = raw;
            return 0;  // stop grinding into the stop
        }
        return power;
    }
};

struct LiftControl {
    enum Mode { IDLE, MANUAL, TARGET, REZERO };
    Mode mode = IDLE;
    bool zeroed = false;
    bool backwards = false;
    double zero = 0;    // raw sensor reading at the bottom
    double target = 0;  // height being driven to
    int approach = 1;   // +1 going up to the target, -1 coming down to it
    int manualPower = 0;
    double stoppedShort = 0;  // how far short the last move ran out of time
    double startRaw = 0;
    std::uint32_t startTime = 0;
    bool downHeld = false;
    std::uint32_t downSince = 0;
    LiftTop top;

    double height(double raw) const { return raw - zero; }

    void setZero(double raw) {
        zero = raw;
        zeroed = true;
    }

    void manual(int power) {
        manualPower = power;
        if (power) {
            mode = MANUAL;
        } else if (mode == MANUAL) {
            mode = IDLE;
        }
    }

    void nextPreset(double raw, std::uint32_t now) {
        if (!zeroed || backwards) return;
        double from = height(raw);
        if (mode == TARGET && approach > 0) from = std::max(from, target);
        for (double preset : LIFT_PRESETS) {
            if (preset > from + LIFT_TOLERANCE) {
                driveTo(preset, raw, now);
                return;
            }
        }
    }

    bool down(double raw, std::uint32_t now) {
        if (!zeroed || backwards || height(raw) <= LIFT_BOTTOM) return false;
        driveTo(0, raw, now);
        return true;
    }

    void downButton(bool held, double raw, std::uint32_t now) {
        if (held && !downHeld) {
            downHeld = true;
            downSince = now;
            down(raw, now);
        } else if (held && mode == IDLE && now - downSince >= LIFT_REZERO_HOLD_MS) {
            begin(REZERO, raw, now);  // only once it's already down at the bottom limit
        } else if (!held && downHeld) {
            downHeld = false;
            if (mode == REZERO) {
                setZero(raw);
                mode = IDLE;
            }
        }
    }

    // Returns the power to send. While the robot is disabled the motors ignore
    // commands, so a pending move just waits.
    int update(double raw, double rpm, bool enabled, std::uint32_t now) {
        if (!enabled) {
            begin(mode, raw, now);
            return 0;
        }
        switch (mode) {
            case IDLE:
                return 0;
            case MANUAL:
                return bottomLimit(top.limit(manualPower, raw, rpm, now), raw);
            case REZERO:
                return -LIFT_REZERO_POWER;  // the driver decides where the bottom is
            case TARGET: {
                double error = target - height(raw);
                // Done once it reaches the target. The last stretch is at
                // LIFT_MIN_POWER, so it only coasts a degree or two past.
                if (error * approach <= 0) {
                    mode = IDLE;
                    return 0;
                }
                if (now - startTime > LIFT_MOVE_TIMEOUT_MS) {
                    stoppedShort = std::fabs(error);
                    mode = IDLE;
                    return 0;
                }
                double power = std::clamp(error * LIFT_KP, -127.0, 127.0);
                if (std::fabs(power) < LIFT_MIN_POWER) power = std::copysign(LIFT_MIN_POWER, error);
                if (std::fabs(power) < LIFT_WRONG_WAY_POWER) {
                    startRaw = raw;  // only judge direction while pushing hard
                } else if (wentWrongWay(raw, error > 0 ? 1 : -1)) {
                    return 0;
                }
                int out = bottomLimit(top.limit(static_cast<int>(power), raw, rpm, now), raw);
                if (out == 0) mode = IDLE;  // reached the top or the bottom limit first
                return out;
            }
        }
        return 0;
    }

  private:
    void driveTo(double height_, double raw, std::uint32_t now) {
        begin(TARGET, raw, now);
        target = height_;
        approach = target > height(raw) ? 1 : -1;
    }

    void begin(Mode m, double raw, std::uint32_t now) {
        mode = m;
        startRaw = raw;
        startTime = now;
    }

    // Never unwind the winch past the bottom; slow down on the way to it.
    int bottomLimit(int power, double raw) const {
        if (power >= 0 || !zeroed) return power;
        double h = height(raw);
        if (h <= LIFT_BOTTOM) return 0;
        if (h < LIFT_SLOW_ZONE) return std::max(power, -LIFT_SLOW_POWER);
        return power;
    }

    // If the sensor counts the opposite way to the motors, driving to a height
    // would run the lift into an end stop at full power. Stop and flag it.
    bool wentWrongWay(double raw, int direction) {
        if ((raw - startRaw) * direction > -LIFT_WRONG_WAY) return false;
        backwards = true;
        mode = IDLE;
        return true;
    }
};
