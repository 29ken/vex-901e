#pragma once
#include "api.h"
#include "config.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

extern pros::MotorGroup lift;

// Starts the lift's background task. Call once from initialize().
void initLift();

// Hand control from R1/R2, + is up. Anything but 0 cancels a preset move or homing.
void liftManual(int power);
// Goes to the next preset height above where the lift is, or is already headed.
void liftNextPreset();
// Goes all the way down and calls that height 0. This is the homing, and it
// re-zeros every time, so any drift is gone after each trip to the bottom.
void liftDown();
bool liftHomed();

struct LiftStatus {
    bool sensorOk;
    bool homed;
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
inline constexpr int LIFT_HOME_POWER = 40;        // gentle push into the bottom
inline constexpr std::uint32_t LIFT_MOVE_TIMEOUT_MS = 3000;
// Moved this far the wrong way while pushing at least this hard = sensor
// backwards. Low power doesn't count: a heavy load can sag while creeping.
inline constexpr double LIFT_WRONG_WAY = 20;
inline constexpr int LIFT_WRONG_WAY_POWER = 80;

// Stalled = pushing but barely turning, for LIFT_STALL_MS. Only near-full power
// counts at the top, so a heavy load hanging still mid-air isn't mistaken for it.
inline constexpr int LIFT_BOTTOM_STALL_POWER = 30;
inline constexpr int LIFT_TOP_STALL_POWER = 100;
inline constexpr double LIFT_STALL_RPM = 5;
inline constexpr std::uint32_t LIFT_STALL_MS = 250;

// Near either end, slow down instead of slamming into it.
inline constexpr double LIFT_SLOW_ZONE = 120;
inline constexpr double LIFT_TOP_MARGIN = 15;     // stop this far under the learned top
inline constexpr int LIFT_SLOW_POWER = 40;

// Everything below is the lift's logic, kept apart from the motors so
// test/lift_check.cpp can run it against a simulated lift.

struct StallTimer {
    std::uint32_t since = 0;

    bool stalled(int power, int minPower, double rpm, std::uint32_t now) {
        if (std::abs(power) < minPower || std::fabs(rpm) > LIFT_STALL_RPM) {
            since = 0;
            return false;
        }
        if (!since) since = now;
        return now - since >= LIFT_STALL_MS;
    }
};

// Learns where the top is from the first stall going up, then slows near it
// and stops just under it. Works in raw sensor degrees, so homing doesn't move it.
// ponytail: the first stall wins, so a snag on the way up becomes the top until
// the program restarts. Add a re-learn button if that bites.
struct LiftTop {
    bool found = false;
    double position = 0;
    StallTimer stall;

    int limit(int power, double raw, double rpm, std::uint32_t now) {
        if (power <= 0) {
            stall.since = 0;
            return power;
        }
        if (found) {
            double left = position - raw;
            if (left <= LIFT_TOP_MARGIN) return 0;
            if (left < LIFT_SLOW_ZONE) return std::min(power, LIFT_SLOW_POWER);
            return power;
        }
        if (stall.stalled(power, LIFT_TOP_STALL_POWER, rpm, now)) {
            found = true;
            position = raw;
            return 0;  // stop grinding into the stop
        }
        return power;
    }
};

struct LiftControl {
    enum Mode { IDLE, MANUAL, TARGET, HOMING };
    Mode mode = IDLE;
    bool homed = false;
    bool backwards = false;
    double zero = 0;    // raw sensor reading at the bottom
    double target = 0;  // height being driven to
    int approach = 1;   // +1 going up to the target, -1 coming down to it
    double stoppedShort = 0;  // how far short the last move ran out of time
    int manualPower = 0;
    double startRaw = 0;
    std::uint32_t startTime = 0;
    StallTimer bottom;
    LiftTop top;

    double height(double raw) const { return raw - zero; }

    void manual(int power) {
        manualPower = power;
        if (power) {
            mode = MANUAL;
        } else if (mode == MANUAL) {
            mode = IDLE;
        }
    }

    void nextPreset(double raw, std::uint32_t now) {
        if (!homed || backwards) return;
        double from = height(raw);
        if (mode == TARGET) from = std::max(from, target);
        for (double preset : LIFT_PRESETS) {
            if (preset > from + LIFT_TOLERANCE) {
                begin(TARGET, raw, now);
                target = preset;
                approach = target > height(raw) ? 1 : -1;
                return;
            }
        }
    }

    void down(double raw, std::uint32_t now) {
        if (!backwards) begin(HOMING, raw, now);
    }

    // Returns the power to send. While the robot is disabled the motors ignore
    // commands, so a pending move or homing just waits instead of learning
    // anything from the lift sitting still.
    int update(double raw, double rpm, bool enabled, std::uint32_t now) {
        if (!enabled) {
            begin(mode, raw, now);
            return 0;
        }
        switch (mode) {
            case IDLE:
                return 0;
            case MANUAL:
                return top.limit(manualPower, raw, rpm, now);
            case HOMING: {
                if (wentWrongWay(raw, -1)) return 0;
                int power = homed && height(raw) > LIFT_SLOW_ZONE ? -127 : -LIFT_HOME_POWER;
                if (bottom.stalled(power, LIFT_BOTTOM_STALL_POWER, rpm, now)) {
                    zero = raw;
                    homed = true;
                    mode = IDLE;
                    return 0;
                }
                return power;
            }
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
                int out = top.limit(static_cast<int>(power), raw, rpm, now);
                if (power > 0 && out == 0) mode = IDLE;  // the preset is above the top
                return out;
            }
        }
        return 0;
    }

  private:
    void begin(Mode m, double raw, std::uint32_t now) {
        mode = m;
        startRaw = raw;
        startTime = now;
        bottom.since = 0;
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
