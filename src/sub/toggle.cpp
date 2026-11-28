#include "toggle.hpp"
#include "runlog.hpp"
#include "main.h"
#include "config.hpp"
#include <atomic>
#include <cmath>
#include <cstdio>

// Made in initToggle(), and only for ports that are set (see toggle.hpp).
static pros::Motor* motor = nullptr;
static pros::Rotation* sensor = nullptr;
static pros::Optical* optical = nullptr;

static std::atomic<bool> flipping{false};
static pros::Mutex flipMutex;  // one flip at a time
static std::atomic<bool> byHand{false};  // R2 is turning the flapper (toggleManual())

void initToggle() {
#ifndef RW_SIM_PROS
    if (RATCHET_MOTOR) {
        motor = new pros::Motor(RATCHET_MOTOR);
        motor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);  // the lift drags it when it lowers
    }
    if (TOGGLE_SENSOR) sensor = new pros::Rotation(TOGGLE_SENSOR);
    if (TOGGLE_OPTICAL) {
        optical = new pros::Optical(TOGGLE_OPTICAL);
        optical->set_led_pwm(TOGGLE_LED_PERCENT);
    }
#endif
}

// Toggle shaft, degrees turned since power-on (keeps counting past 360).
static bool shaftTurned(double& degrees) {
    std::int32_t c = sensor->get_position();
    degrees = c / 100.0;
    return c != PROS_ERR;
}

// Toggle shaft, 0-360. This one is the same after a restart.
static bool shaftAngle(double& degrees) {
    std::int32_t c = sensor->get_angle();
    degrees = c / 100.0;
    return c != PROS_ERR;
}

static void stopFlapper() {
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    motor->move(0);
    pros::delay(80);
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

// Runs the ratchet motor in reverse (flapper only) to a rest angle at least
// minTravel degrees on. False if the motor isn't wired or the flapper didn't
// get there in time.
static bool turnFlapper(double minTravel) {
    if (!motor) {
        pros::delay(TOGGLE_FLIP_MS);
        return false;
    }
    flipping = true;
    motor->move(-TOGGLE_FLIP_POWER);
    bool ok = false;
    double start;
    if (!sensor || !shaftTurned(start)) {
        // No sensor: by time.
        pros::delay(TOGGLE_FLIP_MS);
        ok = true;
    } else {
        double goal = -1;  // degrees to turn in all; set once the shaft shows which way it counts
        std::uint32_t began = pros::millis();
        while (pros::millis() - began < static_cast<std::uint32_t>(TOGGLE_FLIP_TIMEOUT_MS)) {
            double now, angle;
            if (!shaftTurned(now) || !shaftAngle(angle)) break;
            double moved = now - start;
            if (goal < 0 && std::fabs(moved) >= 5) {
                double way = moved > 0 ? 1 : -1;
                double past = std::fmod(way * (angle - TOGGLE_REST_DEG), TOGGLE_FLAP_SPACING);
                if (past < 0) past += TOGGLE_FLAP_SPACING;       // how far past the last rest angle
                goal = std::fabs(moved) + (TOGGLE_FLAP_SPACING - past);
                while (goal < minTravel - TOGGLE_PARK_TOLERANCE) goal += TOGGLE_FLAP_SPACING;
            }
            if (goal >= 0 && std::fabs(moved) >= goal - TOGGLE_STOP_EARLY) {
                ok = true;
                break;
            }
            pros::delay(5);
        }
        if (!ok) say("Toggle: the flapper didn't get round in %d ms (jammed, or the sensor isn't turning)\n", TOGGLE_FLIP_TIMEOUT_MS);
    }
    stopFlapper();
    flipping = false;
    return ok;
}

const char* toggleColorName(ToggleColor c) {
    switch (c) {
        case ToggleColor::Red: return "red";
        case ToggleColor::Blue: return "blue";
        case ToggleColor::Yellow: return "yellow";
        default: return "?";
    }
}

ToggleColor toggleColor() {
    if (!optical) return ToggleColor::Unknown;
    if (optical->get_saturation() < TOGGLE_MIN_SATURATION) return ToggleColor::Unknown;
    double hue = optical->get_hue();  // 0-360: red 0, yellow 60, blue 240
    if (hue < 25 || hue > 335) return ToggleColor::Red;
    if (hue > 35 && hue < 80) return ToggleColor::Yellow;
    if (hue > 190 && hue < 265) return ToggleColor::Blue;
    return ToggleColor::Unknown;
}

bool setToggle(bool blueAlliance) {
    flipMutex.take();
    ToggleColor ours = blueAlliance ? ToggleColor::Blue : ToggleColor::Red;
    bool done = false;
    if (!motor) {
        say("Toggle: ratchet motor not wired yet, waiting %d ms instead (%s)\n", TOGGLE_FLIP_MS,
                    toggleColorName(ours));
        pros::delay(TOGGLE_FLIP_MS);
    } else {
        for (int flip = 1; flip <= TOGGLE_MAX_FLIPS && !done; flip++) {
            if (!turnFlapper(TOGGLE_FLAP_SPACING)) break;
            pros::delay(TOGGLE_SETTLE_MS);
            ToggleColor seen = toggleColor();
            if (seen == ToggleColor::Unknown) {
                say("Toggle: flipped; can't see a colour to check\n");
                done = true;
            } else if (seen == ours) {
                say("Toggle: %s after %d flip%s\n", toggleColorName(seen), flip, flip > 1 ? "s" : "");
                done = true;
            } else {
                say("Toggle: still %s after flip %d\n", toggleColorName(seen), flip);
            }
        }
    }
    flipMutex.give();
    return done;
}

static bool asyncBlue = false;
static void asyncFlip(void*) {
    setToggle(asyncBlue);
}

void toggleManual(bool held) {
    if (!motor || flipping) return;
    if (held) {
        byHand = true;
        motor->move(-TOGGLE_FLIP_POWER);
    } else if (byHand) {
        motor->move(0);  // coasts, like the rest of the time: the lift drags it when it lowers
        byHand = false;
    }
}

void setToggleAsync(bool blueAlliance) {
    if (flipping || byHand) return;
    asyncBlue = blueAlliance;
    pros::Task task(asyncFlip, nullptr, "Toggle flip");
}

void parkToggle() {
    if (!motor || !sensor) return;
    double angle;
    if (!shaftAngle(angle)) return;
    double off = std::fabs(std::remainder(angle - TOGGLE_REST_DEG, TOGGLE_FLAP_SPACING));
    if (off <= TOGGLE_PARK_TOLERANCE) return;  // already parked
    flipMutex.take();
    turnFlapper(0);
    flipMutex.give();
}

void ratchetWithLift(int liftPower) {
    if (!motor || flipping || byHand) return;
    // Coasting at 0 lets the lift drag the axle when it lowers.
    motor->move(RATCHET_ASSIST && liftPower > 0 ? liftPower : 0);
}

ToggleStatus toggleStatus() {
    ToggleStatus s{motor != nullptr, sensor != nullptr, optical != nullptr, 0, ToggleColor::Unknown, flipping.load()};
    if (sensor) shaftAngle(s.angle);
    s.color = toggleColor();
    return s;
}
