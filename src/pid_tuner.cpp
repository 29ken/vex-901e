#include "pid_tuner.hpp"
#include "controls.hpp"
#include "drivetrain.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <vector>

// Relay auto-tune: bang the motors back and forth around where the robot is
// sitting, measure how big and how fast the wobble is (the ultimate gain Ku and
// period Tu), and turn that into PD gains.

static constexpr int TURN_POWER = 50;
static constexpr float TURN_HYSTERESIS = 0.5f;  // degrees
static constexpr int MOVE_POWER = 40;
static constexpr float MOVE_HYSTERESIS = 0.2f;  // inches

static constexpr int FLIPS = 12;  // relay flips to record
static constexpr int WARMUP = 4;  // ignore the first few while the wobble settles
static constexpr std::uint32_t TIMEOUT_MS = 8000;
static constexpr int ABORT_STICK = 40;

// How the measurement becomes gains. Overshooting? Lower KP_FACTOR.
// Sluggish to settle? Raise TD_FACTOR.
static constexpr float KP_FACTOR = 0.3f;    // fraction of Ku
static constexpr float TD_FACTOR = 0.125f;  // derivative time as a fraction of Tu
// LemLib's D term is (error - lastError) per 10 ms update, not per second.
static constexpr float LEMLIB_LOOP_S = 0.01f;

std::atomic<TuneKind> tuning{TUNE_NONE};
TuneResult turnTune;
TuneResult moveTune;

static TuneResult gainsFrom(int power, float hysteresis, float amplitude, float period) {
    TuneResult r;
    r.done = true;
    if (amplitude <= hysteresis * 1.1f || period <= 0.02f) {
        r.error = "no swing";
        return r;
    }
    r.ku = 4 * power / (M_PI * std::sqrt(amplitude * amplitude - hysteresis * hysteresis));
    r.tu = period;
    r.kP = KP_FACTOR * r.ku;
    r.kD = r.kP * TD_FACTOR * r.tu / LEMLIB_LOOP_S;
    r.ok = true;
    return r;
}

static bool sticksMoved() {
    return std::abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > ABORT_STICK ||
           std::abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) > ABORT_STICK ||
           std::abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > ABORT_STICK;
}

template <typename Measure, typename Drive>
static TuneResult relay(Measure measure, Drive drive, int power, float hysteresis) {
    float setpoint = measure();
    int output = power;
    std::vector<std::uint32_t> flips;
    float high = -1e9f, low = 1e9f;
    std::uint32_t start = pros::millis();
    const char* error = nullptr;

    while (static_cast<int>(flips.size()) < FLIPS) {
        if (sticksMoved()) {
            error = "aborted";
            break;
        }
        if (pros::millis() - start > TIMEOUT_MS) {
            error = "timed out";
            break;
        }

        float value = measure();
        float offset = setpoint - value;
        if (output > 0 && offset < -hysteresis) {
            output = -power;
            flips.push_back(pros::millis());
        } else if (output < 0 && offset > hysteresis) {
            output = power;
            flips.push_back(pros::millis());
        }
        if (static_cast<int>(flips.size()) > WARMUP) {
            high = std::max(high, value);
            low = std::min(low, value);
        }

        drive(output);
        pros::delay(10);
    }
    drive(0);

    if (error) {
        TuneResult r;
        r.done = true;
        r.error = error;
        return r;
    }
    // Two flips per full period.
    float halfPeriods = FLIPS - 1 - WARMUP;
    float period = (flips.back() - flips[WARMUP]) / 1000.0f / (halfPeriods / 2);
    return gainsFrom(power, hysteresis, (high - low) / 2, period);
}

// PID's gains are const, so the only way to change them live is to rebuild it.
// kI stays 0 and windup matches drivetrain.cpp.
static void applyGains(lemlib::PID& pid, const TuneResult& r) {
    pid.~PID();
    new (&pid) lemlib::PID(r.kP, 0, r.kD, 3, true);
}

static void report(const char* name, const char* settings, const TuneResult& r) {
    if (!r.ok) {
        std::printf("%s tune failed: %s\n", name, r.error);
        return;
    }
    std::printf("%s tune: Ku %.2f  Tu %.3fs\n", name, r.ku, r.tu);
    std::printf("  paste into drivetrain.cpp: %s kP %.2f, kD %.2f\n", settings, r.kP, r.kD);
}

void startTurnTune() {
    TuneKind idle = TUNE_NONE;
    if (!tuning.compare_exchange_strong(idle, TUNE_TURN)) return;

    pros::Task task([] {
        turnTune = relay([] { return chassis.getPose().theta; },
                         [](int power) {
                             leftMotors.move(power);
                             rightMotors.move(-power);
                         },
                         TURN_POWER, TURN_HYSTERESIS);
        if (turnTune.ok) applyGains(chassis.angularPID, turnTune);
        report("Turn", "angularController", turnTune);
        tuning = TUNE_NONE;
    });
}

void startMoveTune() {
    TuneKind idle = TUNE_NONE;
    if (!tuning.compare_exchange_strong(idle, TUNE_MOVE)) return;

    pros::Task task([] {
        // Distance along the way the robot was facing when the tune started.
        lemlib::Pose start = chassis.getPose(true);
        auto distance = [start] {
            lemlib::Pose now = chassis.getPose(true);
            return (now.x - start.x) * std::sin(start.theta) + (now.y - start.y) * std::cos(start.theta);
        };
        moveTune = relay(distance,
                         [](int power) {
                             leftMotors.move(power);
                             rightMotors.move(power);
                         },
                         MOVE_POWER, MOVE_HYSTERESIS);
        if (moveTune.ok) applyGains(chassis.lateralPID, moveTune);
        report("Move", "lateralController", moveTune);
        tuning = TUNE_NONE;
    });
}
