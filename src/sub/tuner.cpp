#include "tuner.hpp"
#include "config.hpp"
#include "controls.hpp"
#include "drivetrain.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <vector>

// Step 1, relay: bang the motors back and forth around where the robot is
// sitting and measure how big and how fast the wobble is (the ultimate gain Ku
// and period Tu). That gives a first guess at the gains.
static constexpr int TURN_POWER = 50;
static constexpr float TURN_HYSTERESIS = 0.5f;  // degrees
static constexpr int MOVE_POWER = 40;
static constexpr float MOVE_HYSTERESIS = 0.2f;  // inches
static constexpr int FLIPS = 12;                // relay flips to record
static constexpr int WARMUP = 4;                // ignore the first few while the wobble settles
static constexpr std::uint32_t TIMEOUT_MS = 8000;

// How the measurement becomes gains.
static constexpr float KP_FACTOR = 0.3f;    // fraction of Ku
static constexpr float TD_FACTOR = 0.125f;  // derivative time as a fraction of Tu
// LemLib's D term is (error - lastError) per 10 ms update, not per second.
static constexpr float LEMLIB_LOOP_S = 0.01f;

// Step 2, refine: drive real moves there and back with those gains and nudge
// them until the robot stops within the exact tolerance without overshooting.
// Short, medium and long moves each round, so the gains hold across what autons
// use. The long move needs that much clear space in front of the robot.
static constexpr float TEST_INCHES[] = {4, 10, 18};
static constexpr float TEST_DEGREES[] = {15, 60, 120};
static constexpr int REFINE_ROUNDS = 8;
// Aim inside the exact tolerance, so distances it didn't test land inside it too.
static constexpr float AIM = 0.7f;
static constexpr float OVERSHOOT_ALLOWED = 2;  // times the aimed tolerance
static constexpr float KI_START = 0.1f;        // fraction of kP, first time integral is needed

static constexpr int ABORT_STICK = 40;

std::atomic<TuneKind> tuning{TUNE_NONE};
std::atomic<float> calReading{0};
TuneResult turnTune;
TuneResult moveTune;
CalResult distCal;
CalResult turnCal;

static std::atomic<bool> stopRequested{false};
static std::atomic<bool> calConfirmed{false};

static bool shouldStop() {
    return stopRequested ||
           std::abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > ABORT_STICK ||
           std::abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) > ABORT_STICK ||
           std::abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > ABORT_STICK;
}

static bool claim(TuneKind kind) {
    TuneKind idle = TUNE_NONE;
    if (!tuning.compare_exchange_strong(idle, kind)) return false;
    stopRequested = false;
    return true;
}

void cancelTune() {
    stopRequested = true;
}

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

template <typename Measure, typename Drive>
static TuneResult relay(Measure measure, Drive drive, int power, float hysteresis) {
    float setpoint = measure();
    int output = power;
    std::vector<std::uint32_t> flips;
    float high = -1e9f, low = 1e9f;
    std::uint32_t start = pros::millis();
    const char* error = nullptr;

    while (static_cast<int>(flips.size()) < FLIPS) {
        if (shouldStop()) {
            error = "stopped";
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

// Searches kI up and down, taking smaller steps each time it changes direction
// so it closes in instead of bouncing between too much and too little.
struct Search {
    float step = 1.5f;
    int lastDirection = 0;

    float nudge(int direction) {
        if (lastDirection && direction != lastDirection) step = std::sqrt(step);
        lastDirection = direction;
        return direction > 0 ? step : 1 / step;
    }
};

// One round of test moves in, gains adjusted out. Returns true once the moves
// are already landing on target.
static bool adjust(TuneResult& r, Search& ki, float overshoot, float shortBy, float longBy,
                   bool timedOut, float tolerance) {
    r.miss = std::max(shortBy, longBy);
    if (overshoot > tolerance * OVERSHOOT_ALLOWED) {
        // Carrying too much speed into the target: more damping, a little less
        // push, and less integral in case that's winding up too.
        r.kP *= 0.95f;
        r.kD *= 1.2f;
        if (r.kI > 0) r.kI *= ki.nudge(-1);
    } else if (shortBy > tolerance) {
        // Stalling short: near the target, kP * error is less than it takes to
        // overcome friction. Only the integral keeps growing until it does, and
        // it only builds inside the windup range, so that has to reach the stall.
        r.windup = std::max(r.windup, shortBy * 1.5f);
        r.kI = r.kI > 0 ? r.kI * ki.nudge(+1) : r.kP * KI_START;
    } else if (longBy > tolerance) {
        // Settling just past the target.
        if (r.kI > 0) {
            r.kI *= ki.nudge(-1);
        } else {
            r.kP *= 0.9f;
        }
    } else if (timedOut) {
        // Landed, but still creeping in when time ran out: too sluggish.
        r.kP *= 1.15f;
    } else {
        return true;
    }
    return false;
}

// PID's gains are const, so the only way to change them live is to rebuild it.
// The integral resets whenever the error flips sign, so it can't wind up into
// an overshoot.
static void applyGains(lemlib::PID& pid, const TuneResult& r) {
    pid.~PID();
    new (&pid) lemlib::PID(r.kP, r.kI, r.kD, r.windup, true);
}

template <typename Measure, typename Move, std::size_t N>
static void refine(TuneResult& r, lemlib::PID& pid, Measure measure, Move move,
                   const float (&targets)[N], float tolerance, int timeout) {
    // Test the way autons run: holding at the end of each move, not coasting.
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    struct RestoreBrakes {
        ~RestoreBrakes() { applyDriverBrakes(); }
    } restore;

    Search ki;
    TuneResult best = r;
    best.miss = INFINITY;
    bool stopped = false;
    for (int round = 0; round < REFINE_ROUNDS && !stopped; round++) {
        float overshoot = 0, shortBy = 0, longBy = 0;
        bool timedOut = false;
        for (float target : targets) {
            for (int direction : {1, -1}) {
                float start = measure();
                std::uint32_t began = pros::millis();
                move(direction * target, timeout);
                float peak = 0;
                while (chassis.isInMotion() && !stopped) {
                    if (shouldStop()) {
                        chassis.cancelAllMotions();
                        stopped = true;
                    }
                    peak = std::max(peak, direction * (measure() - start));
                    pros::delay(10);
                }
                if (stopped) break;
                timedOut |= pros::millis() - began >= static_cast<std::uint32_t>(timeout) - 20;
                pros::delay(250);  // let it coast to a stop before measuring
                float travelled = direction * (measure() - start);
                overshoot = std::max(overshoot, std::max(peak, travelled) - target);
                shortBy = std::max(shortBy, target - travelled);
                longBy = std::max(longBy, travelled - target);
            }
            if (stopped) break;
        }
        if (stopped) break;

        TuneResult tried = r;
        tried.miss = std::max(shortBy, longBy);
        if (tried.miss < best.miss) best = tried;

        if (adjust(r, ki, overshoot, shortBy, longBy, timedOut, tolerance * AIM)) {
            r.exact = true;
            return;
        }
        applyGains(pid, r);
    }
    // Not fully on target: keep whichever gains landed closest, so a tune never
    // leaves the robot worse than a round it already tried.
    if (best.miss < INFINITY) r = best;
    applyGains(pid, r);
}

static void report(const char* name, const char* settings, const TuneResult& r) {
    if (!r.ok) {
        std::printf("%s tune failed: %s\n", name, r.error);
        return;
    }
    std::printf("%s tune: Ku %.2f  Tu %.3fs  worst miss %.2f  %s\n", name, r.ku, r.tu, r.miss,
                r.exact ? "on target" : "NOT on target yet, run it again");
    std::printf("  paste into drivetrain.cpp: %s(%.3f, %.3f, %.3f, %.2f, ...)\n", settings, r.kP, r.kI,
                r.kD, r.windup);
}

static float heading() {
    return chassis.getPose().theta;
}

void startTurnTune() {
    if (!claim(TUNE_TURN)) return;

    pros::Task task([] {
        // A second run picks up refining where the last one left off.
        if (!turnTune.ok) {
            turnTune = relay(heading,
                             [](int power) {
                                 leftMotors.move(power);
                                 rightMotors.move(-power);
                             },
                             TURN_POWER, TURN_HYSTERESIS);
        }
        if (turnTune.ok) {
            applyGains(chassis.angularPID, turnTune);
            refine(turnTune, chassis.angularPID, heading,
                   [](float degrees, int timeout) { turnDegrees(degrees, timeout, true); },
                   TEST_DEGREES, TURN_EXACT_DEG, TURN_TIMEOUT_MS);
            applyGains(chassis.angularPID, turnTune);
        }
        report("Turn", "angularController", turnTune);
        tuning = TUNE_NONE;
    });
}

void startMoveTune() {
    if (!claim(TUNE_MOVE)) return;

    pros::Task task([] {
        // Distance along the way the robot was facing when the tune started.
        lemlib::Pose start = chassis.getPose(true);
        auto distance = [start] {
            lemlib::Pose now = chassis.getPose(true);
            return static_cast<float>((now.x - start.x) * std::sin(start.theta) +
                                      (now.y - start.y) * std::cos(start.theta));
        };
        if (!moveTune.ok) {
            moveTune = relay(distance,
                             [](int power) {
                                 leftMotors.move(power);
                                 rightMotors.move(power);
                             },
                             MOVE_POWER, MOVE_HYSTERESIS);
        }
        if (moveTune.ok) {
            applyGains(chassis.lateralPID, moveTune);
            refine(moveTune, chassis.lateralPID, distance,
                   [](float inches, int timeout) { driveInches(inches, timeout, true); },
                   TEST_INCHES, DRIVE_EXACT_IN, DRIVE_TIMEOUT_MS);
            applyGains(chassis.lateralPID, moveTune);
        }
        report("Move", "lateralController", moveTune);
        tuning = TUNE_NONE;
    });
}

// Coasts the drive so it can be pushed by hand, shows the live reading, and
// hands the final reading to finish() once confirmed.
template <typename Measure, typename Finish>
static void calibrate(TuneKind kind, CalResult& result, Measure measure, Finish finish) {
    if (!claim(kind)) return;
    calConfirmed = false;
    result.done = false;

    pros::Task task([&result, measure, finish] {
        pros::MotorBrake brake = leftMotors.get_brake_mode();
        leftMotors.set_brake_mode_all(pros::MotorBrake::coast);
        rightMotors.set_brake_mode_all(pros::MotorBrake::coast);

        lemlib::Pose start = chassis.getPose();
        calReading = 0;
        while (!calConfirmed && !shouldStop()) {
            calReading = measure(start);
            pros::delay(20);
        }
        // Either direction works; only how far matters.
        float reading = std::fabs(measure(start));
        if (calConfirmed && reading > 1) {
            finish(reading);
            result.done = true;
        }

        leftMotors.set_brake_mode_all(brake);
        rightMotors.set_brake_mode_all(brake);
        tuning = TUNE_NONE;
    });
}

void startDistCal() {
    calibrate(TUNE_CAL_DIST, distCal,
              [](lemlib::Pose start) {
                  lemlib::Pose now = chassis.getPose();
                  return static_cast<float>(std::hypot(now.x - start.x, now.y - start.y));
              },
              [](float measured) {
                  // Odom scales distance by wheel diameter, so the true diameter is
                  // the configured one times how far off it read.
                  bool pod = VERTICAL_POD_PORT != 0;
                  float current = pod ? POD_WHEEL_DIAMETER : DRIVE_WHEEL_DIAMETER;
                  distCal.label = pod ? "POD" : "WHEEL";
                  distCal.value = current * CAL_INCHES / measured;
                  std::printf("Dist cal: odom read %.2f in for %.0f in\n", measured, CAL_INCHES);
                  std::printf("  paste into config.hpp: %s = %.4ff;\n",
                              pod ? "POD_WHEEL_DIAMETER" : "DRIVE_WHEEL_DIAMETER", distCal.value);
              });
}

void startTurnCal() {
    calibrate(TUNE_CAL_TURN, turnCal,
              [](lemlib::Pose start) { return chassis.getPose().theta - start.theta; },
              [](float measured) {
                  std::printf("Turn cal: odom read %.2f deg for 360\n", measured);
                  if (IMU_PORT) {
                      // The IMU measures heading itself; nothing in config scales it.
                      turnCal.label = "IMU%";
                      turnCal.value = (measured - 360) / 360 * 100;
                      std::printf("  IMU is off by %+.2f%%. Nothing to paste.\n", turnCal.value);
                  } else {
                      // Encoder heading is wheel difference / track width.
                      turnCal.label = "TRACK";
                      turnCal.value = TRACK_WIDTH * measured / 360;
                      std::printf("  paste into config.hpp: TRACK_WIDTH = %.4ff;\n", turnCal.value);
                  }
              });
}

void confirmCal() {
    calConfirmed = true;
}
