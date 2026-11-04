#include "lift.hpp"
#include <cstdio>
#include <mutex>

pros::MotorGroup lift({LIFT_LEFT, LIFT_RIGHT});
static pros::Rotation sensor(LIFT_SENSOR);

// The driver loop sends commands while the lift task runs the control.
static pros::Mutex mutex;
static LiftControl control;

static bool readSensor(double& raw) {
    std::int32_t centidegrees = sensor.get_position();
    raw = centidegrees / 100.0;
    return centidegrees != PROS_ERR;
}

static void loop() {
    bool saidTop = false, saidBackwards = false, saidMissing = false;
    double lastZero = control.zero;
    while (true) {
        double raw;
        bool ok = readSensor(raw);
        int power;
        {
            std::lock_guard<pros::Mutex> hold(mutex);
            if (ok) {
                power = control.update(raw, lift.get_actual_velocity(),
                                       !pros::competition::is_disabled(), pros::millis());
            } else {
                // No sensor: presets and the bottom limit can't work, but hand control still does.
                if (control.mode != LiftControl::MANUAL) control.mode = LiftControl::IDLE;
                power = control.mode == LiftControl::MANUAL ? control.manualPower : 0;
            }

            if (control.top.found && !saidTop) {
                std::printf("Lift top found at %.0f\n", control.height(control.top.position));
                saidTop = true;
            }
            if (control.zeroed && control.zero != lastZero) {
                std::printf("Lift re-zeroed\n");
                lastZero = control.zero;
            }
            if (control.backwards && !saidBackwards) {
                std::printf("Lift sensor counts backwards; set LIFT_SENSOR to %d\n", -LIFT_SENSOR);
                saidBackwards = true;
            }
            if (control.stoppedShort > 0) {
                std::printf("Lift stopped %.0f short of a preset: either that preset is above the "
                            "top, or the lift needs a higher LIFT_MIN_POWER\n",
                            control.stoppedShort);
                control.stoppedShort = 0;
            }
            if (!ok && !saidMissing) {
                std::printf("Lift sensor on port %d not responding\n", std::abs(LIFT_SENSOR));
                saidMissing = true;
            }
        }
        lift.move(power);
        pros::delay(10);
    }
}

void initLift() {
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    // Matches start with the lift down, so wherever it is now is the bottom.
    double raw;
    if (readSensor(raw)) {
        control.setZero(raw);
    } else {
        std::printf("Lift sensor on port %d not found at startup; hold L2 to zero the lift\n",
                    std::abs(LIFT_SENSOR));
    }
    pros::Task task(loop, "Lift");
}

void liftManual(int power) {
    std::lock_guard<pros::Mutex> hold(mutex);
    control.manual(power);
}

void liftNextPreset() {
    double raw;
    if (!readSensor(raw)) return;
    std::lock_guard<pros::Mutex> hold(mutex);
    control.nextPreset(raw, pros::millis());
}

void liftDown() {
    double raw;
    if (!readSensor(raw)) return;
    std::lock_guard<pros::Mutex> hold(mutex);
    control.down(raw, pros::millis());
}

void liftDownButton(bool held) {
    double raw;
    if (!readSensor(raw)) return;
    std::lock_guard<pros::Mutex> hold(mutex);
    control.downButton(held, raw, pros::millis());
}

LiftStatus liftStatus() {
    double raw;
    bool ok = readSensor(raw);
    std::lock_guard<pros::Mutex> hold(mutex);
    return {ok, control.zeroed, control.backwards, control.height(raw)};
}
