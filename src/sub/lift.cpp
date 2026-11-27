#include "lift.hpp"
#include "runlog.hpp"
#include "toggle.hpp"
#include <cstdio>
#include <mutex>

// Both motors drive the winch shaft directly (V4 CAD). Set the gearset to the
// cartridges really in them (red = 100 rpm): the top and stall checks use rpm.
pros::MotorGroup lift({LIFT_LEFT, LIFT_RIGHT}, pros::MotorGearset::red);
static pros::Rotation sensor(LIFT_SENSOR);

// The driver loop sends commands while the lift task runs the control.
static pros::Mutex mutex;
static LiftControl control;

static bool readSensor(double& raw) {
#ifdef RW_SIM_PROS
    // The simulator only simulates rotation sensors used as tracking wheels. The
    // motors turn the winch shaft directly, so their position stands in for the
    // sensor there. It has no load on it, so sim lift moves run a bit fast.
    raw = lift.get_position(0);
    return true;
#else
    std::int32_t centidegrees = sensor.get_position();
    raw = centidegrees / 100.0;
    return centidegrees != PROS_ERR;
#endif
}

// The lift motors' speed, + is up. They're geared together, so either one's
// reading will do; this keeps working with one of them unplugged.
static double liftRpm() {
    for (double rpm : lift.get_actual_velocity_all()) {
        if (std::isfinite(rpm)) return rpm;
    }
    return PROS_ERR_F;
}

static void loop() {
    bool saidTop = false, saidBackwards = false, saidMissing = false;
    double lastZero = control.zero;
    while (true) {
        double raw;
        bool ok = readSensor(raw);
        bool disabled = pros::competition::is_disabled();
        double rpm = liftRpm();
        int power;
        {
            std::lock_guard<pros::Mutex> hold(mutex);
            // R1/R2 held at the buzzer doesn't carry into the next period. Preset
            // moves still wait out the disable.
            if (disabled) control.manual(0);
            if (ok) {
                power = control.update(raw, rpm, !disabled, pros::millis());
            } else {
                // No sensor: presets and the bottom limit can't work, but hand control still does.
                if (control.mode != LiftControl::MANUAL) control.mode = LiftControl::IDLE;
                power = control.mode == LiftControl::MANUAL ? control.manualPower : 0;
            }

            if (control.top.found && !saidTop) {
                say("Lift top found at %.0f\n", control.height(control.top.position));
                saidTop = true;
            }
            if (control.zeroed && control.zero != lastZero) {
                say("Lift re-zeroed\n");
                lastZero = control.zero;
            }
            if (control.backwards && !saidBackwards) {
                say("Lift sensor counts backwards; set LIFT_SENSOR to %d\n", -LIFT_SENSOR);
                saidBackwards = true;
            }
            if (control.stoppedShort > 0) {
                say("Lift stopped %.0f short of a preset: either that preset is above the "
                            "top, or the lift needs a higher LIFT_MIN_POWER\n",
                            control.stoppedShort);
                control.stoppedShort = 0;
            }
            if (!ok && !saidMissing) {
                say("Lift sensor on port %d not responding\n", std::abs(LIFT_SENSOR));
                saidMissing = true;
            }
        }
        lift.move(power);
        ratchetWithLift(power);  // the ratchet motor shares the winch (toggle.hpp)
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
        say("Lift sensor on port %d not found at startup; restart with the lift down\n",
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

void liftTo(double height) {
    double raw;
    if (!readSensor(raw)) return;
    std::lock_guard<pros::Mutex> hold(mutex);
    control.goTo(height, raw, pros::millis());
}

bool liftMoving() {
    std::lock_guard<pros::Mutex> hold(mutex);
    return control.mode == LiftControl::TARGET;
}

bool liftWait(int timeout_ms) {
    std::uint32_t start = pros::millis();
    while (liftMoving() && pros::millis() - start < static_cast<std::uint32_t>(timeout_ms)) {
        pros::delay(10);
    }
    double raw;
    if (!readSensor(raw)) return false;
    std::lock_guard<pros::Mutex> hold(mutex);
    if (control.mode == LiftControl::TARGET) return false;  // still going: timed out
    // The bottom limit stops it LIFT_BOTTOM above 0, which counts as there.
    return control.zeroed && std::fabs(control.target - control.height(raw)) <= LIFT_TOLERANCE;
}

LiftStatus liftStatus() {
    double raw;
    bool ok = readSensor(raw);
    std::lock_guard<pros::Mutex> hold(mutex);
    return {ok, control.zeroed, control.backwards, control.height(raw)};
}
