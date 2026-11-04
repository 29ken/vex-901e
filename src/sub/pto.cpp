#include "pto.hpp"
#include <cstdio>
#include <mutex>

// Red, like the lift motors: the PTO compares their speeds, so the readings have
// to be in the same units.
static pros::Motor motor(INTAKE, pros::MotorGearset::red);
static pros::adi::DigitalOut* piston = nullptr;  // made in initPto()

// The driver loop and autons send requests; the lift task runs the PTO.
static pros::Mutex mutex;
static PtoControl control;
static int intakePower = 0;

// Only the lift task touches these.
static PtoMode sentPiston = PtoMode::INTAKE;
static pros::motor_brake_mode_e_t sentBrake = pros::E_MOTOR_BRAKE_COAST;
static int saidMeshes = 0, saidSlips = 0, saidGaveUp = 0;
static bool saidNoMesh = false;

static bool pistonValue(PtoMode mode) {
    return (mode == PtoMode::LIFT) == PTO_PISTON_LIFT;
}

void initPto() {
    static pros::adi::DigitalOut out(PTO_PISTON, pistonValue(PtoMode::INTAKE));
    piston = &out;
    motor.set_brake_mode(sentBrake);
}

bool ptoStep(const PtoLiftTick& tick) {
    if (!piston) return false;  // initPto() hasn't run
    double ptoRpm = motor.get_actual_velocity();
    PtoOutput out;
    {
        std::lock_guard<pros::Mutex> hold(mutex);
        PtoInput in;
        in.now = pros::millis();
        in.enabled = !pros::competition::is_disabled();
        if (!in.enabled) intakePower = 0;  // a button held at the buzzer doesn't carry over
        in.liftPower = tick.power;
        in.liftHeld = tick.held;
        in.liftAsk = tick.ask;
        in.liftKnown = tick.heightKnown;
        in.liftHeight = tick.height;
        in.intakePower = intakePower;
        in.liftRpm = tick.rpm;
        in.ptoRpm = ptoRpm;
        out = control.update(in);

        if (control.meshes != saidMeshes) {
            std::printf("PTO meshed with the lift; intake motor power %+d raises it\n", control.dir);
            saidMeshes = control.meshes;
        }
        if (control.slips != saidSlips) {
            std::printf("PTO fell out of step with the lift; the intake motor stopped pushing "
                        "until it turns with the lift again\n");
            saidSlips = control.slips;
        }
        if (control.noMesh && !saidNoMesh) {
            std::printf("PTO in LIFT but the intake motor isn't turning with the lift: check the "
                        "air, PTO_PISTON, and that it has the same red cartridge as the lift motors\n");
        }
        saidNoMesh = control.noMesh;
        if (control.gaveUpLowering != saidGaveUp) {
            std::printf("PTO couldn't get the cascade down in %.1f s; shifted to INTAKE anyway\n",
                        PTO_LOWER_TIMEOUT_MS / 1000.0);
            saidGaveUp = control.gaveUpLowering;
        }
    }

    if (out.piston != sentPiston) {
        piston->set_value(pistonValue(out.piston));
        sentPiston = out.piston;
    }
    if (out.brake != sentBrake) {
        motor.set_brake_mode(out.brake);
        sentBrake = out.brake;
    }
    motor.move(out.power);
    return out.lowerLift;
}

void ptoShift(PtoMode mode) {
    std::lock_guard<pros::Mutex> hold(mutex);
    control.shift(mode);
}

void ptoToggle() {
    std::lock_guard<pros::Mutex> hold(mutex);
    control.shift(control.target == PtoMode::LIFT ? PtoMode::INTAKE : PtoMode::LIFT);
}

void intakeMove(int power) {
    std::lock_guard<pros::Mutex> hold(mutex);
    intakePower = power;
}

void ptoWaitUntilShifted(std::uint32_t timeoutMs) {
    // A shift asked for with intakeMove() or liftManual() starts on the lift
    // task's next tick, so give it one before checking.
    pros::delay(20);
    std::uint32_t start = pros::millis();
    while (pros::millis() - start < timeoutMs) {
        PtoStatus s = ptoStatus();
        if (!s.lowering && !s.shifting) return;
        pros::delay(10);
    }
}

bool ptoHoldingLift() {
    std::lock_guard<pros::Mutex> hold(mutex);
    return !control.liftFree();
}

PtoStatus ptoStatus() {
    std::lock_guard<pros::Mutex> hold(mutex);
    bool onWinch = control.liftFree();
    bool lowering = control.phase == PtoControl::LOWERING;
    bool shifting = control.phase == PtoControl::STOPPING || control.phase == PtoControl::SHIFTING;
    return {control.target, lowering, shifting, onWinch && control.pushing, onWinch && control.noMesh};
}
