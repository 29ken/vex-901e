#pragma once
#include "api.h"
#include "config.hpp"
#include "lift.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

// The PTO lends the intake motor to the lift. One piston slides the motor's
// 36T gear between two jobs:
//   INTAKE (piston retracted): the gear is on the intake's 36T, so B and Y run
//                              the intake. The cascade stays all the way down.
//   LIFT   (piston extended):  the gear is on the winch's 12T, so the motor is a
//                              third lift motor. R1/R2 and every limit in
//                              lift.hpp drive all three.
// Both lift motors and this one reach the winch at 300 rpm (100 rpm x 36/12), so
// in LIFT it just gets the same power they do. That only holds while all three
// have the same red 100 rpm cartridge.
//
// It follows the driver. A fresh R1/R2 press shifts to LIFT, then the cascade
// moves. A fresh B/Y press shifts to INTAKE, then the intake runs; if the cascade
// is up, it first goes all the way down (on all three motors), then shifts.
// Whichever was pressed last wins. R2 on the way down cancels the trip.
// DOWN shifts too, so the driver can get into LIFT on the way to the goal and
// skip the pause.
//
// A shift holds the cascade still and stops the motor, fires the piston, then
// turns the gear slowly so it drops into the teeth: about 0.35 s.
//
// In LIFT the motor doesn't push until it's been seen turning with the lift. That
// also shows which way it has to push, so there is no direction to set, and it
// never pushes against the lift motors.
//
// The lift task runs the PTO (ptoStep), so both get their power in the same 10 ms tick.
//
// First time on the robot, with it up on a stand:
//   1. Tap R2. The gear should slide onto the winch. If it slides the other way,
//      flip PTO_PISTON_LIFT in config.hpp.
//   2. Hold R2 for a moment. The terminal prints "PTO meshed with the lift" and
//      from then on the intake motor lifts too. If the controller shows NOMESH
//      instead, check the air and PTO_PISTON in config.hpp.
//   3. Press B. The cascade comes down, the gear slides back, the intake runs.
//   4. Lower PTO_SHIFT_MS until a shift now and then ends in NOMESH, then add 50.

enum class PtoMode { INTAKE, LIFT };

// Sets the piston to INTAKE. Call once from initialize(), before initLift().
void initPto();
// Shifts to that mode, unless it's already there or on the way. INTAKE takes
// the cascade down first.
void ptoShift(PtoMode mode);
// INTAKE <-> LIFT.
void ptoToggle();
// Runs the intake, -127..127, + pulls in. Only does anything in INTAKE; a fresh
// press in LIFT shifts to INTAKE first. Resets to 0 whenever the robot is disabled.
void intakeMove(int power);
// For autons: waits for a trip down and shift in progress to finish.
void ptoWaitUntilShifted(std::uint32_t timeoutMs = 3000);

// What the lift task tells the PTO each tick.
struct PtoLiftTick {
    int power = 0;             // what the lift motors were just sent
    bool held = false;         // ptoHoldingLift() at the start of the tick
    int ask = 0;               // LiftControl::ask(): +1 up, -1 down, 0 nothing
    bool heightKnown = false;  // the lift sensor works and is zeroed
    double height = 0;         // lift sensor degrees above the bottom
    double rpm = 0;            // lift motor speed, + is up (PROS_ERR_F if unreadable)
};
// The lift task calls these every loop.
bool ptoHoldingLift();  // true unless the gear is on the winch: send the lift 0
// Runs the PTO for this tick. Returns true when it wants the cascade taken to the
// bottom (LiftControl::lowerForPto) before shifting to INTAKE.
bool ptoStep(const PtoLiftTick& tick);

struct PtoStatus {
    PtoMode mode;   // where it is, or where it's going
    bool lowering;  // taking the cascade down before shifting to INTAKE
    bool shifting;
    bool pushing;   // LIFT: the intake motor is turning with the lift and helping
    bool noMesh;    // LIFT: the lift keeps moving but the intake motor isn't turning with it
};
PtoStatus ptoStatus();

// Tuning. Speeds are rpm at the red cartridge's output, as get_actual_velocity()
// reads them (lift.cpp and pto.cpp set all three motors to red).

// true: B/Y shift to INTAKE and R1/R2 to LIFT by themselves.
// false: only DOWN shifts, and R1/R2 do nothing in INTAKE.
inline constexpr bool PTO_FOLLOW_DRIVER = true;

// The cascade counts as all the way down within this many lift sensor degrees of
// LIFT_BOTTOM, where the lift itself stops going down.
inline constexpr double PTO_DOWN_SLACK = 5;
// If it coasts up while stopping to shift, it goes back down first, but only
// once it's this much further up, so one resting right at the edge doesn't
// bounce back and forth. It never shifts to INTAKE higher than
// LIFT_BOTTOM + PTO_DOWN_SLACK + PTO_DOWN_HYSTERESIS.
inline constexpr double PTO_DOWN_HYSTERESIS = 3;
// If a trip down hasn't got there in this long (stuck, or the lift sensor is
// lying), it shifts to INTAKE anyway so the intake still works. Time with R1/R2
// held doesn't count: the driver is in charge of the cascade then.
inline constexpr std::uint32_t PTO_LOWER_TIMEOUT_MS = 3000;
// The lift sensor has to be silent this long before the height counts as
// unknown; until then the last good reading stands. (Unknown: shifts to INTAKE
// without a trip down, so a dead sensor doesn't leave the intake dead too.)
inline constexpr std::uint32_t PTO_SENSOR_LOST_MS = 200;

// Before firing the piston, the lift has to have been held still this long...
inline constexpr std::uint32_t PTO_SETTLE_MS = 30;
// ...and both the intake motor and the lift turning slower than this, so the
// gear never slides into moving teeth...
inline constexpr double PTO_STILL_RPM = 10;
// ...unless this long has passed since the shift began (a speed reading that
// never settles).
inline constexpr std::uint32_t PTO_STOP_TIMEOUT_MS = 250;
// From firing the piston until the gear counts as in. Covers the piston's travel
// plus finding the teeth.
inline constexpr std::uint32_t PTO_SHIFT_MS = 300;
// Power the gear turns at during a shift, so it drops into the teeth instead of
// sitting tooth-on-tooth. About 15 rpm, one tooth per 0.1 s; far too weak to move
// the lift, which is held still.
inline constexpr int PTO_JOG_POWER = 20;

// In LIFT, the intake motor counts as turning with the lift when its speed is
// within this fraction of the lift's speed of it, give or take PTO_STEP_RPM.
// Meshed, the two are locked together, so this can be tight.
inline constexpr double PTO_STEP_FRACTION = 0.2;
inline constexpr double PTO_STEP_RPM = 5;
// It starts pushing once that's held this long with the lift going at least this
// fast. Well above the ~15 rpm a loose gear can be left spinning at after a shift.
inline constexpr std::uint32_t PTO_MESH_MS = 50;
inline constexpr double PTO_MESH_RPM = 40;
// Pushing but out of step this long: the gear slipped out. It stops pushing and
// waits until it's turning with the lift again. (A loose gear pushed at the lift's
// power spins at about the lift's speed, so this mostly shows up when the lift
// stops at the top and the gear doesn't.)
inline constexpr std::uint32_t PTO_SLIP_MS = 100;
// The lift moving this long without the motor joining in: NOMESH on the controller.
inline constexpr std::uint32_t PTO_NO_MESH_MS = 500;

// Fast enough that "turning with the lift" can only mean one direction.
static_assert(PTO_MESH_RPM * (1 - PTO_STEP_FRACTION) > PTO_STEP_RPM);

// Everything below is the PTO's logic, kept apart from the motors so
// test/pto_check.cpp can run it, with the real LiftControl, against a simulated PTO.

// How long something has been true, in ms. 0 while it isn't.
struct PtoTimer {
    bool on = false;
    std::uint32_t since = 0;

    std::uint32_t held(bool isTrue, std::uint32_t now) {
        if (!isTrue) {
            on = false;
            return 0;
        }
        if (!on) {
            on = true;
            since = now;
        }
        return now - since;
    }
};

struct PtoInput {
    std::uint32_t now = 0;
    bool enabled = true;
    int liftPower = 0;         // what the lift motors were just sent, + is up
    bool liftHeld = false;     // the lift task held the lift still for the PTO this tick
    int liftAsk = 0;           // what the driver or an auton wants from the lift: +1 up, -1 down
    bool liftKnown = false;    // the lift sensor works, so liftHeight means something
    double liftHeight = 0;     // lift sensor degrees above the bottom
    int intakePower = 0;       // what's wanted from the intake, + is in
    double liftRpm = 0;        // lift motor speed, + is up
    double ptoRpm = 0;         // intake motor speed
};

struct PtoOutput {
    PtoMode piston;                     // the job the piston is set for
    int power;                          // intake motor, -127..127
    pros::motor_brake_mode_e_t brake;   // intake motor
    bool lowerLift;                     // take the cascade to the bottom
};

struct PtoControl {
    enum Phase { READY, LOWERING, STOPPING, SHIFTING };
    PtoMode target = PtoMode::INTAKE;  // the mode it's in, or going to
    PtoMode piston = PtoMode::INTAKE;  // what the piston was last told
    Phase phase = READY;
    bool settled = true;               // the gear finished moving where the piston sent it

    // LIFT
    bool pushing = false;  // turning with the lift, so it pushes with it
    int dir = 0;           // the intake motor power sign that raises the lift
    bool noMesh = false;
    int meshes = 0;        // times it started pushing, for the terminal
    int slips = 0;         // times it stopped pushing because it fell out of step
    int gaveUpLowering = 0;  // times it shifted to INTAKE without the cascade getting down
    int candidate = 0;     // the way it seems to be turning, before it's sure

    int lastIntakePower = 0;
    int lastLiftAsk = 0;
    double lastHeight = 0;     // the last good lift sensor reading; matches start down
    bool forceShift = false;   // the trip down gave up: shift to INTAKE wherever the cascade is

    // The lift may only move while the gear is on the winch and staying there.
    bool liftFree() const {
        return phase == LOWERING || (phase == READY && target == PtoMode::LIFT);
    }

    // Shifting has nothing to do with clocks here: a phase starts counting on the
    // first update() that sees it, so requests from other tasks can't skew it.
    void shift(PtoMode to) {
        if (to == target) return;
        target = to;
        forceShift = false;
        if (phase == LOWERING) {
            phase = READY;  // was on the way to INTAKE; the gear never left the winch
            clearTimers();
            return;
        }
        if (phase == STOPPING && settled && piston == to) {
            phase = READY;  // changed its mind before the piston moved: nothing to undo
            clearTimers();
            return;
        }
        // Off the winch: the cascade goes all the way down first.
        bool onWinch = phase == READY && piston == PtoMode::LIFT;
        if (to == PtoMode::INTAKE && onWinch) {
            lowerTime.on = false;  // a new trip down: a fresh give-up clock
            begin(LOWERING);
        } else {
            begin(STOPPING);
        }
    }

    PtoOutput update(const PtoInput& in) {
        if (in.liftKnown) lastHeight = in.liftHeight;
        bool sensorGone = sensorLost.held(!in.liftKnown, in.now) >= PTO_SENSOR_LOST_MS;
        bool down = sensorGone || lastHeight <= LIFT_BOTTOM + PTO_DOWN_SLACK;

        // Go where the driver is working. A fresh intake press (B, Y, or one
        // straight to the other; not just a change in power) means INTAKE...
        int intakeWay = (in.intakePower > 0) - (in.intakePower < 0);
        int lastIntakeWay = (lastIntakePower > 0) - (lastIntakePower < 0);
        if (PTO_FOLLOW_DRIVER && intakeWay != 0 && intakeWay != lastIntakeWay) shift(PtoMode::INTAKE);
        lastIntakePower = in.intakePower;
        // ...and a fresh lift press means LIFT: up, or down if there's anywhere to
        // go and it isn't already on its way down to shift to INTAKE.
        if (PTO_FOLLOW_DRIVER && in.liftAsk != 0 && in.liftAsk != lastLiftAsk) {
            if (in.liftAsk > 0 || (!down && phase != LOWERING)) shift(PtoMode::LIFT);
        }
        lastLiftAsk = in.liftAsk;

        if (!in.enabled) {
            // The motors are off while disabled, so a shift in progress starts
            // over once enabled, a trip down carries on, and nothing counts
            // toward the checks.
            if (phase == STOPPING || phase == SHIFTING) begin(STOPPING);
            clearTimers();
            lowerTime.on = false;
            PtoOutput out = output(in);
            out.power = 0;
            out.lowerLift = false;
            return out;
        }

        std::uint32_t inPhase = phaseTime.held(true, in.now);
        if (phase == LOWERING) {
            // The give-up clock only runs while the lift is left to go down on its
            // own: not while the driver is holding R1/R2.
            bool stuck = lowerTime.held(in.liftAsk == 0, in.now) >= PTO_LOWER_TIMEOUT_MS;
            if (down || stuck) {
                if (!down) {
                    gaveUpLowering++;
                    forceShift = true;
                }
                begin(STOPPING);
                inPhase = phaseTime.held(true, in.now);
            }
        }
        bool offBottom = !sensorGone && lastHeight > LIFT_BOTTOM + PTO_DOWN_SLACK + PTO_DOWN_HYSTERESIS;
        if (phase == STOPPING && target == PtoMode::INTAKE && piston == PtoMode::LIFT && settled &&
            offBottom && !forceShift) {
            // Off the bottom with the gear on the winch (it was moving up when the
            // trip down was asked for, and coasted up while stopping): go back
            // down. The give-up clock keeps running from the first trip.
            begin(LOWERING);
        }
        if (phase == STOPPING) {
            bool liftStill = liftHeldFor.held(in.liftHeld, in.now) >= PTO_SETTLE_MS;
            if (liftStill && (still(in) || inPhase >= PTO_STOP_TIMEOUT_MS)) {
                piston = target;
                forceShift = false;
                settled = false;
                pushing = false;  // the gear is moving: it has to show it's meshed again
                noMesh = false;
                candidate = 0;
                begin(SHIFTING);
                phaseTime.held(true, in.now);
            }
        } else if (phase == SHIFTING && inPhase >= PTO_SHIFT_MS) {
            phase = READY;
            settled = true;
            clearTimers();
        }
        if (liftFree()) watchLift(in);
        return output(in);
    }

    // Turning together, the way given: speeds within PTO_STEP_FRACTION of the
    // lift's speed of each other, give or take PTO_STEP_RPM.
    static bool inStep(double lift, double pto, int way) {
        return std::fabs(pto - way * lift) <= PTO_STEP_FRACTION * std::fabs(lift) + PTO_STEP_RPM;
    }

  private:
    PtoTimer phaseTime, liftHeldFor, agreeing, moving, outOfStep;
    // Not phase timers, so clearTimers() leaves them alone: the give-up clock
    // for a trip down (it carries on if it has to go back down again), and how
    // long the lift sensor has been silent.
    PtoTimer lowerTime, sensorLost;

    void begin(Phase p) {
        phase = p;
        clearTimers();
    }

    void clearTimers() {
        phaseTime.on = false;
        liftHeldFor.on = false;
        agreeing.on = false;
        moving.on = false;
        outOfStep.on = false;
    }

    static bool still(const PtoInput& in) {
        return std::fabs(in.liftRpm) < PTO_STILL_RPM && std::fabs(in.ptoRpm) < PTO_STILL_RPM;
    }

    void watchLift(const PtoInput& in) {
        double lift = in.liftRpm, pto = in.ptoRpm;
        if (!std::isfinite(lift) || !std::isfinite(pto)) {  // a motor is unplugged
            agreeing.on = false;
            moving.on = false;
            outOfStep.on = false;
            return;
        }

        if (pushing) {
            if (outOfStep.held(!inStep(lift, pto, dir), in.now) >= PTO_SLIP_MS) {
                pushing = false;
                slips++;
                candidate = 0;
                agreeing.on = false;
                moving.on = false;
            }
            return;
        }

        // Not pushing yet: wait until it's clearly turning with the lift, one
        // way, for a moment. The lift motors move it alone until then.
        int way = 0;
        if (std::fabs(lift) >= PTO_MESH_RPM) {
            if (inStep(lift, pto, 1)) {
                way = 1;
            } else if (inStep(lift, pto, -1)) {
                way = -1;
            }
        }
        if (way != candidate) {
            candidate = way;
            agreeing.on = false;
        }
        if (way != 0 && agreeing.held(true, in.now) >= PTO_MESH_MS) {
            pushing = true;
            dir = way;
            noMesh = false;
            meshes++;
            moving.on = false;
            outOfStep.on = false;
            return;
        }
        if (moving.held(std::fabs(lift) >= PTO_MESH_RPM, in.now) >= PTO_NO_MESH_MS) noMesh = true;
    }

    PtoOutput output(const PtoInput& in) const {
        if (phase == STOPPING) return {piston, 0, pros::E_MOTOR_BRAKE_BRAKE, false};
        if (phase == SHIFTING) return {piston, PTO_JOG_POWER, pros::E_MOTOR_BRAKE_COAST, false};
        if (phase == READY && target == PtoMode::INTAKE) {
            return {piston, std::clamp(in.intakePower, -127, 127), pros::E_MOTOR_BRAKE_COAST, false};
        }
        // On the winch: in LIFT, or taking the cascade down on the way to INTAKE.
        bool lowering = phase == LOWERING;
        if (!pushing) return {piston, 0, pros::E_MOTOR_BRAKE_COAST, lowering};
        // BRAKE, not HOLD: when the lift stops, the lift motors hold it, and this
        // one resists without pulling toward a hold point of its own.
        return {piston, std::clamp(dir * in.liftPower, -127, 127), pros::E_MOTOR_BRAKE_BRAKE, lowering};
    }
};
