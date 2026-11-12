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
//                              the intake. The two lift motors take the cascade
//                              to PTO_CLAMP_HEIGHT and hold it there, and the
//                              claw gets ready (wrist down, claw open, autograb
//                              armed) to clamp what the intake brings in.
//   LIFT   (piston extended):  the gear is on the winch's 12T, so the motor is a
//                              third lift motor. R1/R2 and every limit in
//                              lift.hpp drive all three.
// Both lift motors and this one reach the winch at 300 rpm (100 rpm x 36/12), so
// in LIFT it just gets the same power they do. That only holds while all three
// have the same red 100 rpm cartridge.
//
// It follows the driver, straight away. A fresh B/Y press shifts to INTAKE, then
// the intake runs while the two lift motors take the cascade to clamp height,
// up or down; once it's there, the claw gets ready. A fresh R1/R2 press shifts
// to LIFT (and cancels a move to clamp height), then the cascade moves on all
// three motors. Whichever was pressed last wins. (R1 with the cascade already
// at the bottom has nothing to do, so it doesn't shift.) DOWN shifts too, so the
// driver can get into LIFT on the way to the goal and skip the pause.
//
// A shift holds the cascade still and stops the motor, fires the piston, then
// turns the gear slowly (PTO_JOG_POWER, the way B turns it) so it drops into the
// teeth. The cascade stays held until the gear has latched: going to LIFT, the
// slow turn stops dead once the gear is in the held winch's teeth; going to
// INTAKE, it turns freely once the gear is off the winch. That's usually about
// 0.2 s. If it never shows (no air, say), the cascade is let go after
// PTO_SHIFT_MS anyway and runs on its two motors. The gear can't slide into a
// turning winch or intake without the teeth clashing, so the pause can't be skipped.
//
// In LIFT the motor doesn't push until it's been seen turning with the lift. That
// also shows which way it has to push, so there is no direction to set, and it
// never pushes against the lift motors.
//
// The lift task runs the PTO (ptoStep), so both get their power in the same 10 ms tick.
//
// First time on the robot, with it up on a stand. To try shifting on its own
// first, set PTO_FOLLOW_DRIVER to false so only DOWN shifts.
//   1. Tap R2 (or DOWN). The gear should slide onto the winch. If it slides the
//      other way, flip PTO_PISTON_LIFT in config.hpp. The terminal prints how
//      long the shift took and whether it saw the gear latch.
//   2. Hold R2 for a moment. The terminal prints "PTO meshed with the lift" and
//      from then on the intake motor lifts too. If the controller shows NOMESH
//      instead, check the air and PTO_PISTON in config.hpp.
//   3. Measure PTO_CLAMP_HEIGHT (see below), then press B. The gear slides back,
//      the intake runs, the cascade goes to clamp height and the claw gets ready.

enum class PtoMode { INTAKE, LIFT };

// Sets the piston to INTAKE. Call once from initialize(), before initLift().
void initPto();
// Shifts to that mode, unless it's already there or on the way.
void ptoShift(PtoMode mode);
// INTAKE <-> LIFT.
void ptoToggle();
// Runs the intake, -127..127, + pulls in. Only does anything in INTAKE; a fresh
// press in LIFT shifts to INTAKE first. Resets to 0 whenever the robot is disabled.
void intakeMove(int power);
// For autons: waits for a shift in progress to finish.
void ptoWaitUntilShifted(std::uint32_t timeoutMs = 1000);

// What the lift task tells the PTO each tick.
struct PtoLiftTick {
    int power = 0;             // what the lift motors were just sent
    bool held = false;         // ptoLiftHold() said HELD at the start of the tick
    int ask = 0;               // LiftControl::ask(): +1 up, -1 down, 0 nothing
    bool heightKnown = false;  // the lift sensor works and is zeroed
    double height = 0;         // lift sensor degrees above the bottom
    double rpm = 0;            // lift motor speed, + is up (PROS_ERR_F if unreadable)
};
// What the PTO wants the lift to do this tick.
struct PtoLiftRequest {
    bool move = false;  // take the cascade to height (LiftControl::moveForPto); else
                        // end any move the PTO started (LiftControl::endPtoMove)
    double height = 0;
};
// What the lift may do this tick: anything (LIFT), only a move the PTO started
// (on the way to clamp height), or nothing (INTAKE, or mid-shift: send it 0).
enum class PtoLiftHold { FREE, PTO_MOVE_ONLY, HELD };
// The lift task calls these every loop.
PtoLiftHold ptoLiftHold();
PtoLiftRequest ptoStep(const PtoLiftTick& tick);
// Driver loop: true once each time the cascade reaches clamp height after a
// switch to INTAKE. Then extend the wrist, open the claw and arm autograb.
bool ptoClawReady();

struct PtoStatus {
    PtoMode mode;   // where it is, or where it's going
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

// R1 in INTAKE only shifts to LIFT if the cascade is more than this many lift
// sensor degrees above LIFT_BOTTOM, where the lift stops going down anyway.
inline constexpr double PTO_DOWN_SLACK = 5;

// Where the cascade goes after a switch to INTAKE: the lift sensor height (see
// "Lift" on the brain screen) at which the claw, with the wrist extended, can
// clamp a cup or pin the intake has brought in.
// TODO: measure. Put the cascade there by hand and read the brain screen.
inline constexpr double PTO_CLAMP_HEIGHT = 50;
// Counts as there within this many degrees, once it has sat there, barely
// moving, for PTO_CLAMP_SETTLE_MS. Then the claw gets ready.
inline constexpr double PTO_CLAMP_SLACK = 10;
inline constexpr std::uint32_t PTO_CLAMP_SETTLE_MS = 50;
// If it hasn't got there in this long (stuck, or no lift sensor), it gives up
// and leaves the claw alone. Time with R1/R2 still held doesn't count: the
// move waits for them to be let go.
inline constexpr std::uint32_t PTO_CLAMP_TIMEOUT_MS = 3000;

// Before firing the piston, the lift has to have been held still this long...
inline constexpr std::uint32_t PTO_SETTLE_MS = 30;
// ...and both the intake motor and the lift turning slower than this, so the
// gear never slides into moving teeth...
inline constexpr double PTO_STILL_RPM = 10;
// ...unless this long has passed since the shift began (a speed reading that
// never settles).
inline constexpr std::uint32_t PTO_STOP_TIMEOUT_MS = 250;
// The longest a shift waits for the gear to latch after firing the piston.
inline constexpr std::uint32_t PTO_SHIFT_MS = 600;
// Power the gear turns at during a shift, so it drops into the teeth instead of
// sitting tooth-on-tooth. About 15 rpm, one tooth per 0.1 s; far too weak to move
// the lift, which is held still. + is the way B turns it, both ways of shifting.
inline constexpr int PTO_JOG_POWER = 20;
// It counts as latched once the piston has had PTO_LATCH_MIN_MS to move and
// then, for PTO_LATCH_MS, the intake motor reads under PTO_LATCH_RPM going to
// LIFT (stopped by the held winch) or over it going to INTAKE (turning freely).
// PTO_LATCH_RPM is about half what the slow turn reaches with nothing stopping it.
inline constexpr std::uint32_t PTO_LATCH_MIN_MS = 150;
inline constexpr std::uint32_t PTO_LATCH_MS = 50;
inline constexpr double PTO_LATCH_RPM = 7;

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
    bool toClamp;                       // take the cascade to PTO_CLAMP_HEIGHT
};

struct PtoControl {
    enum Phase { READY, STOPPING, SHIFTING };
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
    int candidate = 0;     // the way it seems to be turning, before it's sure

    // INTAKE
    bool toClamp = false;  // the lift motors are taking the cascade to clamp height
    int clampReady = 0;    // times it got there; the claw gets ready each time
    int clampGaveUp = 0;   // times it didn't get there in PTO_CLAMP_TIMEOUT_MS

    int lastIntakePower = 0;
    int lastLiftAsk = 0;

    // Shifts finished, for the terminal: how long the last one took, and whether
    // it saw the gear latch or gave up waiting.
    int shifts = 0;
    std::uint32_t lastShiftMs = 0;
    bool lastShiftLatched = false;

    // The lift may do anything while the gear is on the winch and staying there.
    // In INTAKE it only moves on the way to clamp height, and only by the PTO's
    // own move, so a stale R2 still held can't raise it on two motors.
    PtoLiftHold liftHold() const {
        if (phase != READY) return PtoLiftHold::HELD;
        if (target == PtoMode::LIFT) return PtoLiftHold::FREE;
        return toClamp ? PtoLiftHold::PTO_MOVE_ONLY : PtoLiftHold::HELD;
    }

    // Shifting has nothing to do with clocks here: a phase starts counting on the
    // first update() that sees it, so requests from other tasks can't skew it.
    void shift(PtoMode to) {
        if (to == target) return;
        target = to;
        toClamp = false;  // the driver wants something else now
        if (phase == STOPPING && settled && piston == to) {
            phase = READY;  // changed its mind before the piston moved: nothing to undo
            clearTimers();
            return;
        }
        begin(STOPPING);
    }

    PtoOutput update(const PtoInput& in) {
        // Go where the driver is working, straight away. A fresh intake press (B,
        // Y, or one straight to the other; not just a change in power) means INTAKE...
        int intakeWay = (in.intakePower > 0) - (in.intakePower < 0);
        int lastIntakeWay = (lastIntakePower > 0) - (lastIntakePower < 0);
        if (PTO_FOLLOW_DRIVER && intakeWay != 0 && intakeWay != lastIntakeWay) shift(PtoMode::INTAKE);
        lastIntakePower = in.intakePower;
        // ...and a fresh lift press means LIFT: up, or down unless the cascade is
        // already at the bottom (or the sensor can't tell; then it shifts).
        if (PTO_FOLLOW_DRIVER && in.liftAsk != 0 && in.liftAsk != lastLiftAsk) {
            bool atBottom = in.liftKnown && in.liftHeight <= LIFT_BOTTOM + PTO_DOWN_SLACK;
            if (in.liftAsk > 0 || !atBottom) shift(PtoMode::LIFT);
        }
        lastLiftAsk = in.liftAsk;

        if (!in.enabled) {
            // The motors are off while disabled, so a shift in progress starts
            // over once enabled, and nothing counts toward the checks.
            if (phase != READY) begin(STOPPING);
            clearTimers();
            clampTime.on = false;  // a move to clamp height carries on once enabled
            clampSettle.on = false;
            PtoOutput out = output(in);
            out.power = 0;
            return out;
        }

        std::uint32_t inPhase = phaseTime.held(true, in.now);
        if (phase == STOPPING) {
            bool liftStill = liftHeldFor.held(in.liftHeld, in.now) >= PTO_SETTLE_MS;
            if (liftStill && (still(in) || inPhase >= PTO_STOP_TIMEOUT_MS)) {
                piston = target;
                settled = false;
                pushing = false;  // the gear is moving: it has to show it's meshed again
                noMesh = false;
                candidate = 0;
                begin(SHIFTING);
                phaseTime.held(true, in.now);
            }
        } else if (phase == SHIFTING) {
            bool gearIn = latched(in, inPhase);
            if (gearIn || inPhase >= PTO_SHIFT_MS) {
                shifts++;
                lastShiftMs = inPhase;
                lastShiftLatched = gearIn;
                phase = READY;
                settled = true;
                clearTimers();
                if (target == PtoMode::INTAKE) {
                    toClamp = true;  // now the lift motors take it to clamp height
                    clampTime.on = false;
                    clampSettle.on = false;
                }
            }
        }
        if (phase == READY && target == PtoMode::LIFT) watchLift(in);
        if (phase == READY && toClamp) watchClamp(in);
        return output(in);
    }

    // Turning together, the way given: speeds within PTO_STEP_FRACTION of the
    // lift's speed of each other, give or take PTO_STEP_RPM.
    static bool inStep(double lift, double pto, int way) {
        return std::fabs(pto - way * lift) <= PTO_STEP_FRACTION * std::fabs(lift) + PTO_STEP_RPM;
    }

  private:
    PtoTimer phaseTime, liftHeldFor, agreeing, moving, outOfStep, latchTime;

    // The slow turn shows when the gear is in: on the held winch it stops dead;
    // off the winch nothing holds it, so it turns.
    bool latched(const PtoInput& in, std::uint32_t inPhase) {
        bool known = std::isfinite(in.ptoRpm);
        bool turning = known && std::fabs(in.ptoRpm) >= PTO_LATCH_RPM;
        bool looksIn = known && inPhase >= PTO_LATCH_MIN_MS && (target == PtoMode::LIFT ? !turning : turning);
        return latchTime.held(looksIn, in.now) >= PTO_LATCH_MS;
    }
    PtoTimer clampTime, clampSettle;  // not phase timers: clearTimers() leaves them alone

    void watchClamp(const PtoInput& in) {
        bool slow = !std::isfinite(in.liftRpm) || std::fabs(in.liftRpm) < PTO_STILL_RPM;
        bool there = in.liftKnown && std::fabs(in.liftHeight - PTO_CLAMP_HEIGHT) <= PTO_CLAMP_SLACK;
        if (clampSettle.held(there && slow, in.now) >= PTO_CLAMP_SETTLE_MS) {
            toClamp = false;
            clampReady++;
        } else if (clampTime.held(in.liftAsk == 0, in.now) >= PTO_CLAMP_TIMEOUT_MS) {
            toClamp = false;
            clampGaveUp++;
        }
    }

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
        latchTime.on = false;
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
        if (target == PtoMode::INTAKE) {
            // On the way to clamp height, the lift only drives while outside the
            // window around it; inside, the lift motors just hold it while it
            // settles, so it doesn't hunt back and forth.
            bool inWindow = in.liftKnown && std::fabs(in.liftHeight - PTO_CLAMP_HEIGHT) <= PTO_CLAMP_SLACK;
            return {piston, std::clamp(in.intakePower, -127, 127), pros::E_MOTOR_BRAKE_COAST,
                    toClamp && !inWindow};
        }
        if (!pushing) return {piston, 0, pros::E_MOTOR_BRAKE_COAST, false};
        // BRAKE, not HOLD: when the lift stops, the lift motors hold it, and this
        // one resists without pulling toward a hold point of its own.
        return {piston, std::clamp(dir * in.liftPower, -127, 127), pros::E_MOTOR_BRAKE_BRAKE, false};
    }
};
