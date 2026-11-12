// Runs the PTO logic from pto.hpp, together with the real lift logic from
// lift.hpp, against a simulated robot: a cascade on a winch, a piston that takes
// a moment to slide the gear over, teeth that can land tooth-on-tooth, speed
// readings that lag the real speed, and an intake motor that's locked to the
// winch once its gear is on it. Each tick runs like the robot: the driver loop
// calls liftManual() and intakeMove() and acts on ptoClawReady() (before or
// after the lift task), then the lift task reads ptoLiftHold(), updates the
// lift, runs the PTO and starts or ends the PTO's move to clamp height. The
// lift slows down when held about as fast as it speeds up, not instantly. Tests
// end with checks(): the intake motor never pushed against the lift motors (its
// slow turn during a shift, against the held lift, is on purpose and not
// counted), never ran the intake with its gear on the winch, the cascade never
// moved in INTAKE except on its way to clamp height, the piston only moved with
// everything still, and the lift never decided its sensor was backwards. Run
// test/run_tests.sh.
#include "pto.hpp"
#include <cassert>
#include <cstdio>
#include <deque>

struct Sim {
    // The robot
    int wiring = 1;                // intake motor power that raises the lift: +1 or -1
    bool pistonWorks = true;       // false: no air, or PTO_PISTON is the wrong port
    std::uint32_t strokeMs = 120;  // piston travel
    bool toothOnTooth = false;     // the gear lands on a tooth and has to turn to drop in
    double liftTau = 0.05;         // how quickly the lift gets up to speed, s
    double holdTau = 0.04;         // how quickly the held lift comes to a stop, s
    double gravityRpm = 20;        // how much the lift's weight slows one motor
    double coastTau = 0.3;         // how long the intake motor takes to coast down, s
    double noise = 2;              // rpm of noise on every speed reading
    int lagTicks = 2;              // speed readings are this many ticks old
    double degPerRev = 1080;       // lift sensor degrees per lift motor turn (3:1 to the winch)
    double topHeight = 1500;       // the top hard stop, lift sensor degrees
    double jamHeight = -1;         // something stops the cascade going below this
    bool liftUnplugged = false;
    bool sensorBroken = false;     // the lift's rotation sensor isn't answering
    std::uint32_t glitchAt = 0;    // ...or misses one reading, at this time
    double heightNoise = 0;        // degrees of jitter on the lift sensor
    bool driverLate = false;       // the driver loop runs after the lift task this tick
    bool ptoStuck = false;         // the intake motor's speed reading is stuck...
    double stuckRpm = 0;           // ...at this

    enum Where { ON_INTAKE, BETWEEN, ON_WINCH };
    Where where = ON_INTAKE;
    PtoMode goingTo = PtoMode::INTAKE;
    std::uint32_t arrives = 0;
    std::uint32_t leavesWinch = 0; // going to INTAKE, the winch's teeth hold the gear until then
    bool sliding = false;          // fired toward the winch and not in its teeth yet
    double toothLeft = 0;          // degrees the gear still has to turn to drop in

    double liftW = 0, ptoW = 0;    // real speeds in motor rpm, lift + is up
    double height = 0;             // real lift sensor degrees above the bottom
    std::deque<double> liftSeen, ptoSeen;  // what the speed readings will say
    int ptoPower = 0;
    pros::motor_brake_mode_e_t brake = pros::E_MOTOR_BRAKE_COAST;
    PtoMode pistonSent = PtoMode::INTAKE;

    // The code
    LiftControl lc;
    PtoControl c;
    // What the driver is doing
    int liftCmd = 0;   // R2 127, R1 -127
    int intake = 0;    // B 127, Y -127
    bool downPress = false;  // DOWN this tick
    bool enabled = true;
    bool shiftMidTick = false;          // a shift request that lands after the lift task
    PtoMode midTickTo = PtoMode::INTAKE;  // read ptoHoldingLift() but before ptoStep()
    std::uint32_t now = 1;

    // What happened
    int fights = 0;            // ticks the intake motor pushed against the lift motors
    int intakeOnWinch = 0;     // ticks the intake motor ran for the intake with its gear on the winch
    int movesInIntake = 0;     // ticks the lift was sent power in INTAKE, other than to clamp height
    int grinds = 0;            // ticks the lift was sent power with the gear sliding onto the winch
    int drags = 0;             // ticks the lift went for clamp height with the gear still on the winch
    int clawReadies = 0;       // times the driver loop got the claw ready
    bool wristDown = false, clawOpen = true, autograbArmed = false;
    int pistonMoves = 0;
    double fastestAtFire = 0;  // fastest anything was really turning when the piston moved
    double highestEver = 0;
    std::uint32_t liftMovedAt = 0;  // last time the lift was sent power
    std::uint32_t firedAt = 0;
    std::uint32_t shortestGapAtFire = 0xffffffffu;  // least time from lift power to a piston move
    std::uint32_t lastGapAtFire = 0;
    unsigned seed = 1;

    Sim() { lc.setZero(0); }  // what initLift() does: matches start with the cascade down

    double rnd() {
        seed = seed * 1103515245u + 12345u;
        return ((seed >> 16) & 0x7fff) / 32767.0 * 2 - 1;
    }

    void driverLoop() {
        if (enabled) lc.manual(liftCmd);
        if (enabled && c.clampReady != clawReadies) {  // ptoClawReady()
            clawReadies = c.clampReady;
            wristDown = true;
            clawOpen = true;
            autograbArmed = true;
        }
        if (downPress) {
            c.shift(c.target == PtoMode::LIFT ? PtoMode::INTAKE : PtoMode::LIFT);
            downPress = false;
        }
    }

    double reading(std::deque<double>& seen, double value) {
        seen.push_back(value);
        while (static_cast<int>(seen.size()) > lagTicks + 1) seen.pop_front();
        return seen.front() + noise * rnd();
    }

    void step() {
        // The driver loop: before the lift task, or (driverLate) after it.
        if (!driverLate) driverLoop();

        // The lift task, as in lift.cpp.
        bool ok = !sensorBroken && now != glitchAt;
        double raw = height + heightNoise * rnd();  // what the lift sensor reads
        PtoLiftHold hold = c.liftHold();  // ptoLiftHold()
        bool held = hold == PtoLiftHold::HELD;
        double liftRead = reading(liftSeen, liftW), ptoRead = reading(ptoSeen, ptoW);
        double liftRpm = liftUnplugged ? PROS_ERR_F : liftRead;
        if (!enabled) lc.manual(0);
        int power;
        if (ok) {
            bool mayMove = hold == PtoLiftHold::FREE || (hold == PtoLiftHold::PTO_MOVE_ONLY && lc.ptoMove);
            power = lc.update(raw, liftRpm, mayMove && enabled, now);
        } else {
            if (lc.mode != LiftControl::MANUAL) lc.mode = LiftControl::IDLE;
            power = lc.mode == LiftControl::MANUAL && hold == PtoLiftHold::FREE ? lc.manualPower : 0;
        }
        if (enabled && power != 0) liftMovedAt = now;
        if (shiftMidTick) {
            c.shift(midTickTo);
            shiftMidTick = false;
        }
        PtoInput in;
        in.now = now;
        in.enabled = enabled;
        in.liftPower = power;
        in.liftHeld = held;
        in.liftAsk = lc.ask();
        in.liftKnown = ok && lc.zeroed && !lc.backwards;
        in.liftHeight = in.liftKnown ? lc.height(raw) : 0;
        in.intakePower = enabled ? intake : 0;  // pto.cpp drops it while disabled
        in.liftRpm = liftRpm;
        in.ptoRpm = ptoStuck ? stuckRpm : ptoRead;
        PtoOutput out = c.update(in);
        assert(enabled || out.power == 0);
        if (ok) {
            if (out.toClamp) {
                lc.moveForPto(PTO_CLAMP_HEIGHT, raw, liftRpm, now);
            } else {
                lc.endPtoMove();
            }
        }
        if (driverLate) driverLoop();

        if (out.piston != pistonSent) {
            pistonSent = out.piston;
            pistonMoves++;
            firedAt = now;
            lastGapAtFire = now - liftMovedAt;
            shortestGapAtFire = std::min(shortestGapAtFire, lastGapAtFire);
            fastestAtFire = std::max(fastestAtFire, std::max(std::fabs(liftW), std::fabs(ptoW)));
            if (pistonWorks) {
                goingTo = out.piston;
                // Leaving the winch, the gear stays in its teeth for the first half of the stroke.
                leavesWinch = where == ON_WINCH && goingTo == PtoMode::INTAKE ? now + strokeMs / 2 : 0;
                where = leavesWinch ? ON_WINCH : BETWEEN;
                sliding = goingTo == PtoMode::LIFT;
                arrives = now + strokeMs;
                toothLeft = toothOnTooth ? 8 : 0;
            }
        }
        ptoPower = enabled ? out.power : 0;
        brake = out.brake;
        bool intakeJob = c.phase == PtoControl::READY && c.target == PtoMode::INTAKE;
        if (where == ON_WINCH && intakeJob && ptoPower != 0) intakeOnWinch++;
        if (enabled && power != 0 && sliding) grinds++;
        if (enabled && power != 0 && where == ON_WINCH && intakeJob) drags++;
        physics(enabled ? power : 0);
        if (intakeJob && c.settled && !c.toClamp && enabled && power != 0) movesInIntake++;
        highestEver = std::max(highestEver, height);
        now += 10;
    }

    void physics(int liftPower) {
        const double dt = 0.01;
        bool onWinch = where == ON_WINCH;
        if (onWinch && ptoPower != 0 && liftPower != 0 && (wiring * ptoPower > 0) != (liftPower > 0)) {
            fights++;
        }

        // The cascade. Its motors hold when sent 0, and that beats anything the
        // intake motor can do at jog power.
        if (liftPower == 0) {
            liftW += (0 - liftW) * dt / holdTau;
            if (std::fabs(liftW) < 0.5) liftW = 0;
        } else {
            int motors = onWinch ? 3 : 2;
            double drive = 2.0 * liftPower + (onWinch ? wiring * ptoPower : 0);
            double target = drive / motors / 127.0 * 100 - gravityRpm / motors;
            liftW += (target - liftW) * dt / liftTau;
        }
        height += liftW / 60 * degPerRev * dt;
        if (height >= topHeight && liftW > 0) {  // the top hard stop
            height = topHeight;
            liftW = 0;
        }
        double bottom = std::max(0.0, jamHeight);
        if (height <= bottom && liftW < 0) {
            height = bottom;
            liftW = 0;
        }

        // The intake motor: locked to the winch when its gear is on it, otherwise on its own.
        if (onWinch && leavesWinch && now >= leavesWinch) {
            where = BETWEEN;
            leavesWinch = 0;
        } else if (onWinch) {
            ptoW = wiring * liftW;
            return;
        }
        if (ptoPower == 0 && brake != pros::E_MOTOR_BRAKE_COAST) {
            ptoW += (0 - ptoW) * dt / 0.03;
            if (std::fabs(ptoW) < 0.5) ptoW = 0;
        } else {
            double drag = where == ON_INTAKE ? 0.9 : 1.0;  // the rollers
            double tau = ptoPower == 0 ? coastTau : 0.05;  // coasting takes a while
            ptoW += (ptoPower / 127.0 * 100 * drag - ptoW) * dt / tau;
        }
        if (where == BETWEEN && now >= arrives) {
            // Pressed against the other gear: drops in once it has turned past the tooth.
            double other = goingTo == PtoMode::LIFT ? wiring * liftW : 0;
            toothLeft -= std::fabs(ptoW - other) * 6 * dt;
            if (toothLeft <= 0) {
                where = goingTo == PtoMode::LIFT ? ON_WINCH : ON_INTAKE;
                sliding = false;
            }
        }
    }

    void run(int ms) {
        for (int t = 0; t < ms; t += 10) step();
    }

    // Steps until the PTO is settled where it's going. Returns how long that took.
    std::uint32_t settle(int ms = 5000) {
        std::uint32_t start = now;
        do {
            step();
        } while (c.phase != PtoControl::READY && now - start < static_cast<std::uint32_t>(ms));
        assert(c.phase == PtoControl::READY);
        return now - start;
    }

    // Holds R2 until the cascade is about this high, then lets go.
    void raiseTo(double h) {
        liftCmd = 127;
        while (height < h) step();
        liftCmd = 0;
        run(200);
    }

    void checks() const {
        assert(!lc.backwards);
        assert(fights == 0 && intakeOnWinch == 0 && movesInIntake == 0);
        assert(grinds == 0 && drags == 0);
        assert(fastestAtFire < PTO_STILL_RPM);
        assert(shortestGapAtFire >= PTO_SETTLE_MS);
    }
};

static void starts_in_intake_with_the_cascade_held() {
    Sim s;
    s.intake = 127;
    s.run(300);
    assert(s.c.target == PtoMode::INTAKE && s.ptoPower == 127 && s.pistonMoves == 0);
    s.intake = -127;
    s.run(50);
    assert(s.ptoPower == -127 && s.highestEver < 1);
    s.checks();
    std::printf("  starts in INTAKE: B and Y run the intake, the cascade stays put\n");
}

static void r2_shifts_to_lift_then_lifts(int wiring, bool toothOnTooth) {
    Sim s;
    s.wiring = wiring;
    s.toothOnTooth = toothOnTooth;
    s.seed = 7 + wiring + toothOnTooth;
    s.intake = 127;  // B held the whole time: mustn't bounce it back to INTAKE
    s.run(500);
    std::uint32_t pressed = s.now;
    s.liftCmd = 127;
    s.step();
    assert(s.c.target == PtoMode::LIFT && s.c.phase != PtoControl::READY);  // straight away
    while (s.height < 1) {
        s.step();
        assert(s.now - pressed < 2000);
    }
    std::uint32_t moved = s.now - pressed;
    assert(s.pistonMoves == 1 && s.where == Sim::ON_WINCH);  // in the teeth before the cascade moved
    std::uint32_t start = s.now;
    while (!s.c.pushing && s.now - start < 1000) {
        s.step();
        assert(s.c.pushing || s.ptoPower == 0);  // never pushes before it's sure
    }
    std::uint32_t joined = s.now - start;
    assert(s.c.pushing && s.c.dir == wiring);
    s.run(100);
    assert(s.ptoPower == wiring * 127 && s.brake == pros::E_MOTOR_BRAKE_BRAKE);
    assert(s.c.target == PtoMode::LIFT && !s.c.noMesh);
    // Let go: the lift motors hold it, this one brakes.
    s.liftCmd = 0;
    s.run(100);
    assert(s.ptoPower == 0 && s.brake == pros::E_MOTOR_BRAKE_BRAKE);
    s.checks();
    std::printf("  R2 in INTAKE, motor %+d raises%s: cascade moving %u ms after the press, "
                "intake motor joined %u ms later\n",
                wiring, toothOnTooth ? ", tooth-on-tooth" : "", static_cast<unsigned>(moved),
                static_cast<unsigned>(joined));
}

// B with the cascade up: shifts at once, then the two lift motors take the
// cascade to clamp height while the intake runs, and the claw gets ready.
static void b_takes_the_cascade_to_clamp_height(int wiring, double from) {
    Sim s;
    s.wiring = wiring;
    if (from > 0) {
        s.raiseTo(from);
    } else {
        s.downPress = true;  // into LIFT at the bottom, cascade never moved
        s.settle();
    }
    assert(s.c.target == PtoMode::LIFT);
    double h0 = s.height;
    std::uint32_t pressed = s.now;
    s.intake = 127;
    s.step();
    assert(s.c.target == PtoMode::INTAKE && s.c.phase != PtoControl::READY);
    s.settle();
    std::uint32_t intakeAt = s.now - pressed;
    assert(s.c.toClamp && s.clawReadies == 0);
    while (s.c.toClamp && s.now - pressed < 5000) {
        s.step();
        if (s.c.phase == PtoControl::READY) assert(s.ptoPower == 127);  // intake keeps running
    }
    std::uint32_t readyAt = s.now - pressed;
    s.step();
    assert(s.c.clampReady == 1 && s.clawReadies == 1 && s.wristDown && s.clawOpen && s.autograbArmed);
    assert(std::fabs(s.height - PTO_CLAMP_HEIGHT) <= PTO_CLAMP_SLACK);
    s.run(200);  // the last of the coast
    double there = s.height;
    s.run(1000);
    assert(std::fabs(s.height - there) < 1 && s.ptoPower == 127 && s.where == Sim::ON_INTAKE);
    assert(std::fabs(s.height - PTO_CLAMP_HEIGHT) <= PTO_CLAMP_SLACK);
    s.checks();
    std::printf("  B with the cascade at %.0f, motor %+d raises: intake running at %u ms, cascade "
                "at clamp height %.0f and claw ready at %u ms\n",
                h0, wiring, static_cast<unsigned>(intakeAt), there,
                static_cast<unsigned>(readyAt));
}

static void r2_on_the_way_to_clamp_height_cancels() {
    Sim s;
    s.raiseTo(1000);
    s.intake = 127;
    s.settle();
    s.run(150);
    assert(s.c.toClamp && s.height < 1000 && s.height > 300);
    s.liftCmd = 127;  // R2: the driver wants it up after all
    s.step();
    assert(s.c.target == PtoMode::LIFT && !s.c.toClamp);
    s.run(1000);
    assert(s.c.target == PtoMode::LIFT && s.height > 600 && s.clawReadies == 0);
    s.checks();
    std::printf("  R2 on the way to clamp height: back to LIFT, the claw is left alone\n");
}

static void r2_still_held_when_b_is_pressed() {
    // B was pressed last, so INTAKE wins and the cascade stops where it is. R2
    // is still held, so it stays there (no rising on two motors) until R2 is let
    // go, however long that is; then it goes to clamp height.
    Sim s;
    s.liftCmd = 127;
    while (s.height < 600) s.step();
    s.intake = 127;
    s.settle();
    s.run(200);
    double stopped = s.height;
    s.run(PTO_CLAMP_TIMEOUT_MS + 500);
    assert(s.c.target == PtoMode::INTAKE && s.c.toClamp && std::fabs(s.height - stopped) < 1);
    s.liftCmd = 0;
    s.run(2000);
    assert(!s.c.toClamp && s.clawReadies == 1 && std::fabs(s.height - PTO_CLAMP_HEIGHT) <= PTO_CLAMP_SLACK);
    s.checks();
    std::printf("  B with R2 still held: cascade waits at %.0f until R2 is let go, then clamp height\n",
                stopped);
}

static void r1_in_intake_shifts_and_lowers() {
    Sim s;
    s.raiseTo(800);
    s.intake = 127;
    s.run(2000);
    s.intake = 0;
    assert(s.c.target == PtoMode::INTAKE && s.clawReadies == 1);
    s.liftCmd = -127;  // R1 below clamp height
    s.step();
    assert(s.c.target == PtoMode::LIFT);
    s.run(1500);
    assert(s.height <= LIFT_BOTTOM + 3);
    s.checks();
    std::printf("  R1 in INTAKE: shifts to LIFT, comes down below clamp height\n");
}

static void r1_at_the_bottom_does_nothing() {
    Sim s;
    s.liftCmd = -127;
    s.run(500);
    assert(s.pistonMoves == 0 && s.c.target == PtoMode::INTAKE && s.highestEver < 1);
    s.checks();
    std::printf("  R1 in INTAKE with the cascade at the bottom: no shift, nothing moves\n");
}

static void the_piston_waits_for_the_cascade_to_stop() {
    // R2 lifting at full speed, B landing just after the lift task sent power
    // and while the lagging speed readings still say it's moving. The piston
    // waits until the cascade has really been held still.
    Sim s;
    s.liftCmd = 127;
    while (s.height < 500) s.step();
    s.shiftMidTick = true;
    s.midTickTo = PtoMode::INTAKE;
    s.step();
    assert(s.c.phase == PtoControl::STOPPING && s.pistonMoves == 1);
    s.settle();
    s.liftCmd = 0;
    s.run(100);
    s.checks();
    std::printf("  shift asked for while lifting at full speed: piston fired %u ms after the "
                "last lift power, %.1f rpm\n",
                static_cast<unsigned>(s.lastGapAtFire), s.fastestAtFire);
}

static void changing_its_mind() {
    // Before the piston moves: it just stays.
    Sim s;
    s.intake = 127;
    s.run(500);
    s.c.shift(PtoMode::LIFT);
    s.step();
    assert(s.c.phase == PtoControl::STOPPING);  // still spinning down
    s.c.shift(PtoMode::INTAKE);
    assert(s.c.phase == PtoControl::READY && s.c.target == PtoMode::INTAKE);
    s.run(100);
    assert(s.pistonMoves == 0 && s.ptoPower == 127);

    // After the piston has moved, going back means a full shift back.
    Sim t;
    t.c.shift(PtoMode::LIFT);
    t.run(100);
    assert(t.c.phase == PtoControl::SHIFTING);
    t.c.shift(PtoMode::INTAKE);
    t.settle();
    t.run(200);
    assert(t.pistonMoves == 2 && t.where == Sim::ON_INTAKE && t.c.target == PtoMode::INTAKE);

    // DOWN, B, DOWN in quick succession after the piston fired: the gear is still
    // sliding toward the winch, so it still gets its full turn into the teeth
    // before the lift is let go.
    Sim u;
    u.toothOnTooth = true;
    u.c.shift(PtoMode::LIFT);
    while (u.pistonMoves == 0) u.step();
    u.step();
    u.c.shift(PtoMode::INTAKE);
    u.c.shift(PtoMode::LIFT);
    std::uint32_t start = u.now;
    u.settle();
    assert(u.c.target == PtoMode::LIFT && u.c.lastShiftLatched && u.where == Sim::ON_WINCH);
    s.checks();
    t.checks();
    u.checks();
    std::printf("  changing its mind mid-shift ends up where it was told, gear fully in\n");
}

// However long the piston takes, and with the gear landing tooth-on-tooth, the
// cascade only moves once the gear has latched: in the winch's teeth for LIFT,
// clear of them for INTAKE. A quick latch doesn't wait the whole PTO_SHIFT_MS.
static void waits_for_the_gear_to_latch(std::uint32_t strokeMs) {
    Sim s;
    s.strokeMs = strokeMs;
    s.toothOnTooth = true;
    s.liftCmd = 127;
    std::uint32_t start = s.now;
    while (s.height < 1) {
        s.step();
        assert(s.now - start < 2000);
    }
    std::uint32_t lifted = s.now - start;
    s.raiseTo(800);
    s.intake = 127;
    start = s.now;
    s.settle();
    std::uint32_t freed = s.now - start;
    s.run(3000);
    assert(s.clawReadies == 1);
    s.checks();  // no grinds or drags
    std::printf("  %u ms piston, tooth-on-tooth: R2 lifted after %u ms, B freed the cascade after %u ms\n",
                static_cast<unsigned>(strokeMs), static_cast<unsigned>(lifted), static_cast<unsigned>(freed));
}

static void stuck_missing_or_lying_readings() {
    // A speed reading that never drops: fires after the stop timeout anyway.
    Sim s;
    s.ptoStuck = true;
    s.stuckRpm = 50;
    s.c.shift(PtoMode::LIFT);
    std::uint32_t ms = s.settle();
    assert(s.pistonMoves == 1);
    assert(ms >= PTO_STOP_TIMEOUT_MS + PTO_SHIFT_MS && ms <= PTO_STOP_TIMEOUT_MS + PTO_SHIFT_MS + 40);

    // Lift motor speed unreadable: R2 still shifts and lifts; the intake motor
    // never joins; B still shifts back.
    Sim t;
    t.liftUnplugged = true;
    t.liftCmd = 127;
    t.run(1500);
    assert(t.c.target == PtoMode::LIFT && !t.c.pushing && t.ptoPower == 0 && !t.c.noMesh);
    t.liftCmd = 0;
    t.intake = 127;
    t.settle(1000);
    t.run(50);
    assert(t.c.target == PtoMode::INTAKE && t.ptoPower == 127);

    // Lift sensor dead: R1 can't know the cascade is at the bottom, so it shifts
    // to LIFT and leaves it to the driver.
    Sim u;
    u.sensorBroken = true;
    u.liftCmd = -127;
    u.run(600);
    assert(u.c.target == PtoMode::LIFT);
    s.checks();
    t.checks();
    u.checks();
    std::printf("  stuck reading: shifted in %u ms; unreadable lift motor: stays out of it; dead "
                "lift sensor: R1 still shifts\n",
                static_cast<unsigned>(ms));
}

static void disabled_mid_shift_starts_over() {
    Sim s;
    s.c.shift(PtoMode::LIFT);
    s.run(100);
    assert(s.c.phase == PtoControl::SHIFTING);
    s.enabled = false;  // step() checks nothing is sent to the motor meanwhile
    s.run(1000);
    s.enabled = true;
    std::uint32_t start = s.now;
    s.settle();
    assert(s.now - start >= PTO_LATCH_MIN_MS && s.c.lastShiftLatched);  // the shift ran again
    s.liftCmd = 127;
    s.run(800);
    assert(s.c.pushing && s.where == Sim::ON_WINCH);
    s.checks();
    std::printf("  disabled mid-shift: nothing moves, finishes the shift once enabled\n");
}

static void disabled_or_no_sensor_on_the_way_to_clamp_height() {
    // Disabled on the way: carries on once enabled, then the claw gets ready.
    Sim s;
    s.raiseTo(1000);
    s.intake = 127;
    s.settle();
    s.run(100);
    assert(s.c.toClamp);
    s.enabled = false;
    s.run(1000);
    s.enabled = true;
    s.run(2000);
    assert(!s.c.toClamp && s.clawReadies == 1);
    s.checks();

    // Lift sensor dead: it can't know where clamp height is, so nothing moves,
    // it gives up, the claw is left alone, and the intake still runs.
    Sim t;
    t.raiseTo(800);
    t.sensorBroken = true;
    t.intake = 127;
    t.settle();
    double h = t.height;
    t.run(PTO_CLAMP_TIMEOUT_MS + 300);
    assert(!t.c.toClamp && t.c.clampGaveUp == 1 && t.clawReadies == 0 && t.ptoPower == 127);
    assert(std::fabs(t.height - h) < 1);
    t.checks();
    std::printf("  disabled on the way to clamp height: carries on; dead lift sensor: stays put, "
                "claw left alone\n");
}

static void no_air_shows_nomesh(double liftTau, int power) {
    Sim s;
    s.pistonWorks = false;
    s.liftTau = liftTau;
    s.coastTau = 2;  // the loose gear keeps spinning from the shift as long as it can
    s.liftCmd = power;
    s.run(1800);
    // Never mistakes the loose gear, still turning a little from the shift, for
    // turning with the lift.
    assert(s.c.target == PtoMode::LIFT && !s.c.pushing && s.c.meshes == 0 && s.ptoPower == 0);
    assert(s.c.noMesh && s.height > 100);  // the cascade still works on its two motors
    assert(s.c.shifts == 1 && !s.c.lastShiftLatched);  // and the terminal said why
    s.liftCmd = 0;
    s.intake = 127;
    s.settle();
    s.run(50);
    assert(!s.c.noMesh && s.c.target == PtoMode::INTAKE && s.ptoPower == 127);
    s.checks();
    std::printf("  piston never moves, R2 at power %d with a %.2f s lift: NOMESH, never pushes\n",
                power, liftTau);
}

static void slipping_out_stops_it_pushing() {
    Sim s;
    s.wiring = -1;
    s.liftCmd = 127;
    while (!s.c.pushing) s.step();
    s.run(100);
    // The gear pops out and, pushed at the lift's power, spins at about the
    // lift's speed. That looks fine until the cascade stops at the top and the
    // gear doesn't.
    s.where = Sim::BETWEEN;
    s.arrives = s.now;
    s.toothLeft = 1e9;
    s.topHeight = s.height + 60;
    std::uint32_t start = s.now;
    while (s.c.pushing && s.now - start < 2000) s.step();
    assert(!s.c.pushing && s.c.slips == 1);
    s.run(50);
    assert(s.ptoPower == 0 && s.brake == pros::E_MOTOR_BRAKE_COAST);
    s.checks();
    std::printf("  gear slips out: stopped pushing %u ms after, once the cascade reached the top\n",
                static_cast<unsigned>(s.now - start));
}

static void y_straight_to_b_counts_as_a_press() {
    Sim s;
    s.intake = -127;  // Y
    s.run(200);
    s.liftCmd = 127;  // then R2: LIFT wins
    s.run(600);
    s.liftCmd = 0;
    s.run(200);
    assert(s.c.target == PtoMode::LIFT);
    s.intake = 127;  // B pressed with Y still held: B wins in opcontrol
    s.settle();
    s.run(50);
    assert(s.c.target == PtoMode::INTAKE && s.ptoPower == 127);
    s.checks();
    std::printf("  Y held, R2, then B: B still counts as a fresh press\n");
}

static void an_auton_changing_intake_power_isnt_a_press() {
    Sim s;
    s.intake = 127;
    s.run(200);
    s.liftCmd = 127;  // R2: LIFT
    s.run(700);
    s.liftCmd = 0;
    s.run(200);
    assert(s.c.target == PtoMode::LIFT);
    s.intake = 100;  // same direction, different power
    s.run(300);
    assert(s.c.target == PtoMode::LIFT && s.c.phase == PtoControl::READY);
    s.checks();
    std::printf("  intake power 127 -> 100 in LIFT: not a fresh press, stays in LIFT\n");
}

static void a_random_driver_for_two_minutes(int wiring, double holdTau) {
    Sim s;
    s.wiring = wiring;
    s.holdTau = holdTau;
    s.noise = 3;
    s.seed = 99 + wiring + static_cast<unsigned>(holdTau * 1000);
    int shifts = 0, ups = 0, intakes = 0, glitches = 0;
    PtoMode was = s.c.target;
    std::uint32_t end = s.now + 120000;
    while (s.now < end) {
        double r = s.rnd();
        // Mostly R2/R1/B/Y, sometimes both at once, sometimes DOWN or a disable.
        int lift = r < -0.5 ? 127 : r < -0.2 ? -127 : 0;
        double q = s.rnd();
        int in = q < -0.4 ? 127 : q < -0.2 ? -127 : 0;
        if (lift == 0 && in == 0 && s.rnd() > 0.6) s.downPress = true;
        if (s.height > s.topHeight - 200 && lift > 0) lift = 0;
        s.liftCmd = lift;
        s.intake = in;
        s.driverLate = s.rnd() > 0;  // the two tasks interleave either way
        if (s.rnd() > 0.8) {         // a missed lift sensor reading somewhere in there
            s.glitchAt = s.now + 10 * static_cast<std::uint32_t>((s.rnd() + 1) * 20);
            glitches++;
        }
        if (s.rnd() > 0.97) {
            s.enabled = false;
            s.run(300);
            s.enabled = true;
        }
        s.run(100 + static_cast<int>((s.rnd() + 1) * 400));
        if (s.c.target != was) shifts++;
        was = s.c.target;
        ups += lift > 0;
        intakes += in != 0;
    }
    s.liftCmd = 0;
    s.intake = 0;
    s.run(1000);
    s.checks();
    assert(s.c.meshes >= 1 && s.c.slips == 0);
    std::printf("  2 min of random driving, motor %+d raises, stop in %.2f s, %d sensor glitches: "
                "%d shifts, %d times to clamp height, never fought\n",
                wiring, holdTau, glitches, shifts, s.c.clampReady);
    assert(shifts > 20 && ups > 20 && intakes > 20);
}

int main() {
    starts_in_intake_with_the_cascade_held();
    r2_shifts_to_lift_then_lifts(1, false);
    r2_shifts_to_lift_then_lifts(-1, false);
    r2_shifts_to_lift_then_lifts(1, true);
    r2_shifts_to_lift_then_lifts(-1, true);
    b_takes_the_cascade_to_clamp_height(1, 1000);
    b_takes_the_cascade_to_clamp_height(-1, 1000);
    b_takes_the_cascade_to_clamp_height(1, 0);
    r2_on_the_way_to_clamp_height_cancels();
    r2_still_held_when_b_is_pressed();
    r1_in_intake_shifts_and_lowers();
    disabled_or_no_sensor_on_the_way_to_clamp_height();
    r1_at_the_bottom_does_nothing();
    the_piston_waits_for_the_cascade_to_stop();
    changing_its_mind();
    waits_for_the_gear_to_latch(60);
    waits_for_the_gear_to_latch(400);
    stuck_missing_or_lying_readings();
    disabled_mid_shift_starts_over();
    no_air_shows_nomesh(0.05, 127);
    no_air_shows_nomesh(0.15, 70);
    no_air_shows_nomesh(0.3, 80);
    slipping_out_stops_it_pushing();
    y_straight_to_b_counts_as_a_press();
    an_auton_changing_intake_power_isnt_a_press();
    a_random_driver_for_two_minutes(1, 0.04);
    a_random_driver_for_two_minutes(-1, 0.04);
    a_random_driver_for_two_minutes(1, 0.1);
    a_random_driver_for_two_minutes(-1, 0.1);
    std::puts("pto check passes");
}
