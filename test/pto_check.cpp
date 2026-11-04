// Runs the PTO logic from pto.hpp, together with the real lift logic from
// lift.hpp, against a simulated robot: a cascade on a winch, a piston that takes
// a moment to slide the gear over, teeth that can land tooth-on-tooth, speed
// readings that lag the real speed, and an intake motor that's locked to the
// winch once its gear is on it. Each tick runs like the robot: the driver loop
// calls liftManual() and intakeMove(), then the lift task reads
// ptoHoldingLift(), updates the lift, runs the PTO, and starts a trip down if
// the PTO asks. The lift slows down when held about as fast as it speeds up,
// not instantly. Tests end with checks(): the intake motor never pushed against
// the lift motors (its slow turn during a shift, against the held lift, is on
// purpose and not counted), never ran the intake with its gear on the winch,
// the cascade never rose in INTAKE, the piston only moved with everything
// still, never toward INTAKE with the cascade up, and the lift never decided
// its sensor was backwards. Run test/run_tests.sh.
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
    int risesInIntake = 0;     // ticks the cascade rose with the PTO settled in INTAKE
    int pistonMoves = 0;
    int bounces = 0;           // times it had to go back down while stopping to shift
    double fastestAtFire = 0;  // fastest anything was really turning when the piston moved
    double highestToIntake = 0;  // highest the cascade was when the piston moved to INTAKE
    double highestEver = 0;
    std::uint32_t liftMovedAt = 0;  // last time the lift was sent power
    std::uint32_t firedAt = 0;
    unsigned seed = 1;

    Sim() { lc.setZero(0); }  // what initLift() does: matches start with the cascade down

    double rnd() {
        seed = seed * 1103515245u + 12345u;
        return ((seed >> 16) & 0x7fff) / 32767.0 * 2 - 1;
    }

    void driverLoop() {
        if (enabled) lc.manual(liftCmd);
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
        bool held = !c.liftFree();  // ptoHoldingLift()
        double liftRead = reading(liftSeen, liftW), ptoRead = reading(ptoSeen, ptoW);
        double liftRpm = liftUnplugged ? PROS_ERR_F : liftRead;
        if (!enabled) lc.manual(0);
        int power;
        if (ok) {
            power = lc.update(raw, liftRpm, !held && enabled, now);
        } else {
            if (lc.mode != LiftControl::MANUAL) lc.mode = LiftControl::IDLE;
            power = lc.mode == LiftControl::MANUAL && !held ? lc.manualPower : 0;
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
        PtoControl::Phase before = c.phase;
        PtoOutput out = c.update(in);
        if (before == PtoControl::STOPPING && c.phase == PtoControl::LOWERING) bounces++;
        assert(enabled || out.power == 0);
        if (ok) {
            if (out.lowerLift) {
                lc.lowerForPto(raw, liftRpm, now);
            } else {
                lc.endPtoTrip();
            }
        }
        if (driverLate) driverLoop();

        if (out.piston != pistonSent) {
            pistonSent = out.piston;
            pistonMoves++;
            firedAt = now;
            fastestAtFire = std::max(fastestAtFire, std::max(std::fabs(liftW), std::fabs(ptoW)));
            if (out.piston == PtoMode::INTAKE) highestToIntake = std::max(highestToIntake, height);
            if (pistonWorks) {
                goingTo = out.piston;
                where = BETWEEN;
                arrives = now + strokeMs;
                toothLeft = toothOnTooth ? 8 : 0;
            }
        }
        ptoPower = enabled ? out.power : 0;
        brake = out.brake;
        bool intakeJob = c.phase == PtoControl::READY && c.target == PtoMode::INTAKE;
        if (where == ON_WINCH && intakeJob && ptoPower != 0) intakeOnWinch++;
        physics(enabled ? power : 0);
        if (intakeJob && c.settled && liftW > 1) risesInIntake++;
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
        if (onWinch) {
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
            if (toothLeft <= 0) where = goingTo == PtoMode::LIFT ? ON_WINCH : ON_INTAKE;
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
        assert(fights == 0 && intakeOnWinch == 0 && risesInIntake == 0);
        assert(fastestAtFire < PTO_STILL_RPM);
        assert(highestToIntake <= LIFT_BOTTOM + PTO_DOWN_SLACK + PTO_DOWN_HYSTERESIS);
    }
};

static void starts_in_intake_with_the_cascade_held_down() {
    Sim s;
    s.intake = 127;
    s.run(300);
    assert(s.c.target == PtoMode::INTAKE && s.ptoPower == 127 && s.pistonMoves == 0);
    s.intake = -127;
    s.run(50);
    assert(s.ptoPower == -127 && s.highestEver < 1);
    s.checks();
    std::printf("  starts in INTAKE: B and Y run the intake, the cascade stays down\n");
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

static void b_takes_the_cascade_down_then_intakes(int wiring) {
    Sim s;
    s.wiring = wiring;
    s.raiseTo(1000);
    assert(s.c.target == PtoMode::LIFT && s.c.pushing);
    std::uint32_t pressed = s.now;
    s.intake = 127;  // B with the cascade up
    s.step();
    assert(s.c.phase == PtoControl::LOWERING && s.ptoPower != 127);
    bool allThreeDown = true;
    while (s.c.phase == PtoControl::LOWERING) {
        if (!s.c.pushing) allThreeDown = false;
        s.step();
    }
    std::uint32_t down = s.now - pressed;
    s.settle();
    s.run(50);
    assert(s.c.target == PtoMode::INTAKE && s.where == Sim::ON_INTAKE && s.ptoPower == 127);
    assert(allThreeDown && s.height <= LIFT_BOTTOM + PTO_DOWN_SLACK);
    assert(s.firedAt - s.liftMovedAt >= PTO_SETTLE_MS);
    assert(s.lc.mode == LiftControl::IDLE && !s.lc.ptoTrip);  // no trip left waiting in INTAKE
    s.checks();
    std::printf("  B with the cascade at 1000: down on all three motors in %u ms, intake running "
                "%u ms after the press, piston moved at height %.1f\n",
                static_cast<unsigned>(down), static_cast<unsigned>(s.now - pressed),
                s.highestToIntake);
}

static void r2_on_the_way_down_cancels_and_r1_doesnt() {
    Sim s;
    s.raiseTo(1000);
    s.intake = 127;
    s.run(200);
    assert(s.c.phase == PtoControl::LOWERING && s.height < 1000);
    s.liftCmd = 127;  // R2: the driver wants it up after all
    s.step();
    assert(s.c.target == PtoMode::LIFT && s.c.phase == PtoControl::READY);
    s.run(300);
    assert(s.liftW > 50 && s.pistonMoves == 1);  // only the first shift, into LIFT
    s.liftCmd = 0;
    s.run(200);

    // B again, then R1 on the way down and let go halfway: it carries on down.
    s.intake = 0;
    s.step();
    s.intake = 127;
    s.run(100);
    s.liftCmd = -127;
    s.run(150);
    s.liftCmd = 0;
    assert(s.c.phase == PtoControl::LOWERING && s.height > 100);
    s.settle();
    s.run(50);
    assert(s.c.target == PtoMode::INTAKE && s.ptoPower == 127);
    s.checks();
    std::printf("  R2 on the way down cancels the shift; R1 on the way down doesn't\n");
}

static void b_while_r2_is_held_waits_for_r2() {
    // R2 still held when B is pressed: the driver keeps the cascade until letting
    // go of R2, however long that is, then it comes down and shifts.
    Sim s;
    s.raiseTo(400);
    s.liftCmd = 127;
    s.run(100);
    s.intake = 127;
    s.run(PTO_LOWER_TIMEOUT_MS + 1000);
    assert(s.c.phase == PtoControl::LOWERING && s.c.target == PtoMode::INTAKE);
    assert(s.pistonMoves == 1 && s.c.gaveUpLowering == 0 && s.height > 1000);
    s.liftCmd = 0;
    s.settle();
    s.run(50);
    assert(s.c.target == PtoMode::INTAKE && s.ptoPower == 127 && s.c.gaveUpLowering == 0);
    s.checks();
    std::printf("  B with R2 still held: the cascade stays with R2, comes down once it's let go\n");
}

static void b_then_r2_let_go_while_still_rising(double holdTau, bool sameTick) {
    // R2 from the bottom, B on the way up, then R2 let go while the cascade is
    // still rising fast (or B pressed the same tick R2 is let go). It must coast
    // to a stop before the trip down starts, or the lift's backwards-sensor
    // check sees it go up while sent down.
    Sim s;
    s.holdTau = holdTau;
    s.liftCmd = 127;
    while (s.height < 300) s.step();
    if (!sameTick) {
        s.intake = 127;
        while (s.height < 600) s.step();
    } else {
        while (s.height < 600) s.step();
        s.intake = 127;
    }
    assert(s.liftW > 50);
    s.liftCmd = 0;
    s.settle();
    s.run(50);
    assert(s.c.target == PtoMode::INTAKE && s.ptoPower == 127 && s.c.gaveUpLowering == 0);
    s.checks();
    std::printf("  B and R2 let go at 600 while rising%s (stop in %.2f s): coasted to %.0f, came "
                "down, shifted at %.1f\n",
                sameTick ? " in the same tick" : "", holdTau, s.highestEver, s.highestToIntake);
}

static void b_just_after_a_rise_starts(double holdTau) {
    // R2 from INTAKE, then B the moment the cascade starts moving: still "down"
    // when B lands, but it coasts up off the bottom while stopping, so it has
    // to go back down before the piston moves.
    Sim s;
    s.holdTau = holdTau;
    s.liftCmd = 127;
    // Already moving up, but still inside the bottom slack.
    while (s.height < LIFT_BOTTOM) s.step();
    assert(s.height <= LIFT_BOTTOM + PTO_DOWN_SLACK && s.liftW > 10);
    s.liftCmd = 0;
    s.intake = 127;
    s.step();
    assert(s.c.phase == PtoControl::STOPPING);  // counted as down when B landed
    s.settle();
    s.run(50);
    assert(s.c.target == PtoMode::INTAKE && s.ptoPower == 127);
    // It really did coast up far enough to have to go back down.
    assert(s.highestEver > LIFT_BOTTOM + PTO_DOWN_SLACK + PTO_DOWN_HYSTERESIS);
    s.checks();
    std::printf("  B as a rise passes the bottom zone (stop in %.2f s): coasted to %.0f, came back down, "
                "shifted at %.1f\n",
                holdTau, s.highestEver, s.highestToIntake);
}

static void a_sensor_glitch_on_the_way_down() {
    // One missed lift sensor reading mid-trip mustn't count as "can't tell, so
    // shift now".
    Sim s;
    s.raiseTo(1000);
    s.intake = 127;
    s.run(150);
    assert(s.c.phase == PtoControl::LOWERING && s.height > 500);
    s.glitchAt = s.now;
    for (int i = 0; i < 3; i++) {
        s.step();
        assert(s.c.phase == PtoControl::LOWERING);  // the last good reading still stands
    }
    assert(s.pistonMoves == 1);  // still only the shift into LIFT
    s.settle();
    s.run(50);
    assert(s.c.target == PtoMode::INTAKE && s.ptoPower == 127);
    s.checks();
    std::printf("  one missed sensor reading on the way down: kept going, shifted at %.1f\n",
                s.highestToIntake);
}

static void down_on_the_way_down_stops_the_cascade() {
    Sim s;
    s.raiseTo(1000);
    s.intake = 127;
    s.run(150);
    assert(s.c.phase == PtoControl::LOWERING);
    s.downPress = true;  // changed their mind: stay in LIFT
    s.step();
    assert(s.c.target == PtoMode::LIFT && s.c.phase == PtoControl::READY);
    s.run(100);
    double stopped = s.height;
    s.run(500);
    assert(std::fabs(s.height - stopped) < 5 && s.lc.mode == LiftControl::IDLE && stopped > 500);
    s.checks();
    std::printf("  DOWN on the way down: stays in LIFT, the cascade stops at %.0f\n", stopped);
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

static void resting_right_at_the_edge_doesnt_bounce() {
    // Something stops the cascade just above "down", and the sensor jitters
    // across the line. It must still shift, not go back and forth forever.
    Sim s;
    s.raiseTo(300);
    s.jamHeight = LIFT_BOTTOM + PTO_DOWN_SLACK + 0.2;
    s.heightNoise = 0.3;
    s.intake = 127;
    std::uint32_t ms = s.settle(PTO_LOWER_TIMEOUT_MS + 1500);
    s.run(50);
    assert(s.c.target == PtoMode::INTAKE && s.ptoPower == 127 && s.bounces < 3);
    s.checks();
    std::printf("  stuck just above the bottom with a jittery sensor: shifted in %u ms, %d bounces\n",
                static_cast<unsigned>(ms), s.bounces);
}

static void letting_go_of_r1_on_the_way_down_keeps_going() {
    Sim s;
    s.raiseTo(1000);
    s.intake = 127;
    s.run(100);
    s.liftCmd = -127;
    s.run(150);
    s.liftCmd = 0;
    s.run(20);
    assert(s.c.phase == PtoControl::LOWERING && s.lc.mode == LiftControl::TARGET && s.lc.ptoTrip);
    assert(s.liftW < -20);  // never stopped
    s.settle();
    s.run(50);
    assert(s.c.target == PtoMode::INTAKE && s.ptoPower == 127);
    s.checks();
    std::printf("  R1 let go on the way down: the trip carries straight on\n");
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

static void r1_at_the_bottom_does_nothing() {
    Sim s;
    s.liftCmd = -127;
    s.run(500);
    assert(s.pistonMoves == 0 && s.c.target == PtoMode::INTAKE && s.highestEver < 1);
    s.checks();
    std::printf("  R1 in INTAKE with the cascade down: no shift, nothing moves\n");
}

static void the_piston_waits_for_the_cascade_to_stop() {
    // R1 bringing it down, DOWN pressed just after the lift task sent power. It
    // reaches the bottom moving; the piston waits until it's really held still.
    Sim s;
    s.raiseTo(300);
    s.liftCmd = -127;
    s.run(50);
    s.shiftMidTick = true;
    s.midTickTo = PtoMode::INTAKE;
    s.step();
    assert(s.c.phase == PtoControl::LOWERING);
    s.settle();
    assert(s.firedAt - s.liftMovedAt >= PTO_SETTLE_MS);
    s.liftCmd = 0;
    s.run(100);
    s.checks();
    std::printf("  reaching the bottom moving: piston fired %u ms after the last lift power, "
                "%.1f rpm\n",
                static_cast<unsigned>(s.firedAt - s.liftMovedAt), s.fastestAtFire);
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
    assert(u.c.target == PtoMode::LIFT && u.now - start >= PTO_SHIFT_MS && u.where == Sim::ON_WINCH);
    s.checks();
    t.checks();
    u.checks();
    std::printf("  changing its mind mid-shift ends up where it was told, gear fully in\n");
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

    // Lift motor unplugged: R2 still shifts and lifts; the intake motor never joins.
    Sim t;
    t.liftUnplugged = true;
    t.liftCmd = 127;
    t.run(1500);
    assert(t.c.target == PtoMode::LIFT && !t.c.pushing && t.ptoPower == 0 && !t.c.noMesh);
    // ...and B still brings the cascade down before shifting, with no speed to go by.
    t.liftCmd = 0;
    t.run(200);
    t.intake = 127;
    t.settle();
    t.run(50);
    assert(t.c.target == PtoMode::INTAKE && t.c.gaveUpLowering == 0 && t.ptoPower == 127);
    s.checks();
    t.checks();

    // The next two give up on "all the way down" on purpose (see pto.hpp), so
    // no checks() for them.

    // Lift sensor dead: the height is unknown, so B shifts straight away rather
    // than leave the intake dead for the match.
    Sim u;
    u.raiseTo(800);
    u.sensorBroken = true;
    u.intake = 127;
    u.settle();
    u.run(50);
    assert(u.c.target == PtoMode::INTAKE && u.ptoPower == 127);

    // Something stops the cascade getting down: shifts anyway after the timeout.
    Sim v;
    v.raiseTo(800);
    v.jamHeight = 600;
    v.intake = 127;
    v.settle(PTO_LOWER_TIMEOUT_MS + 1000);
    v.run(50);
    assert(v.c.gaveUpLowering == 1 && v.c.target == PtoMode::INTAKE && v.ptoPower == 127);
    std::printf("  stuck reading: shifted in %u ms; unplugged lift motor: stays out of it; dead "
                "lift sensor or a jam: the intake still works\n",
                static_cast<unsigned>(ms));
}

static void disabled_mid_shift_or_mid_trip_down() {
    Sim s;
    s.c.shift(PtoMode::LIFT);
    s.run(100);
    assert(s.c.phase == PtoControl::SHIFTING);
    s.enabled = false;  // step() checks nothing is sent to the motor meanwhile
    s.run(1000);
    s.enabled = true;
    std::uint32_t start = s.now;
    s.settle();
    assert(s.now - start >= PTO_SHIFT_MS);  // the turn into the teeth ran again
    s.checks();

    // Disabled on the way down: carries on down once enabled, never shifts up high.
    Sim t;
    t.raiseTo(1000);
    t.intake = 127;
    t.run(100);
    assert(t.c.phase == PtoControl::LOWERING);
    t.enabled = false;
    t.run(1000);
    assert(t.c.phase == PtoControl::LOWERING && t.pistonMoves == 1);
    t.enabled = true;
    t.intake = 0;  // pto.cpp dropped B at the disable; the driver presses it again
    t.step();
    t.intake = 127;
    t.settle();
    t.run(50);
    assert(t.c.target == PtoMode::INTAKE && t.ptoPower == 127);
    t.checks();
    std::printf("  disabled mid-shift or on the way down: picks up where it was once enabled\n");
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
    s.liftCmd = 0;
    s.intake = 127;
    s.settle();
    s.run(50);
    assert(!s.c.noMesh && s.c.target == PtoMode::INTAKE && s.ptoPower == 127);
    s.checks();
    std::printf("  piston never moves, R2 at power %d with a %.2f s lift: NOMESH, never pushes; "
                "B still brings it down\n",
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

static void a_random_driver_for_two_minutes(int wiring, double holdTau) {
    Sim s;
    s.wiring = wiring;
    s.holdTau = holdTau;
    s.noise = 3;
    s.seed = 99 + wiring + static_cast<unsigned>(holdTau * 1000);
    int glitches = 0;
    int shifts = 0, ups = 0, intakes = 0;
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
                "%d shifts, never fought, never rose in INTAKE, never shifted to INTAKE above %.1f\n",
                wiring, holdTau, glitches, shifts, s.highestToIntake);
    assert(shifts > 20 && ups > 20 && intakes > 20);
}

int main() {
    starts_in_intake_with_the_cascade_held_down();
    r2_shifts_to_lift_then_lifts(1, false);
    r2_shifts_to_lift_then_lifts(-1, false);
    r2_shifts_to_lift_then_lifts(1, true);
    r2_shifts_to_lift_then_lifts(-1, true);
    b_takes_the_cascade_down_then_intakes(1);
    b_takes_the_cascade_down_then_intakes(-1);
    r2_on_the_way_down_cancels_and_r1_doesnt();
    b_while_r2_is_held_waits_for_r2();
    b_then_r2_let_go_while_still_rising(0.04, false);
    b_then_r2_let_go_while_still_rising(0.1, false);
    b_then_r2_let_go_while_still_rising(0.1, true);
    b_just_after_a_rise_starts(0.04);
    b_just_after_a_rise_starts(0.1);
    a_sensor_glitch_on_the_way_down();
    down_on_the_way_down_stops_the_cascade();
    y_straight_to_b_counts_as_a_press();
    resting_right_at_the_edge_doesnt_bounce();
    letting_go_of_r1_on_the_way_down_keeps_going();
    an_auton_changing_intake_power_isnt_a_press();
    r1_at_the_bottom_does_nothing();
    the_piston_waits_for_the_cascade_to_stop();
    changing_its_mind();
    stuck_missing_or_lying_readings();
    disabled_mid_shift_or_mid_trip_down();
    no_air_shows_nomesh(0.05, 127);
    no_air_shows_nomesh(0.15, 70);
    no_air_shows_nomesh(0.3, 80);
    slipping_out_stops_it_pushing();
    a_random_driver_for_two_minutes(1, 0.04);
    a_random_driver_for_two_minutes(-1, 0.04);
    a_random_driver_for_two_minutes(1, 0.1);
    a_random_driver_for_two_minutes(-1, 0.1);
    std::puts("pto check passes");
}
