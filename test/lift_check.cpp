// Runs the lift logic from lift.hpp against a simulated winch cascade. The
// sensor counts spool turns. Unwinding past the bottom winds the string back on
// the other way and raises the lift again, just like the real one, so the tests
// can check the code never does that. Run test/run_tests.sh.
#include "lift.hpp"
#include <cassert>
#include <cstdio>
#include <initializer_list>

struct Sim {
    double spool0 = -350;          // spool angle with the string fully unwound
    double travel = 1500;          // the top hard stop, in spool degrees above the bottom
    double spool = -350, w = 0;    // spool angle and speed (deg, deg/s)
    int sign = 1;                  // -1 = sensor mounted backwards
    double gravity = 15;           // power it takes to hold the lift up
    bool enabled = true;
    std::uint32_t now = 1;
    double lowestSpool = 1e9;      // lowest spool angle seen, to catch over-unwinding
    LiftControl c;

    double raw() const { return sign * spool; }
    double rpm() const { return w / 6; }
    double liftHeight() const { return std::fabs(spool - spool0); }  // what the lift really does

    void step() {
        int power = c.update(raw(), rpm(), enabled, now);
        if (!enabled) power = 0;
        if (power == 0) {
            w *= 0.5;  // hold brake
        } else {
            // Gravity always pulls the spool back toward fully unwound.
            double pull = spool > spool0 ? gravity : spool < spool0 ? -gravity : 0;
            w += 0.01 * (6 * (power - pull) - w) / 0.08;
        }
        spool += w * 0.01;
        if (liftHeight() >= travel) {  // top hard stop, either winding direction
            spool = spool0 + (spool > spool0 ? travel : -travel);
            w = 0;
        }
        lowestSpool = std::min(lowestSpool, spool);
        now += 10;
    }
    void run(int ms) {
        for (int t = 0; t < ms; t += 10) step();
    }
    void tapDown() {
        c.downButton(true, raw(), now);
        step();
        c.downButton(false, raw(), now);
    }
    double height() const { return c.height(raw()); }
    bool neverUnwoundPastBottom() const { return lowestSpool >= spool0 - 3; }
};

static Sim zeroedAtBottom() {
    Sim s;
    s.c.setZero(s.raw());  // what initLift() does at power-on
    return s;
}

static void lifttop_basics() {
    LiftTop top;
    assert(top.limit(127, 0, 0, 1000) == 127);        // from rest reads 0 rpm for a moment
    assert(top.limit(127, 5, 2, 1100) == 127);
    assert(top.limit(127, 40, 150, 1150) == 127);
    assert(top.limit(-127, 40, 0, 2000) == -127);     // going down never stalls the top
    assert(top.limit(60, 40, 0, 2100) == 60);         // part-power holds aren't a stall either
    assert(top.limit(60, 40, 0, 2600) == 60);
    assert(!top.found);
    assert(top.limit(127, 900, 1, 3000) == 127);
    assert(top.limit(127, 900, 1, 3000 + LIFT_STALL_MS) == 0);
    assert(top.found && top.position == 900);
    assert(top.limit(127, 100, 150, 5000) == 127);
    assert(top.limit(127, 900 - LIFT_SLOW_ZONE + 1, 150, 5010) == LIFT_SLOW_POWER);
    assert(top.limit(127, 900 - LIFT_TOP_MARGIN, 30, 5020) == 0);
    assert(top.limit(-127, 900, 0, 5040) == -127);
}

static void presets_and_the_bottom_limit() {
    Sim s = zeroedAtBottom();

    // Presets wait while the robot is disabled.
    s.enabled = false;
    s.c.nextPreset(s.raw(), s.now);
    s.run(1000);
    assert(s.liftHeight() == 0);
    s.enabled = true;
    s.run(3000);
    assert(std::fabs(s.height() - LIFT_PRESETS[0]) <= 3);
    std::printf("  preset %.0f -> %.1f (after waiting out disabled)\n", LIFT_PRESETS[0], s.liftHeight());

    // Each further L1 press: the next preset up, and it stays there.
    for (double preset : {LIFT_PRESETS[1], LIFT_PRESETS[2]}) {
        s.c.nextPreset(s.raw(), s.now);
        s.run(3000);
        assert(s.c.mode == LiftControl::IDLE);
        assert(std::fabs(s.liftHeight() - preset) <= 3);
        s.run(500);
        assert(std::fabs(s.liftHeight() - preset) <= 3);  // and it holds there
        std::printf("  preset %.0f -> %.1f\n", preset, s.liftHeight());
    }
    s.c.nextPreset(s.raw(), s.now);  // past the last preset, L1 does nothing
    assert(s.c.mode == LiftControl::IDLE);

    // Holding R1 all the way down stops at the bottom instead of unwinding past it.
    s.c.manual(-127);
    s.run(5000);
    s.c.manual(0);
    assert(s.neverUnwoundPastBottom() && s.liftHeight() <= LIFT_BOTTOM + 3);
    std::printf("  R1 held for 5 s: stopped %.1f above the bottom\n", s.liftHeight());

    // Tap L2 from the top preset: down to the bottom and stops.
    s.c.nextPreset(s.raw(), s.now);
    s.c.nextPreset(s.raw(), s.now);
    s.c.nextPreset(s.raw(), s.now);
    s.run(4000);
    assert(std::fabs(s.liftHeight() - LIFT_PRESETS[2]) <= 3);
    s.tapDown();
    s.run(4000);
    assert(s.neverUnwoundPastBottom() && s.liftHeight() <= LIFT_BOTTOM + 3);
    std::printf("  L2 tap from %.0f: stopped %.1f above the bottom\n", LIFT_PRESETS[2], s.liftHeight());

    // Holding R1 or tapping L2 again at the bottom does nothing.
    s.tapDown();
    s.c.manual(-127);
    s.run(1000);
    s.c.manual(0);
    assert(s.neverUnwoundPastBottom());

    // Hand control takes over from a preset move.
    s.c.nextPreset(s.raw(), s.now);
    s.c.manual(127);
    assert(s.c.mode == LiftControl::MANUAL);
    s.c.manual(0);
    assert(s.c.mode == LiftControl::IDLE);
}

static void rezero_when_started_with_the_lift_up() {
    Sim s;
    s.spool = s.spool0 + 400;  // program started with the lift up
    s.c.setZero(s.raw());      // so it thinks this is the bottom
    assert(std::fabs(s.height()) < 1 && s.liftHeight() == 400);

    // Hold L2: it can't go below its wrong 0 normally, so after the hold time it
    // creeps down past it. The driver lets go when the lift touches the bottom.
    s.c.downButton(true, s.raw(), s.now);
    std::uint32_t held = 0;
    while (s.liftHeight() > 2 && held < 30000) {
        s.c.downButton(true, s.raw(), s.now);
        s.step();
        held += 10;
    }
    s.c.downButton(false, s.raw(), s.now);
    assert(s.c.zeroed && std::fabs(s.height() - s.liftHeight()) < 5);
    std::printf("  started 400 up: held L2 %.1f s, zero now off by %.1f\n", held / 1000.0,
                std::fabs(s.height() - s.liftHeight()));

    // After that, presets land on the real heights.
    s.c.nextPreset(s.raw(), s.now);
    s.run(3000);
    assert(std::fabs(s.liftHeight() - LIFT_PRESETS[0]) <= 5);
}

static void preset_above_the_top() {
    // Hits the top while pushing hard: learns it and stops right there.
    Sim s = zeroedAtBottom();
    s.travel = 650;
    for (int i = 0; i < 3; i++) {
        s.c.nextPreset(s.raw(), s.now);
        s.run(3000);
    }
    assert(s.c.top.found && s.c.mode == LiftControl::IDLE);
    assert(s.c.update(s.raw(), s.rpm(), true, s.now) == 0);  // not grinding
    std::printf("  preset far above the top: learned it at %.0f and stopped\n", s.height());

    // Reaches the top only a little short of the preset, pushing gently: gives
    // up after the timeout and says why, without blaming the sensor.
    Sim t = zeroedAtBottom();
    t.travel = 750;
    for (int i = 0; i < 3; i++) {
        t.c.nextPreset(t.raw(), t.now);
        t.run(LIFT_MOVE_TIMEOUT_MS + 500);
    }
    assert(t.c.mode == LiftControl::IDLE && t.c.stoppedShort > 0 && !t.c.backwards);
    std::printf("  preset just above the top: gave up %.0f short\n", t.c.stoppedShort);
}

static void too_heavy_stops_short_and_says_so() {
    Sim s = zeroedAtBottom();
    s.c.nextPreset(s.raw(), s.now);  // first preset while light
    s.run(3000);
    s.gravity = LIFT_MIN_POWER + 5;  // now carrying more than the creep power can lift
    s.c.nextPreset(s.raw(), s.now);
    s.run(LIFT_MOVE_TIMEOUT_MS + 500);
    // It gives up and says so, without inventing a top or blaming the sensor.
    assert(s.c.mode == LiftControl::IDLE && s.c.stoppedShort > 0);
    assert(!s.c.top.found && !s.c.backwards && s.neverUnwoundPastBottom());
    std::printf("  too heavy: gave up %.0f short, no false top or backwards flag\n", s.c.stoppedShort);
}

static void backwards_sensor_is_caught() {
    Sim s;
    s.sign = -1;
    s.c.setZero(s.raw());
    s.c.nextPreset(s.raw(), s.now);
    s.run(2000);
    assert(s.c.backwards && s.c.mode == LiftControl::IDLE);
    assert(s.neverUnwoundPastBottom() && s.liftHeight() < 100);
    // With a backwards sensor, presets and L2 refuse to run.
    s.c.nextPreset(s.raw(), s.now);
    s.tapDown();
    assert(s.c.mode == LiftControl::IDLE);
    std::printf("  backwards sensor caught on the first preset, %.0f degrees up\n", s.liftHeight());
}

int main() {
    lifttop_basics();
    presets_and_the_bottom_limit();
    rezero_when_started_with_the_lift_up();
    preset_above_the_top();
    too_heavy_stops_short_and_says_so();
    backwards_sensor_is_caught();
    std::puts("lift check passes");
}
