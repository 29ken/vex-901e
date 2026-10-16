// Runs the lift logic from lift.hpp against a simulated cascade: gravity,
// hard stops at both ends, a hold brake, and a sensor that reads an arbitrary
// number at power-on. Run test/run_tests.sh.
#include "lift.hpp"
#include <cassert>
#include <cstdio>

struct Sim {
    double bottom = -350, top = 1150;  // where the hard stops are, in raw sensor degrees
    double p = 0, v = 0;               // position and speed, deg and deg/s
    int sign = 1;                      // -1 = sensor mounted backwards
    bool enabled = true;
    double gravity = 15;               // power it takes to hold the lift up
    std::uint32_t now = 1;
    LiftControl c;

    double raw() const { return sign * p; }
    double rpm() const { return v / 6; }

    void step(int ms = 10) {
        for (int i = 0; i < ms; i += 10) {
            int power = c.update(raw(), rpm(), enabled, now);
            if (!enabled) power = 0;  // motors ignore commands while disabled
            if (power == 0) {
                v *= 0.5;  // hold brake
            } else {
                v += 0.01 * (6 * (power - gravity) - v) / 0.08;
            }
            p += v * 0.01;
            if (p <= bottom) { p = bottom; v = 0; }
            if (p >= top) { p = top; v = 0; }
            now += 10;
        }
    }
    void run(int ms) { step(ms); }
    double height() const { return c.height(raw()); }
};

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

static void homes_then_reaches_presets() {
    Sim s;
    s.p = s.bottom + 700;  // lift left halfway up; the sensor doesn't know where that is

    // Homing waits while the robot is disabled instead of mistaking "not moving" for the bottom.
    s.enabled = false;
    s.c.down(s.raw(), s.now);
    s.run(1000);
    assert(!s.c.homed && s.p == s.bottom + 700);

    s.enabled = true;
    s.run(4000);
    assert(s.c.homed && s.c.mode == LiftControl::IDLE);
    assert(std::fabs(s.height()) < 1);
    std::printf("  homed from halfway up\n");

    // Each L1 press: the next preset up, and it stays there.
    for (double preset : LIFT_PRESETS) {
        s.c.nextPreset(s.raw(), s.now);
        s.run(3000);
        assert(s.c.mode == LiftControl::IDLE);
        assert(std::fabs(s.height() - preset) <= 3);
        s.run(500);
        assert(std::fabs(s.height() - preset) <= 3);  // and it holds there
        std::printf("  preset %.0f -> %.1f\n", preset, s.height());
    }
    // Past the last preset, L1 does nothing.
    s.c.nextPreset(s.raw(), s.now);
    assert(s.c.mode == LiftControl::IDLE);

    // L2 takes it back down and re-zeros, even if the sensor had drifted.
    s.c.zero += 30;
    s.c.down(s.raw(), s.now);
    s.run(4000);
    assert(s.c.homed && std::fabs(s.height()) < 1 && s.p == s.bottom);
    std::printf("  L2 re-homed after 30 degrees of drift\n");

    // Pressing L1 twice quickly skips ahead to the second preset.
    s.c.nextPreset(s.raw(), s.now);
    s.run(50);
    s.c.nextPreset(s.raw(), s.now);
    assert(s.c.target == LIFT_PRESETS[1]);
    s.run(3000);
    assert(std::fabs(s.height() - LIFT_PRESETS[1]) <= 3);

    // Hand control takes over from a preset move.
    s.c.nextPreset(s.raw(), s.now);
    s.c.manual(-127);
    assert(s.c.mode == LiftControl::MANUAL);
    s.c.manual(0);
    assert(s.c.mode == LiftControl::IDLE);
}

static void preset_above_the_top() {
    // Hits the top while pushing hard: learns it and stops right there.
    Sim s;
    s.top = s.bottom + 650;
    s.c.down(s.raw(), s.now);
    s.run(3000);
    s.c.nextPreset(s.raw(), s.now);
    s.run(3000);
    s.c.nextPreset(s.raw(), s.now);
    s.run(3000);
    s.c.nextPreset(s.raw(), s.now);  // 900, well above the top
    s.run(1000);
    assert(s.c.top.found && s.c.mode == LiftControl::IDLE);
    assert(s.c.update(s.raw(), s.rpm(), true, s.now) == 0);  // not grinding
    std::printf("  preset far above the top: learned it at %.0f and stopped\n", s.height());

    // Reaches the top only a little short of the preset, pushing gently: gives
    // up after the timeout and says why, without blaming the sensor.
    Sim t;
    t.top = t.bottom + 750;
    t.c.down(t.raw(), t.now);
    t.run(3000);
    for (int i = 0; i < 3; i++) {
        t.c.nextPreset(t.raw(), t.now);
        t.run(LIFT_MOVE_TIMEOUT_MS + 500);
    }
    assert(t.c.mode == LiftControl::IDLE && t.c.stoppedShort > 0 && !t.c.backwards);
    std::printf("  preset just above the top: gave up %.0f short\n", t.c.stoppedShort);
}

static void too_heavy_stops_short_and_says_so() {
    Sim s;
    s.c.down(s.raw(), s.now);
    s.run(3000);
    s.c.nextPreset(s.raw(), s.now);  // first preset while light
    s.run(3000);
    s.gravity = LIFT_MIN_POWER + 5;  // now carrying more than the creep power can lift
    s.c.nextPreset(s.raw(), s.now);
    s.run(LIFT_MOVE_TIMEOUT_MS + 500);
    // It gives up and says so, without inventing a top or blaming the sensor.
    assert(s.c.mode == LiftControl::IDLE && s.c.stoppedShort > 0);
    assert(!s.c.top.found && !s.c.backwards);
    std::printf("  too heavy: gave up %.0f short, no false top or backwards flag\n", s.c.stoppedShort);
}

static void backwards_sensor_is_caught() {
    Sim s;
    s.sign = -1;
    s.p = s.bottom + 700;
    s.c.down(s.raw(), s.now);
    s.run(3000);
    assert(s.c.backwards && s.c.mode == LiftControl::IDLE && !s.c.homed);
    // With a backwards sensor, presets refuse to run.
    s.c.nextPreset(s.raw(), s.now);
    assert(s.c.mode == LiftControl::IDLE);
    std::printf("  backwards sensor caught while homing, after %.0f degrees\n", s.bottom + 700 - s.p);

    // Starting at the bottom, homing can't tell, so the first preset has to.
    Sim t;
    t.sign = -1;
    t.p = t.bottom;
    t.c.down(t.raw(), t.now);
    t.run(3000);
    assert(t.c.homed);
    t.c.nextPreset(t.raw(), t.now);
    t.run(2000);
    assert(t.c.backwards && t.c.mode == LiftControl::IDLE);
    assert(t.p - t.bottom < 100);  // stopped long before the top
    std::printf("  backwards sensor caught on the first preset, %.0f degrees up\n", t.p - t.bottom);
}

int main() {
    lifttop_basics();
    homes_then_reaches_presets();
    preset_above_the_top();
    too_heavy_stops_short_and_says_so();
    backwards_sensor_is_caught();
    std::puts("lift check passes");
}
