// Checks LiftTop's max height logic off the robot. Run test/run_tests.sh.
#include "lift.hpp"
#include <cassert>
#include <cstdio>

int main() {
    LiftTop top;

    // Starting from rest reads ~0 rpm for a moment; that isn't a stall.
    assert(top.limit(127, 0, 0, 1000) == 127);
    assert(top.limit(127, 5, 2, 1100) == 127);
    assert(top.limit(127, 40, 150, 1150) == 127);
    assert(!top.found);

    // Going down or holding never counts as a stall.
    assert(top.limit(-127, 40, 0, 2000) == -127);
    assert(top.limit(0, 40, 0, 2500) == 0);
    assert(!top.found);

    // Pushing up against the stop: after LIFT_STALL_MS it learns the top and cuts power.
    assert(top.limit(127, 900, 1, 3000) == 127);
    assert(top.limit(127, 900, 1, 3000 + LIFT_STALL_MS - 10) == 127);
    assert(top.limit(127, 900, 1, 3000 + LIFT_STALL_MS) == 0);
    assert(top.found && top.position == 900);

    // From then on: full speed far below, slow near the top, stop just under it.
    assert(top.limit(127, 100, 150, 5000) == 127);
    assert(top.limit(127, 900 - LIFT_SLOW_ZONE + 1, 150, 5010) == LIFT_SLOW_POWER);
    assert(top.limit(127, 900 - LIFT_TOP_MARGIN, 30, 5020) == 0);
    assert(top.limit(127, 950, 0, 5030) == 0);
    // Going down from the top still works.
    assert(top.limit(-127, 900, 0, 5040) == -127);

    std::puts("lift check passes");
}
