#pragma once

struct Auton {
    const char* name;
    void (*run)();
};

extern const Auton autons[];
extern const int autonCount;
extern int selectedAuton;  // changed by the brain screen selector

// Runs whichever auton is selected. Called from autonomous(). The routes and
// the list are in src/auton/autonroutes.cpp.
void runAuton();

#ifdef RW_SIM_PROS
// Simulator only (sh sim/run-sim.sh builds with RW_SIM_PROS). The simulator
// finds autons by name in RW's own list, which the robot doesn't use, so this
// copies ours into it, plus a few sim-only checks. In src/auton/sim_checks.cpp.
void registerSimAutons();
#endif
