#pragma once

struct Auton {
    const char* name;
    void (*run)();
};

extern const Auton autons[];
extern const int autonCount;
extern int selectedAuton;  // changed by the brain screen selector

// Runs whichever auton is selected. Called from autonomous().
void runAuton();
