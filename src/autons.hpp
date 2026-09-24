#pragma once

namespace autons {

struct Routine {
    const char* name;
    void (*run)();
};

extern const Routine list[];
extern const int count;
extern int selected;  // changed by the brain screen selector

// Runs whichever routine is selected. Called from autonomous().
void run();

} // namespace autons
