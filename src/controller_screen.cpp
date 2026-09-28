#include "screens.hpp"
#include "main.h"
#include "autons.hpp"
#include "config.hpp"
#include "controls.hpp"
#include "drivetrain.hpp"
#include "pid_tuner.hpp"
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <string>

// V5 motors start cutting their own power at 55C.
static constexpr double HOT_TEMP = 55.0;
static constexpr double COOL_TEMP = 25.0;  // where the heat bar starts filling
static constexpr std::uint32_t DRIVER_MS = 105000;
static constexpr std::uint32_t AUTON_MS = 15000;
static constexpr std::uint32_t SPLASH_MS = 1200;
static constexpr std::uint32_t MENU_TIMEOUT_MS = 5000;

// The only buttons the menu touches. Everything else is free for mechanisms.
static constexpr auto BTN_MENU  = pros::E_CONTROLLER_DIGITAL_X;
static constexpr auto BTN_NEXT  = pros::E_CONTROLLER_DIGITAL_UP;
static constexpr auto BTN_PLUS  = pros::E_CONTROLLER_DIGITAL_A;
static constexpr auto BTN_MINUS = pros::E_CONTROLLER_DIGITAL_LEFT;

enum class Phase { Splash, PreMatch, Auton, Driver };

enum MenuItem { MENU_AUTON, MENU_BRAKES, MENU_TIMER, MENU_TURN_PID, MENU_MOVE_PID, MENU_ITEMS };
static const char* MENU_NAMES[] = {"Auton", "Brakes", "Timer", "Turn PID", "Move PID"};

struct Health {
    int plugged = 0;
    int total = 0;
    int deadPort = 0;
    MotorTemp hottest{0, 0, true};
};

// What each line should say vs. what the controller is showing. Only lines
// that differ get sent, since the radio takes one line per 50 ms.
static std::string wanted[3];
static std::string shown[3];
static const char* queuedRumble = nullptr;

static std::uint32_t phaseStart = 0;
static std::uint32_t lastLeft = DRIVER_MS;

static bool menuOpen = false;
static int menuItem = MENU_AUTON;
static std::uint32_t menuTouched = 0;
static bool brakeHold = false;

static void setLine(int line, const char* fmt, ...) {
    char text[16];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);
    wanted[line] = text;
}

static void rumble(const char* pattern) {
    if (!queuedRumble) queuedRumble = pattern;
}

static bool blink(std::uint32_t now, std::uint32_t period = 500) {
    return now % period < period / 2;
}

static std::string bar(double fraction, int width) {
    int filled = static_cast<int>(std::clamp(fraction, 0.0, 1.0) * width + 0.5);
    return std::string(filled, '#') + std::string(width - filled, '-');
}

static std::string center(const char* text, int width) {
    std::string s(text);
    if (static_cast<int>(s.size()) >= width) return s.substr(0, width);
    int left = (width - static_cast<int>(s.size())) / 2;
    return std::string(left, ' ') + s + std::string(width - s.size() - left, ' ');
}

// Two blocks bouncing back and forth, like a scanner.
static std::string scanner(std::uint32_t now) {
    constexpr int width = 8, block = 2, steps = width - block;
    int t = (now / 100) % (steps * 2);
    int pos = t < steps ? t : steps * 2 - t;
    std::string s(width, ' ');
    s.replace(pos, block, block, '#');
    return s;
}

static Health checkMotors() {
    Health h;
    for (pros::MotorGroup* group : {&leftMotors, &rightMotors}) {
        for (MotorTemp m : motorTemps(*group)) {
            h.total++;
            if (!m.plugged) {
                h.deadPort = m.port;
                continue;
            }
            h.plugged++;
            if (m.celsius > h.hottest.celsius) h.hottest = m;
        }
    }
    return h;
}

static void drawSplash(std::uint32_t now) {
    setLine(0, "     901E");
    setLine(1, " [%s]", bar(now / static_cast<double>(SPLASH_MS), 11).c_str());
    setLine(2, "");
}

static void drawTuneLine(TuneKind kind, const TuneResult& r, Phase phase, std::uint32_t now) {
    if (tuning == kind) {
        setLine(2, "TUNE [%s]", scanner(now).c_str());
    } else if (r.ok) {
        setLine(2, "P%.2f  D%.1f", r.kP, r.kD);
    } else if (r.done) {
        setLine(2, "FAIL %s", r.error);
    } else if (phase != Phase::Driver) {
        setLine(2, "%s", center("enable robot", 15).c_str());
    } else {
        setLine(2, "%s", center("A: auto-tune", 15).c_str());
    }
}

static void drawMenu(Phase phase, std::uint32_t now) {
    setLine(0, "MENU %d/%d  X:out", menuItem + 1, MENU_ITEMS);
    setLine(1, "%-8sUP:next", MENU_NAMES[menuItem]);
    switch (menuItem) {
        case MENU_AUTON:
            setLine(2, "<%s>", center(autons[selectedAuton].name, 13).c_str());
            break;
        case MENU_BRAKES:
            setLine(2, "<%s>", center(brakeHold ? "HOLD" : "COAST", 13).c_str());
            break;
        case MENU_TIMER:
            setLine(2, "%s", center("A: restart", 15).c_str());
            break;
        case MENU_TURN_PID:
            drawTuneLine(TUNE_TURN, turnTune, phase, now);
            break;
        case MENU_MOVE_PID:
            drawTuneLine(TUNE_MOVE, moveTune, phase, now);
            break;
    }
}

static void drawPreMatch(const char* auton, double battery, const Health& h, std::uint32_t now) {
    setLine(0, "901E   BAT %3.0f%%", battery);
    setLine(1, "<%s>", center(auton, 13).c_str());
    if (h.deadPort) {
        setLine(2, "%s PORT %d DEAD", blink(now) ? "!!" : "  ", h.deadPort);
    } else {
        setLine(2, "MOTORS %d/%d OK", h.plugged, h.total);
    }
}

static void drawAuton(const char* auton, std::uint32_t elapsed, std::uint32_t now) {
    setLine(0, "AUTO [%s]", scanner(now).c_str());
    setLine(1, "%s", center(auton, 15).c_str());
    setLine(2, "%2lus [%s]", static_cast<unsigned long>(elapsed / 1000),
            bar(elapsed / static_cast<double>(AUTON_MS), 9).c_str());
}

static void drawDriver(std::uint32_t elapsed, double battery, const Health& h, std::uint32_t now) {
    if (elapsed < DRIVER_MS) {
        std::uint32_t left = (DRIVER_MS - elapsed + 999) / 1000;
        std::string timeBar = bar(left * 1000.0 / DRIVER_MS, 8);
        // Last 15 seconds: the clock flashes.
        if (left <= 15 && !blink(now)) {
            setLine(0, "     [%s]", timeBar.c_str());
        } else {
            setLine(0, "%lu:%02lu [%s]", static_cast<unsigned long>(left / 60),
                    static_cast<unsigned long>(left % 60), timeBar.c_str());
        }
    } else {
        std::uint32_t over = (elapsed - DRIVER_MS) / 1000;
        setLine(0, "OVERTIME +%lu:%02lu", static_cast<unsigned long>(over / 60),
                static_cast<unsigned long>(over % 60));
    }

    if (battery < 25 && !blink(now)) {
        setLine(1, "BAT LOW  %3.0f%%", battery);
    } else {
        setLine(1, "BAT %3.0f%% [%s]", battery, bar(battery / 100, 4).c_str());
    }

    const char* alert = blink(now) ? "!!" : "  ";
    if (h.deadPort) {
        setLine(2, "%s PORT %d DEAD", alert, h.deadPort);
    } else if (h.hottest.celsius >= HOT_TEMP) {
        setLine(2, "%s M%d HOT %.0fC", alert, h.hottest.port, h.hottest.celsius);
    } else {
        double heat = (h.hottest.celsius - COOL_TEMP) / (HOT_TEMP - COOL_TEMP);
        setLine(2, "M%-2d %2.0fC [%s]", h.hottest.port, h.hottest.celsius, bar(heat, 5).c_str());
    }
}

static void send() {
    if (queuedRumble) {
        controller.rumble(queuedRumble);
        queuedRumble = nullptr;
        return;
    }
    static int nextLine = 0;
    for (int i = 0; i < 3; i++) {
        int line = (nextLine + i) % 3;
        if (wanted[line] == shown[line]) continue;
        controller.print(line, 0, "%-15s", wanted[line].c_str());
        shown[line] = wanted[line];
        nextLine = (line + 1) % 3;
        return;
    }
}

static Phase currentPhase(std::uint32_t sinceBoot) {
    if (sinceBoot < SPLASH_MS) return Phase::Splash;
    if (pros::competition::is_disabled()) return Phase::PreMatch;
    if (pros::competition::is_autonomous()) return Phase::Auton;
    return Phase::Driver;
}

// A steps forward, LEFT steps back.
static void change(int item, int step, Phase phase, std::uint32_t now) {
    if (item == MENU_TURN_PID || item == MENU_MOVE_PID) {
        // Motors ignore commands while disabled, so a tune would just time out.
        if (step < 0 || phase != Phase::Driver || tuning != TUNE_NONE) return;
        item == MENU_TURN_PID ? startTurnTune() : startMoveTune();
        rumble(".");
        return;
    }

    switch (item) {
        case MENU_AUTON:
            selectedAuton = (selectedAuton + step + autonCount) % autonCount;
            break;
        case MENU_BRAKES:
            brakeHold = !brakeHold;
            chassis.setBrakeMode(brakeHold ? pros::E_MOTOR_BRAKE_HOLD : pros::E_MOTOR_BRAKE_COAST);
            break;
        case MENU_TIMER:
            phaseStart = now;
            lastLeft = DRIVER_MS;
            break;
    }
    rumble(".");
}

static void handleButtons(Phase phase, std::uint32_t now) {
    bool menu  = controller.get_digital_new_press(BTN_MENU);
    bool next  = controller.get_digital_new_press(BTN_NEXT);
    bool plus  = controller.get_digital_new_press(BTN_PLUS);
    bool minus = controller.get_digital_new_press(BTN_MINUS);

    if (menu) {
        menuOpen = !menuOpen;
        menuItem = MENU_AUTON;
    }

    if (menuOpen) {
        if (next) menuItem = (menuItem + 1) % MENU_ITEMS;
        if (plus) change(menuItem, +1, phase, now);
        if (minus) change(menuItem, -1, phase, now);
        if (menu || next || plus || minus || tuning != TUNE_NONE) menuTouched = now;
        // Never leave the menu covering the match clock.
        if (now - menuTouched > MENU_TIMEOUT_MS) menuOpen = false;
    } else if (phase == Phase::PreMatch) {
        // Before a match the auton is the only thing worth changing, so skip the menu.
        if (plus) change(MENU_AUTON, +1, phase, now);
        if (minus) change(MENU_AUTON, -1, phase, now);
    }
}

static void loop() {
    std::uint32_t boot = pros::millis();
    Phase phase = Phase::Splash;
    phaseStart = boot;
    bool warnedHot = false;
    int warnedDead = 0;

    while (true) {
        std::uint32_t now = pros::millis();

        // After a radio dropout the screen is blank, so redraw everything.
        if (!controller.is_connected()) {
            for (std::string& s : shown) s = "?";
            pros::delay(50);
            continue;
        }

        Phase next = currentPhase(now - boot);
        if (next != phase) {
            phase = next;
            phaseStart = now;
            lastLeft = DRIVER_MS;
            menuOpen = false;
        }
        if (phase != Phase::Splash) handleButtons(phase, now);
        std::uint32_t elapsed = now - phaseStart;
        Health h = checkMotors();
        double battery = pros::battery::get_capacity();
        const char* auton = autons[selectedAuton].name;

        switch (phase) {
            case Phase::Splash:
                drawSplash(now - boot);
                break;

            case Phase::PreMatch:
                drawPreMatch(auton, battery, h, now);
                break;

            case Phase::Auton:
                drawAuton(auton, elapsed, now);
                break;

            case Phase::Driver: {
                std::uint32_t left = elapsed < DRIVER_MS ? DRIVER_MS - elapsed : 0;
                if (lastLeft > 30000 && left <= 30000) rumble("-");
                if (lastLeft > 15000 && left <= 15000) rumble("- -");
                if (lastLeft > 0 && left == 0) rumble("---");
                lastLeft = left;

                if (h.hottest.celsius >= HOT_TEMP && !warnedHot) {
                    rumble(". . .");
                    warnedHot = true;
                } else if (h.hottest.celsius < HOT_TEMP - 5) {
                    warnedHot = false;
                }
                if (h.deadPort && h.deadPort != warnedDead) rumble("...");
                warnedDead = h.deadPort;

                drawDriver(elapsed, battery, h, now);
                break;
            }
        }
        if (menuOpen) drawMenu(phase, now);

        static TuneKind lastTuning = TUNE_NONE;
        if (lastTuning == TUNE_TURN && tuning == TUNE_NONE) rumble(turnTune.ok ? ". ." : "-");
        if (lastTuning == TUNE_MOVE && tuning == TUNE_NONE) rumble(moveTune.ok ? ". ." : "-");
        lastTuning = tuning;

        send();
        pros::delay(50);
    }
}

void startControllerScreen() {
    if (!CONTROLLER_UI) return;
    pros::Task task(loop, "Controller screen");
}
