#include "screens.hpp"
#include "main.h"
#include "autons.hpp"
#include "config.hpp"
#include "drivetrain.hpp"
#include "lift.hpp"
#include "tuner.hpp"
#include "resets.hpp"
#include "rw-config.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>

// The middle button switches between the main page and the Distance page.
enum class Page { Main, Distance };
static Page page = Page::Main;

// ---- Distance page --------------------------------------------------------
// Each distance sensor's reading, its scale for the tape distance, and what a
// reset would do with it right now. Left/right set the tape distance.
static int tapeInches = 24;
static constexpr std::size_t AVERAGE_OVER = 40;  // readings, one per 50 ms loop = 2 s
static constexpr ResetSensor SENSORS[] = {ResetSensor::Front, ResetSensor::Back, ResetSensor::Left,
                                          ResetSensor::Right};
static constexpr const char* SCALE_NAMES[] = {"SCALE_FRONT", "SCALE_BACK", "SCALE_LEFT", "SCALE_RIGHT"};
static std::deque<double> recent[4];  // raw readings, inches
static double printedScale[4] = {0, 0, 0, 0};

static double mean(const std::deque<double>& r) {
    double sum = 0;
    for (double v : r) sum += v;
    return r.empty() ? 0 : sum / r.size();
}

// The scale for this sensor once it has sat near the tape distance for 2 s,
// with the robot still and square to the wall, else 0.
static double scaleFor(int i, double rawIn) {
    std::deque<double>& r = recent[i];
    bool still = std::fabs(odom_speed_ips) < 0.5 && std::fabs(odom_turn_rate_dps) < 2;
    bool square = std::fabs(std::remainder(getInertialHeading(), 90.0)) < 3;
    // Start over when the robot moves or turns, or the sensor loses the wall.
    if (!still || !square || rawIn <= 0 || (!r.empty() && std::fabs(rawIn - mean(r)) > 2)) r.clear();
    if (still && square && rawIn > 0) r.push_back(rawIn);
    if (r.size() > AVERAGE_OVER) r.pop_front();
    if (r.size() < AVERAGE_OVER) return 0;

    double average = mean(r), squares = 0;
    for (double v : r) squares += (v - average) * (v - average);
    bool steady = std::sqrt(squares / r.size()) < 0.75;  // something moving in the beam if not
    // Only the sensor parked at the tape distance: real sensors are off by 1-3%.
    bool nearTape = std::fabs(average - tapeInches) < 0.05 * tapeInches;
    return steady && nearTape ? tapeInches / average : 0;
}

void showDistancePage(int tape) {
    tapeInches = tape;
    page = Page::Distance;
}

static void drawDistance() {
    pros::lcd::print(0, "DISTANCE  tape %d in (L/R)  mid: back", tapeInches);
    for (int i = 0; i < 4; i++) {
        ResetPreview p = previewReset(SENSORS[i]);
        double scale = scaleFor(i, p.installed ? p.rawIn : -1);
        char reading[12], scaleText[10], result[24];
        if (!p.installed) {
            std::snprintf(reading, sizeof(reading), "unplgd");
        } else if (!p.seesWall) {
            std::snprintf(reading, sizeof(reading), "  --  ");
        } else {
            std::snprintf(reading, sizeof(reading), "%5.1fin", p.rawIn);
        }
        if (scale > 0) {
            std::snprintf(scaleText, sizeof(scaleText), "x%.3f", scale);
        } else {
            std::snprintf(scaleText, sizeof(scaleText), "x  -  ");
        }
        if (p.ok) {
            std::snprintf(result, sizeof(result), "%c->%.2f", p.axis, p.newPos);
        } else {
            std::snprintf(result, sizeof(result), "%s", p.why);
        }
        pros::lcd::print(i + 1, "%-5s %s %s %s", resetSensorName(SENSORS[i]), reading, scaleText, result);

        // Also to the terminal, ready to paste, when a sensor's scale settles on a
        // new value (at most every 2 s per sensor).
        static std::uint32_t printedAt[4] = {0, 0, 0, 0};
        std::uint32_t now = pros::millis();
        if (scale > 0 && std::fabs(scale - printedScale[i]) > 0.003 && now - printedAt[i] > 2000) {
            std::printf("inline constexpr double %s = %.4f;  // read %.2f in at a %d in tape distance\n",
                        SCALE_NAMES[i], scale, tapeInches / scale, tapeInches);
            printedScale[i] = scale;
            printedAt[i] = now;
        }
    }
    pros::lcd::print(5, "X %.2f  Y %.2f  H %.1f", x_pos, y_pos, getInertialHeading());
    pros::lcd::print(6, "Park one lens the tape distance from a");
    pros::lcd::print(7, "wall, square; its x = scale (resets.hpp)");
}

static void printSide(int line, const char* label, pros::MotorGroup& motors) {
    char row[48];
    int n = std::snprintf(row, sizeof(row), "%s", label);
    for (MotorTemp m : motorTemps(motors)) {
        if (n >= static_cast<int>(sizeof(row))) break;
        if (m.plugged) {
            n += std::snprintf(row + n, sizeof(row) - n, " %d:%.0f", m.port, m.celsius);
        } else {
            n += std::snprintf(row + n, sizeof(row) - n, " %d:--", m.port);
        }
    }
    pros::lcd::print(line, "%s", row);
}

static void draw() {
    if (page == Page::Distance) {
        drawDistance();
        return;
    }
    pros::lcd::print(0, "Auton: %s  (%d/%d)", autons[selectedAuton].name,
                     selectedAuton + 1, autonCount);
    pros::lcd::print(1, "left/right: auton   mid: distance sensors");

    if (!BRAIN_UI) return;

    printSide(3, "L", left_chassis);
    printSide(4, "R", right_chassis);
    
    // Raise the lift by hand to a pin or cone and read this number for LIFT_PRESETS.
    LiftStatus l = liftStatus();
    char liftText[32];
    if (!l.sensorOk) {
        std::snprintf(liftText, sizeof(liftText), "Lift sensor missing");
    } else if (l.backwards) {
        std::snprintf(liftText, sizeof(liftText), "Lift sensor backwards");
    } else {
        std::snprintf(liftText, sizeof(liftText), "Lift %.0f%s", l.height, l.zeroed ? "" : " (restart lift down)");
    }
    pros::lcd::print(5, "Battery %.0f%%   %s", pros::battery::get_capacity(), liftText);

    // RW-Template odometry coordinates and heading
    pros::lcd::print(6, "X %.1f  Y %.1f  H %.1f", x_pos, y_pos, getInertialHeading());

    // Tuned values only last until reboot, so keep them up where they're easy to copy.
    if (distCal.done || turnCal.done) {
        pros::lcd::print(2, "Cal %s %.4f  %s %.4f", distCal.label, distCal.value,
                         turnCal.label, turnCal.value);
    }
    if (turnTune.ok || moveTune.ok) {
        pros::lcd::print(7, "PID T %.3g/%.2g/%.3g  M %.3g/%.2g/%.3g",
                         turnTune.kP, turnTune.kI, turnTune.kD, moveTune.kP, moveTune.kI, moveTune.kD);
    }
}

static void loop() {
    std::uint8_t lastButtons = 0;

    while (true) {
        std::uint8_t buttons = pros::lcd::read_buttons();
        std::uint8_t pressed = buttons & ~lastButtons;  // only react to new presses
        lastButtons = buttons;

        if (pressed & LCD_BTN_CENTER) {
            page = page == Page::Main ? Page::Distance : Page::Main;
            for (int line = 0; line < 8; line++) pros::lcd::clear_line(line);
        }
        if (page == Page::Distance) {
            if (pressed & LCD_BTN_LEFT) tapeInches = std::max(4, tapeInches - 1);
            if (pressed & LCD_BTN_RIGHT) tapeInches = std::min(70, tapeInches + 1);
        } else {
            if (pressed & LCD_BTN_LEFT) selectedAuton = (selectedAuton + autonCount - 1) % autonCount;
            if (pressed & LCD_BTN_RIGHT) selectedAuton = (selectedAuton + 1) % autonCount;
        }

        draw();
        pros::delay(50);
    }
}

void startBrainScreen() {
    pros::lcd::initialize();
    pros::Task task(loop, "Brain screen");
}
