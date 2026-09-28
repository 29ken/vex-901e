#include "screens.hpp"
#include "main.h"
#include "autons.hpp"
#include "config.hpp"
#include "drivetrain.hpp"
#include "tuner.hpp"
#include <cstdio>

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
    pros::lcd::print(0, "Auton: %s  (%d/%d)", autons[selectedAuton].name,
                     selectedAuton + 1, autonCount);
    pros::lcd::print(1, "left / right buttons to change");

    if (!BRAIN_UI) return;

    printSide(3, "L", leftMotors);
    printSide(4, "R", rightMotors);
    pros::lcd::print(5, "Battery %.0f%%", pros::battery::get_capacity());

    lemlib::Pose pose = chassis.getPose();
    pros::lcd::print(6, "X %.1f  Y %.1f  H %.1f", pose.x, pose.y, pose.theta);

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

        if (pressed & LCD_BTN_LEFT) {
            selectedAuton = (selectedAuton + autonCount - 1) % autonCount;
        }
        if (pressed & LCD_BTN_RIGHT) {
            selectedAuton = (selectedAuton + 1) % autonCount;
        }

        draw();
        pros::delay(50);
    }
}

void startBrainScreen() {
    pros::lcd::initialize();
    pros::Task task(loop, "Brain screen");
}
