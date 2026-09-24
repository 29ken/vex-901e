#include "ui.hpp"
#include "main.h"
#include "autons.hpp"
#include "config.hpp"
#include "controls.hpp"
#include "drivetrain.hpp"
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace ui {
namespace {

// A motor that isn't plugged in reports a nonsense temperature.
constexpr double TEMP_UNPLUGGED = 200.0;

void printSide(int line, const char* label, pros::MotorGroup& motors) {
    std::vector<double> temps = motors.get_temperature_all();
    std::vector<std::int8_t> ports = motors.get_port_all();

    char row[48];
    int n = std::snprintf(row, sizeof(row), "%s", label);
    for (std::size_t i = 0; i < temps.size() && n < static_cast<int>(sizeof(row)); i++) {
        int port = std::abs(ports[i]);
        if (temps[i] > TEMP_UNPLUGGED) {
            n += std::snprintf(row + n, sizeof(row) - n, " %d:--", port);
        } else {
            n += std::snprintf(row + n, sizeof(row) - n, " %d:%.0f", port, temps[i]);
        }
    }
    pros::lcd::print(line, "%s", row);
}

// Whatever drive motor is running hottest right now, for the controller screen.
void hottestMotor(int& port, double& temp) {
    port = 0;
    temp = 0;
    for (pros::MotorGroup* group : {&drive::leftMotors, &drive::rightMotors}) {
        std::vector<double> temps = group->get_temperature_all();
        std::vector<std::int8_t> ports = group->get_port_all();
        for (std::size_t i = 0; i < temps.size(); i++) {
            if (temps[i] < TEMP_UNPLUGGED && temps[i] > temp) {
                temp = temps[i];
                port = std::abs(ports[i]);
            }
        }
    }
}

void drawBrain() {
    pros::lcd::print(0, "Auton: %s  (%d/%d)", autons::list[autons::selected].name,
                     autons::selected + 1, autons::count);
    pros::lcd::print(1, "left / right buttons to change");

    if (!config::BRAIN_UI) return;

    printSide(3, "L", drive::leftMotors);
    printSide(4, "R", drive::rightMotors);
    pros::lcd::print(5, "Battery %.0f%%", pros::battery::get_capacity());

    lemlib::Pose pose = drive::chassis.getPose();
    pros::lcd::print(6, "X %.1f  Y %.1f  H %.1f", pose.x, pose.y, pose.theta);
}

// The controller screen only accepts one line every 50 ms, so each pass writes
// one and all three refresh every 150 ms.
void drawController(int line) {
    switch (line) {
        case 0:
            controls::controller.print(0, 0, "%-14s", autons::list[autons::selected].name);
            break;
        case 1: {
            int port;
            double temp;
            hottestMotor(port, temp);
            controls::controller.print(1, 0, "Hot %d: %.0fC   ", port, temp);
            break;
        }
        case 2:
            controls::controller.print(2, 0, "Batt %.0f%%     ", pros::battery::get_capacity());
            break;
    }
}

void loop() {
    std::uint8_t lastButtons = 0;
    int controllerLine = 0;

    while (true) {
        std::uint8_t buttons = pros::lcd::read_buttons();
        std::uint8_t pressed = buttons & ~lastButtons;  // only react to new presses
        lastButtons = buttons;

        if (pressed & LCD_BTN_LEFT) {
            autons::selected = (autons::selected + autons::count - 1) % autons::count;
        }
        if (pressed & LCD_BTN_RIGHT) {
            autons::selected = (autons::selected + 1) % autons::count;
        }

        drawBrain();
        if (config::CONTROLLER_UI) {
            drawController(controllerLine);
            controllerLine = (controllerLine + 1) % 3;
        }

        pros::delay(50);
    }
}

} // namespace

void init() {
    pros::lcd::initialize();
    pros::Task uiTask(loop, "UI");
}

} // namespace ui
