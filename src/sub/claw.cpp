#include "claw.hpp"
#include "main.h"
#include "config.hpp"
#include "drivetrain.hpp"
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>

static pros::adi::DigitalOut piston(CLAW_PORT, false);  // false = open
static pros::Distance sensor(AUTOGRAB_SENSOR);

// The driver loop, autons and the autograb task all move the claw.
static pros::Mutex mutex;
static bool closed = false;
static std::atomic<bool> armed{false};
static std::atomic<std::uint32_t> grabs{0};  // autograb closings, for clawGrabWhenSeen()
// The odometry trigger of autograbArmNear(): off while nearRadius is 0.
static std::atomic<double> nearX{0}, nearY{0}, nearRadius{0};
static std::atomic<char> how{0};
static std::atomic<std::uint32_t> whenMs{0};

static void setClaw(bool close) {
    std::lock_guard<pros::Mutex> hold(mutex);
    closed = close;
    piston.set_value(close);
}

static void loop() {
    while (true) {
        if (armed) {
            std::int32_t mm = sensor.get();  // PROS_ERR (huge) when unplugged, so it never grabs
            bool seen = mm > 0 && mm < AUTOGRAB_MM;
            double r = nearRadius;
            bool there = r > 0 && std::hypot(nearX - x_pos, nearY - y_pos) <= r;
            if (seen || there) {
                setClaw(true);
                how = seen ? 's' : 'o';
                whenMs = pros::millis();
                grabs++;  // before disarming, so clawGrabWhenSeen() sees it when it wakes
                nearRadius = 0;
                armed = false;
            }
        }
        pros::delay(10);
    }
}

void initClaw() {
    setClaw(false);
    pros::Task task(loop, "Claw");
}

void clawGrab() { setClaw(true); }
void clawRelease() { setClaw(false); }

void clawToggle() {
    std::lock_guard<pros::Mutex> hold(mutex);
    closed = !closed;
    piston.set_value(closed);
}

bool clawClosed() {
    std::lock_guard<pros::Mutex> hold(mutex);
    return closed;
}

void autograbArm(bool on) {
    nearRadius = 0;
    if (on) how = 0;
    armed = on;
}

void autograbArmNear(double x, double y, double radius_in) {
    nearX = x;
    nearY = y;
    nearRadius = radius_in;
    how = 0;
    armed = true;
}

char autograbHow() { return how; }
unsigned autograbWhenMs() { return whenMs; }
bool autograbArmed() { return armed; }

bool autograbWait(int timeout_ms) {
    std::uint32_t start = pros::millis();
    while (armed && pros::millis() - start < static_cast<std::uint32_t>(timeout_ms)) {
        pros::delay(10);
    }
    armed = false;
    return clawClosed();
}

bool clawGrabWhenSeen(int timeout_ms) {
    std::uint32_t before = grabs;
    std::uint32_t start = pros::millis();
    armed = true;
    while (armed && pros::millis() - start < static_cast<std::uint32_t>(timeout_ms)) {
        pros::delay(10);
    }
    armed = false;
    return grabs != before;
}
