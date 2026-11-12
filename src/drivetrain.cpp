#include "drivetrain.hpp"
#include "config.hpp"
#include <cmath>
#include <cstdlib>
// RW uses left_chassis / right_chassis (declared in rw/devices.h, defined in
// rw-config.cpp). The rest of our code (controls, screens) still says
// leftMotors / rightMotors, so we alias them here.
pros::MotorGroup& leftMotors  = left_chassis;
pros::MotorGroup& rightMotors = right_chassis;
void initDrive() {
    // Set default brake mode for driver control.
    stopChassis(pros::MotorBrake::coast);
}
void driveInches(float inches, int timeout, bool async) {
    if (async) {
        pros::Task driveTask([inches, timeout]() {
            driveTo(inches, timeout);
        });
    } else {
        driveTo(inches, timeout);
    }
}
void turnDegrees(float degrees, int timeout, bool async) {
    double target = correct_angle + degrees;
    if (async) {
        pros::Task turnTask([target, timeout]() {
            turnToAngle(target, timeout);
        });
    } else {
        turnToAngle(target, timeout);
    }
}
std::vector<MotorTemp> motorTemps(pros::MotorGroup& motors) {
    std::vector<double> temps = motors.get_temperature_all();
    std::vector<std::int8_t> ports = motors.get_port_all();
    std::vector<MotorTemp> out;
    for (std::size_t i = 0; i < temps.size(); i++) {
        // An unplugged motor reports a huge error value instead of a temperature.
        out.push_back({std::abs(ports[i]), temps[i], temps[i] < 200});
    }
    return out;
}
