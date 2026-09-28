#include "drivetrain.hpp"
#include "config.hpp"
#include <cstdlib>

// Blue cartridge is for the 11W motors; the 5.5Ws ignore it.
pros::MotorGroup leftMotors({LEFT_5W, LEFT_11W_A, LEFT_11W_B}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({RIGHT_11W_A, RIGHT_5W, RIGHT_11W_B}, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              TRACK_WIDTH,
                              DRIVE_WHEEL_DIAMETER,
                              DRIVE_RPM,
                              HORIZONTAL_DRIFT);

static lemlib::TrackingWheel* makePod(std::int8_t port, float offset) {
    if (!port) return nullptr;
    return new lemlib::TrackingWheel(new pros::Rotation(port), POD_WHEEL_DIAMETER, offset);
}

static pros::Imu* imu = IMU_PORT ? new pros::Imu(IMU_PORT) : nullptr;
static lemlib::TrackingWheel* verticalPod = makePod(VERTICAL_POD_PORT, VERTICAL_POD_OFFSET);
static lemlib::TrackingWheel* horizontalPod = makePod(HORIZONTAL_POD_PORT, HORIZONTAL_POD_OFFSET);

lemlib::OdomSensors sensors(verticalPod, nullptr, horizontalPod, nullptr, imu);

// kP, kI, kD, anti-windup, small error, small timeout, large error, large timeout, slew
lemlib::ControllerSettings lateralController(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angularController(2, 0, 10, 3, 1, 100, 3, 500, 0);

// deadband, min output, curve gain
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors,
                        &throttleCurve, &steerCurve);

void initDrive() {
    chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
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
