#include "drivetrain.hpp"
#include "config.hpp"

namespace drive {

// Blue cartridge is for the 11W motors; the 5.5Ws ignore it.
pros::MotorGroup leftMotors({config::LEFT_5W, config::LEFT_11W_A, config::LEFT_11W_B},
                            pros::MotorGearset::blue);
pros::MotorGroup rightMotors({config::RIGHT_11W_A, config::RIGHT_5W, config::RIGHT_11W_B},
                             pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              config::TRACK_WIDTH,
                              lemlib::Omniwheel::NEW_275,
                              config::DRIVE_RPM,
                              config::HORIZONTAL_DRIFT);

// No IMU or tracking wheels yet, so odom runs off the drive encoders.
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, nullptr);

// kP, kI, kD, anti-windup, small error, small timeout, large error, large timeout, slew
lemlib::ControllerSettings lateralController(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angularController(2, 0, 10, 3, 1, 100, 3, 500, 0);

// deadband, min output, curve gain
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors,
                        &throttleCurve, &steerCurve);

void init() {
    chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}

} // namespace drive
