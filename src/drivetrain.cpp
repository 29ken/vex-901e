#include "drivetrain.hpp"
#include "config.hpp"

// Blue cartridge is for the 11W motors; the 5.5Ws ignore it.
pros::MotorGroup leftMotors({LEFT_5W, LEFT_11W_A, LEFT_11W_B}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({RIGHT_11W_A, RIGHT_5W, RIGHT_11W_B}, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              TRACK_WIDTH,
                              lemlib::Omniwheel::NEW_275,
                              DRIVE_RPM,
                              HORIZONTAL_DRIFT);

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

void initDrive() {
    chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}
