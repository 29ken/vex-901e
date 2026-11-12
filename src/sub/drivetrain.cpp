#include "drivetrain.hpp"
#include "config.hpp"
#include <cmath>
#include <cstdlib>

bool driverBrakeHold = false;

void initDrive() {
    // RW's odometry, which every auton move and the brain screen's X/Y/H need:
    // calibrate the IMU (about 2 s, so keep the robot still), then start tracking.
    inertial_sensor.reset(true);
    startOdometry();
    stopChassis(pros::MotorBrake::coast);
}

void applyDriverBrakes() {
    stopChassis(driverBrakeHold ? pros::MotorBrake::hold : pros::MotorBrake::coast);
}

static int deadband(int value) {
    return (std::abs(value) < STICK_DEADBAND) ? 0 : value;
}

static float turnScale(int throttle) {
    float speed = std::abs(throttle) / 127.0f;
    return TURN_SCALE_MIN + (TURN_SCALE_MAX - TURN_SCALE_MIN) * speed;
}

// Turning harder builds up slowly; letting off or reversing snaps back.
static float rampTurn(float current, float target) {
    bool turningHarder = std::fabs(target) > std::fabs(current) && target * current >= 0;
    float maxStep = turningHarder ? TURN_RAMP_UP : TURN_RAMP_DOWN;

    float step = target - current;
    if (step >  maxStep) step =  maxStep;
    if (step < -maxStep) step = -maxStep;
    return current + step;
}

void driveControl() {
    static float currentTurn = 0;

    int leftY  = deadband(controller_1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    int leftX  = deadband(controller_1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    int rightX = deadband(controller_1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

    if (SPLIT_ARCADE) leftX = 0;

    // Pushing the left stick nearly straight up or down drives dead straight.
    if (std::abs(leftX) < std::abs(leftY) * STRAIGHT_ZONE) leftX = 0;

    // Either stick can turn; whichever is asking for more wins.
    int turn = (std::abs(rightX) > std::abs(leftX)) ? rightX : leftX;

    // Shove the left stick sideways and you get a clean spin, not an arc.
    int throttle = leftY;
    if (std::abs(leftY) < std::abs(leftX) * SIDEWAYS_ZONE) throttle = 0;

    currentTurn = rampTurn(currentTurn, turn * turnScale(throttle));

    // RW's driveChassis takes -12 to 12 (volts). Scale from -127..127:
    double fwd  = throttle / 127.0 * 12.0;
    double steer = currentTurn / 127.0 * 12.0;
    driveChassis(fwd + steer, fwd - steer);
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
