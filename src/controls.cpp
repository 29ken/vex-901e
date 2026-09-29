#include "controls.hpp"
#include "drivetrain.hpp"
#include "config.hpp"
#include <cmath>
#include <cstdlib>

pros::Controller controller(pros::E_CONTROLLER_MASTER);

bool driverBrakeHold = false;

void applyDriverBrakes() {
    chassis.setBrakeMode(driverBrakeHold ? pros::E_MOTOR_BRAKE_HOLD : pros::E_MOTOR_BRAKE_COAST);
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

    int leftY  = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    int leftX  = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    int rightX = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

    if (SPLIT_ARCADE) leftX = 0;

    // Pushing the left stick nearly straight up or down drives dead straight.
    if (std::abs(leftX) < std::abs(leftY) * STRAIGHT_ZONE) leftX = 0;

    // Either stick can turn; whichever is asking for more wins.
    int turn = (std::abs(rightX) > std::abs(leftX)) ? rightX : leftX;

    // Shove the left stick sideways and you get a clean spin, not an arc.
    int throttle = leftY;
    if (std::abs(leftY) < std::abs(leftX) * SIDEWAYS_ZONE) throttle = 0;

    currentTurn = rampTurn(currentTurn, turn * turnScale(throttle));
    chassis.arcade(throttle, static_cast<int>(std::round(currentTurn)));
}
