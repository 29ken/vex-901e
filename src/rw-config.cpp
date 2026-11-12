#include "rw-config.h"

// ============================================================================
// DEVICES
// RW uses the devices below by name (see rw/devices.h), so keep the names and set the ports.
// Declare your own devices in include/rw-config.h too, so other files can use them.
// ============================================================================

pros::Controller controller_1(pros::E_CONTROLLER_MASTER);

// Drive motors: negative port = reversed. Match config.hpp ports.
// 11W motors first so RW reads drive encoders from the right gearing.
pros::MotorGroup left_chassis({-19, -20, -18}, pros::MotorGearset::blue);   // LEFT_11W_A, LEFT_5W, LEFT_11W_B
pros::MotorGroup right_chassis({9, 10, 8}, pros::MotorGearset::blue);       // RIGHT_11W_A, RIGHT_5W, RIGHT_11W_B

pros::Imu inertial_sensor(11);  // give it a free port (IMU_PORT is 0 in config.hpp = no IMU yet)

// Tracking wheels
pros::Rotation horizontal_tracker(7);    // HORIZONTAL_POD_PORT from config.hpp
pros::Rotation vertical_tracker(6);      // VERTICAL_POD_PORT from config.hpp

// Distance reset sensors (give unused ones any free port)
pros::Distance front_sensor(12);
pros::Distance left_sensor(13);
pros::Distance right_sensor(14);
pros::Distance back_sensor(15);

// ============================================================================
// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
// ============================================================================

// Distance between the middles of the left and right wheels of the drive (in inches)
// (mecanum: plus the distance between the front and back wheels, see the docs: Guide > Drive Types)
double distance_between_wheels = 11.5;

// motor to wheel gear ratio * wheel diameter (in inches) * pi
double wheel_distance_in = (36.0 / 48.0) * 2.75 * M_PI;

// PID Constants for movement
// distance_* : Linear PID for straight driving
// turn_*     : PID for turning in place
// heading_correction_* : PID for heading correction during linear movement
double distance_kp = 1.1, distance_ki = 0.1, distance_kd = 7;
double turn_kp = 0.3, turn_ki = 0, turn_kd = 2.5;
double heading_correction_kp = 0.6, heading_correction_ki = 0, heading_correction_kd = 4;

// Enable or disable the use of tracking wheels
bool using_horizontal_tracker = true;   // You have one on port 7
bool using_vertical_tracker = true;    // You have one on port 6

// IGNORE THESE IF YOU ARE NOT USING TRACKING WHEELS
// These comments are in the perspective of a top down view of the robot when the robot is facing vertical
// Tracking wheel offsets — matched from config.hpp
double horizontal_tracker_dist_from_center = -0.26;   // HORIZONTAL_POD_OFFSET
double vertical_tracker_dist_from_center = 0.0;       // VERTICAL_POD_OFFSET
double horizontal_tracker_diameter = 2.125;            // POD_WHEEL_DIAMETER
double vertical_tracker_diameter = 2.125;              // POD_WHEEL_DIAMETER

// Distance Reset setup
// If you are not using all four distance sensors, just set the unused ones to 0
// If you are not using distance resets these values will be ignored
// Add your sensor offsets here (where the sensor's face is, relative to the robot's center)
// If a sensor is dead-center but 6.5" forward relative to the robot's center:
// double front_sensor_offsetX = 0.0;
// double front_sensor_offsetY = 6.5;
// For the X offset, a negative value indicates that the sensor is located on the left side of the robot, while a positive value indicates that it is on the right side.

double front_sensor_offsetX = 0.0;
double front_sensor_offsetY = 0.0;

double left_sensor_offsetX = 0.0;
double left_sensor_offsetY = 0.0;

double right_sensor_offsetX = 0.0;
double right_sensor_offsetY = 0.0;

double back_sensor_offsetX = 0.0;
double back_sensor_offsetY = 0.0;

// ============================================================================
// ADVANCED TUNING (OPTIONAL)
// ============================================================================

bool heading_correction = true; // Use heading correction when the bot is stationary

// Set to true for more accuracy and smoothness, false for more speed
bool dir_change_start = true;   // Less accel/decel due to expecting direction change at start of movement
bool dir_change_end = true;     // Less accel/decel due to expecting direction change at end of movement

double min_output = 10; // Minimum output voltage to motors while chaining movements

// Maximum allowed change in voltage output per 10 msec during movement
double max_slew_accel_fwd = 24;
double max_slew_decel_fwd = 24;
double max_slew_accel_rev = 24;
double max_slew_decel_rev = 24;

// Prevents too much slipping during boomerang movements
// Decrease if there is too much drifting and inconsistency during boomerang
// Increase for more speed during boomerang
double chase_power = 2;

// ============================================================================
// NEWER SETTINGS (OPTIONAL)
// Every setting here works at its default. Change one only when you use the feature it's for.
// ============================================================================

// Multiplies the IMU heading. Many V5 IMUs read a little off over full turns (e.g. 357 for 360).
// Find yours with measureImuScale(), 1.0 = no correction
double imu_scale = 1.0;

// End a motion early once the robot has been stuck (not moving or turning) for this long, in msec.
// Saves time if the robot gets caught on something. 0 = off.
// Leave it off for moves meant to push into something until they time out. 350 or more avoids false exits.
double stall_exit_msec = 0;

// Distance resets: skip readings where the beam hits the wall at a steeper angle than this (degrees).
// The sensor's beam is a cone about 12 degrees wide on each side, so steep readings get less exact
double distance_reset_max_angle = 30;
// Distance resets: skip readings that would move the position more than this (inches, 0 = off).
// 6-10 helps ignore other robots and game elements in front of the wall
double distance_reset_max_jump = 0;
// Distance resets: skip readings that hit the wall this close to a corner (inches, 0 = off).
// For Override, about 18 keeps the corner loaders out of the readings
double distance_reset_corner_margin = 0;

// MCL (see startMcl() in src/rw-user.cpp): particle count (more = smoother but slower, up to 1000),
// longest distance reading it trusts (inches), and how far behind the distance sensor readings are (msec).
// If the position lags while driving toward or away from a wall, adjust the delay.
// The selector screen shows how long each MCL update takes; lower the particle count if it's more than a few msec
int mcl_particles = 200;
double mcl_max_range = 60;
double distance_sensor_delay_msec = 40;

// Multiplies every distance sensor reading (resets and MCL). V5 distance sensors often read 1-3% long.
// To measure yours: put the robot a tape-measured distance from a wall (2-4 ft), then use
// (measured distance) / (what the sensor reads). 1.0 = no correction
double distance_sensor_scale = 1.0;

// Motion profiles and path following (driveProfiled, turnProfiled, followPath, followPathRamsete).
// These use a model of your drive: drive_kS = volts to start moving, drive_kV = volts per in/s,
// drive_kA = volts per in/s^2. Leave kV and kA at 0 to estimate them from drive_motor_rpm and
// wheel_distance_in, or measure all three with measureDriveFeedforward()
double drive_kS = 0.5;
double drive_kV = 0;
double drive_kA = 0;
double drive_motor_rpm = 450;          // 600 (blue) geared down 36:48 = 450 effective RPM
double profile_max_speed = 0;          // Top speed (in/s, 0 = 80% of the drive's top speed)
double profile_max_accel = 100;        // Speed up / slow down rate (in/s^2): lower if the wheels slip
double profile_turn_speed = 360;       // turnProfiled top speed (degrees/s)
double profile_turn_accel = 1200;      // turnProfiled acceleration (degrees/s^2)
double path_lookahead = 12;            // Pure pursuit lookahead (inches): bigger = smoother, smaller = tighter
double path_max_lateral_accel = 120;   // Slows down in curves so the robot doesn't slide (in/s^2)

// Driver control feel, used by driverTank(), driverArcade() and driverHolonomic()
double driver_curve = 0;     // 0 = linear, higher = finer control at low stick values (2-6 is typical)
double driver_deadband = 3;  // Stick values below this (percent) count as zero

// X-drives, mecanum drives and H-drives (see the docs: Guide > Drive Types): how far the robot moves
// sideways per revolution of a strafing motor (inches). 0 = the same as wheel_distance_in, right for an X-drive
double strafe_distance_in = 0;
