#region VEXcode Generated Robot Configuration
import math
import random
from vexcode_vrc import *
from vexcode_vrc.events import get_Task_func

# Brain should be defined by default
brain=Brain()

claw_motor = Motor("ClawMotor", 6)
arm_motor = Motor("ArmMotor", 8)
distance = Distance("Distance", 10)
ai_vision = AiVision("aiVision", 7)
gps = GPS("GPS", 4)
drivetrain = Drivetrain("drivetrain", 0)

# 'Enum' class for AI Vision GameElements
class GameElements:
    RED_BLUE_PIN = 0
    BLUE_YELLOW_PIN = 1
    RED_YELLOW_PIN = 2
    YELLOW_PIN = 3
    CUP = 4

#endregion VEXcode Generated Robot Configuration
# ------------------------------------------------------------------
# 901E Enthropic - V5RC Override VR Autonomous Coding Skills
# Refactored v15 route (same robot actions as the v15 program).
#
# PRE-MATCH CHECKLIST (location-pin button above Start):
#   Starting Location = B (robot at x -1564, y 368, heading 55)
#   Preload Orientation = yellow on top
#
# Units: GPS millimetres (field centre is 0, 0) and GPS headings in degrees
# (0 = towards +y, 90 = towards +x). Arm and claw angles are motor degrees:
# claw 167 = open, 467 = closed; arm 0 = lowest.
#
# Route (goal centres: left neutral (-1200, 600), left red (-1200, -600),
# bottom red (-600, -1200), bottom neutral (600, -1200), top blue (600, 1200),
# top neutral (-600, 1200)):
#   1. Left neutral goal: preload, then the wall unit at (-1745, 600).
#   2. Left red goal: diamond pin (-600, 0), D unit (-600, -600), left toggle,
#      wall unit (-1745, -600).
#   3. Bottom red goal: diamond pin (0, -600), D unit (-1200, -1200),
#      wall unit (-600, -1745), then the bottom toggle.
#   4. Bottom neutral goal: the pin from the wall unit at (600, -1745).
#   5. Top blue goal: diamond pin (600, 0), D unit (600, 600).
#   6. Top neutral goal: diamond pin (0, 600), then the top toggle.
#   7. Top blue goal: wall unit (600, 1745). Top neutral goal: D unit (1200, 1200),
#      then the wall unit (-600, 1745) if the brain timer is under 55.6 s.
#   8. Park in the midfield.
#
# Every checkpoint prints "CP:<name> t=<s> x=<mm> y=<mm> h=<deg> c=<claw deg>"
# to the console. At 58.3 s (brain timer) a background thread sets out_of_time.
# From then on the driving, pickup, placing and toggle helpers return without
# doing anything, except drive_to_waypoint and approach_point while parking.
# ------------------------------------------------------------------

# Claw and placement
CLAW_TRAVEL = 300              # claw degrees from open to closed
RELEASE_FIRST_OPEN = 120       # open this much before moving on; the rest opens in the background
GRIP_DISTANCE = 200            # robot centre to the gripping point of the claw, mm
SETTLE_BEFORE_RELEASE = 0.05   # seconds to wait at the goal before opening the claw
WALL_UNIT_EXTRA_LIFT = 250     # extra arm lift after releasing a wall unit

# Toggle pushing
TOGGLE_ARM_READY = 850         # arm angle held while backing into a toggle
TOGGLE_ARM_PUSH = 450          # arm swing that pushes the toggle over
TOGGLE_WALL_LINE = 1613        # robot centre is 1613 mm from field centre when backed into the wall
TOGGLE_SLOW_ZONE = 120         # last part of the reverse, driven at TOGGLE_REVERSE_SPEED
TOGGLE_TURN_SPEED = 100        # percent
TOGGLE_REVERSE_SPEED = 50      # percent
TOGGLE_ARM_PUSH_SPEED = 50     # percent
TOGGLE_EXIT_DISTANCE = 150     # mm driven away from the wall after the push

# Timing
STOP_ACTIONS_TIME = 58.3       # brain timer seconds when out_of_time is set
LAST_WALL_UNIT_CUTOFF = 55.6   # only try the last wall unit (-600, 1745) before this time

out_of_time = False            # set by time_limit_watchdog
parking = False                # set when the robot starts driving to the park position


def log_checkpoint(name):
    brain.screen.print("CP:" + name + " t=" + str(round(brain.timer.time(SECONDS), 2)) + " x=" + str(int(gps.x_position(MM))) + " y=" + str(int(gps.y_position(MM))) + " h=" + str(int(gps.heading())) + " c=" + str(int(claw_motor.position(DEGREES))))
    brain.screen.next_row()


def apply_default_motor_settings():
    drivetrain.set_drive_velocity(100, PERCENT)
    drivetrain.set_turn_velocity(100, PERCENT)
    drivetrain.set_timeout(2, SECONDS)
    arm_motor.set_velocity(100, PERCENT)
    claw_motor.set_velocity(100, PERCENT)
    arm_motor.set_timeout(2, SECONDS)
    claw_motor.set_timeout(1, SECONDS)


def heading_between(from_x, from_y, to_x, to_y):
    # GPS heading (0-360 degrees) that points from one field point to another
    return math.degrees(math.atan2(to_x - from_x, to_y - from_y)) % 360


def distance_between(from_x, from_y, to_x, to_y):
    return math.sqrt((to_x - from_x) ** 2 + (to_y - from_y) ** 2)


def turn_to_face(target_heading):
    # Turn in place only if the robot is more than 1.5 degrees off
    current_heading = gps.heading()
    error = (target_heading - current_heading + 540) % 360 - 180
    if abs(error) > 1.5:
        drivetrain.set_heading(current_heading, DEGREES)
        drivetrain.turn_to_heading(target_heading, DEGREES)


def approach_point(target_x, target_y, stop_distance, recheck_heading):
    # Face the target and drive straight until the robot centre is stop_distance
    # mm from it (drives backward if it is already closer than that).
    if out_of_time and not parking:
        return
    x = gps.x_position(MM)
    y = gps.y_position(MM)
    turn_to_face(heading_between(x, y, target_x, target_y))
    if recheck_heading:
        wait(0.03, SECONDS)
        x = gps.x_position(MM)
        y = gps.y_position(MM)
        turn_to_face(heading_between(x, y, target_x, target_y))
    x = gps.x_position(MM)
    y = gps.y_position(MM)
    distance = distance_between(x, y, target_x, target_y) - stop_distance
    if distance > 0:
        drivetrain.drive_for(FORWARD, distance, MM)
    else:
        drivetrain.drive_for(REVERSE, -distance, MM)


def drive_to_waypoint(target_x, target_y):
    # Drive straight to a travel point, forward or backward, whichever needs less turning
    if out_of_time and not parking:
        return
    x = gps.x_position(MM)
    y = gps.y_position(MM)
    current_heading = gps.heading()
    target_heading = heading_between(x, y, target_x, target_y)
    distance = distance_between(x, y, target_x, target_y)
    error = (target_heading - current_heading + 540) % 360 - 180
    if abs(error) <= 90:
        turn_to_face(target_heading)
        drivetrain.drive_for(FORWARD, distance, MM)
    else:
        turn_to_face((target_heading + 180) % 360)
        drivetrain.drive_for(REVERSE, distance, MM)


def back_up(distance):
    if out_of_time:
        return
    drivetrain.drive_for(REVERSE, distance, MM)


def grab_unit(object_x, object_y, carry_angle):
    # Pick up a floor-level unit (cup + pin) by its cup with the arm fully down,
    # then start raising the arm to carry_angle in the background.
    if out_of_time:
        return
    x = gps.x_position(MM)
    y = gps.y_position(MM)
    turn_to_face(heading_between(x, y, object_x, object_y))
    arm_motor.spin_to_position(0, DEGREES, wait=False)
    approach_point(object_x, object_y, GRIP_DISTANCE, True)
    arm_motor.spin_to_position(0, DEGREES)
    claw_motor.spin_for(FORWARD, CLAW_TRAVEL, DEGREES)
    arm_motor.set_velocity(100, PERCENT)
    arm_motor.spin_to_position(carry_angle, DEGREES, wait=False)


def grab_pin_at_height(pin_x, pin_y, grip_angle, carry_angle):
    # Grip a pin at arm angle grip_angle (the diamond pins are gripped above
    # their cup at 300, so the cup stays behind), then raise it to carry_angle.
    arm_motor.spin_to_position(grip_angle, DEGREES, wait=False)
    approach_point(pin_x, pin_y, GRIP_DISTANCE, True)
    arm_motor.spin_to_position(grip_angle, DEGREES)
    claw_motor.spin_for(FORWARD, CLAW_TRAVEL, DEGREES)
    arm_motor.spin_to_position(carry_angle, DEGREES, wait=False)


def grab_wall_pin(pin_x, pin_y, carry_angle):
    # Take only the pin out of a wall unit: lower the claw around it, back off
    # 45 mm while raising the arm to 300, close, then raise to carry_angle.
    if out_of_time:
        return
    arm_motor.spin_to_position(0, DEGREES, wait=False)
    approach_point(pin_x, pin_y, GRIP_DISTANCE, True)
    arm_motor.spin_to_position(0, DEGREES)
    arm_motor.spin_to_position(300, DEGREES, wait=False)
    drivetrain.drive_for(REVERSE, 45, MM)
    arm_motor.spin_to_position(300, DEGREES)
    claw_motor.spin_for(FORWARD, CLAW_TRAVEL, DEGREES)
    arm_motor.spin_to_position(carry_angle, DEGREES, wait=False)


def release_claw():
    claw_motor.spin_for(REVERSE, RELEASE_FIRST_OPEN, DEGREES)
    claw_motor.spin_for(REVERSE, CLAW_TRAVEL - RELEASE_FIRST_OPEN, DEGREES, wait=False)


def place_on_goal(goal_x, goal_y, arm_angle, reach, from_wall, back_off):
    # Put the held piece on a goal. reach is the distance from the robot centre
    # to the goal centre at release. from_wall is for units carried low from a
    # wall: back off 30 mm, raise the arm while turning to the goal, and lift
    # the arm clear of the stack after releasing.
    if out_of_time:
        return
    if from_wall:
        drivetrain.drive_for(REVERSE, 30, MM)
        arm_motor.spin_to_position(arm_angle, DEGREES, wait=False)
        x = gps.x_position(MM)
        y = gps.y_position(MM)
        turn_to_face(heading_between(x, y, goal_x, goal_y))
    else:
        arm_motor.spin_to_position(arm_angle, DEGREES, wait=False)
    approach_point(goal_x, goal_y, reach, True)
    arm_motor.spin_to_position(arm_angle, DEGREES)
    if out_of_time:
        return
    wait(SETTLE_BEFORE_RELEASE, SECONDS)
    release_claw()
    if from_wall:
        arm_motor.spin_to_position(arm_angle + WALL_UNIT_EXTRA_LIFT, DEGREES)
    if back_off > 0:
        drivetrain.drive_for(REVERSE, back_off, MM)
    arm_motor.set_velocity(100, PERCENT)


def distance_from_wall_line(side):
    # How far the robot centre still is from the wall line behind a toggle
    if side == "L":
        return gps.x_position(MM) + TOGGLE_WALL_LINE
    elif side == "B":
        return gps.y_position(MM) + TOGGLE_WALL_LINE
    else:
        return 0 - gps.y_position(MM) + TOGGLE_WALL_LINE


def push_toggle(side):
    # side "L" (left wall), "B" (bottom wall) or "T" (top wall). Back into the
    # wall with the arm up, then swing the arm down once to flip the toggle.
    if out_of_time:
        return
    arm_motor.spin_to_position(TOGGLE_ARM_READY, DEGREES, wait=False)
    if side == "L":
        drive_to_waypoint(-1150, 43)
        facing = 90
    elif side == "B":
        drive_to_waypoint(-43, -1450)
        facing = 0
    else:
        drive_to_waypoint(43, 1450)
        facing = 180
    drivetrain.set_turn_velocity(TOGGLE_TURN_SPEED, PERCENT)
    drivetrain.set_heading(gps.heading(), DEGREES)
    drivetrain.turn_to_heading(facing, DEGREES)
    remaining = distance_from_wall_line(side)
    if remaining > TOGGLE_SLOW_ZONE:
        drivetrain.set_drive_velocity(100, PERCENT)
        drivetrain.drive_for(REVERSE, remaining - TOGGLE_SLOW_ZONE, MM)
    drivetrain.set_drive_velocity(TOGGLE_REVERSE_SPEED, PERCENT)
    arm_motor.spin_to_position(TOGGLE_ARM_READY, DEGREES)
    arm_motor.set_velocity(TOGGLE_ARM_PUSH_SPEED, PERCENT)
    # Store the result first: VEXcode VR does not wait for a helper that reads
    # sensors when it is called inside another command's argument list.
    remaining = distance_from_wall_line(side)
    drivetrain.drive_for(REVERSE, remaining, MM)
    arm_motor.spin_for(REVERSE, TOGGLE_ARM_PUSH, DEGREES)
    apply_default_motor_settings()
    drivetrain.drive_for(FORWARD, TOGGLE_EXIT_DISTANCE, MM)


def start_run():
    brain.timer.clear()
    apply_default_motor_settings()
    drivetrain.set_heading(gps.heading(), DEGREES)
    log_checkpoint("__start")


def time_limit_watchdog():
    global out_of_time
    while brain.timer.time(SECONDS) < STOP_ACTIONS_TIME:
        wait(0.05, SECONDS)
    out_of_time = True


def main():
    global parking
    start_run()

    # Left neutral goal: preload, then the left wall unit on top
    arm_motor.spin_to_position(850, DEGREES)
    approach_point(-1200, 600, 275, False)
    claw_motor.spin_for(REVERSE, CLAW_TRAVEL, DEGREES)
    back_up(150)
    log_checkpoint("preload")
    arm_motor.spin_to_position(0, DEGREES, wait=False)
    drive_to_waypoint(-1496, 600)
    grab_unit(-1745, 600, 0)
    place_on_goal(-1200, 600, 600, 296, True, 100)
    log_checkpoint("nl1")

    # Left red goal: diamond pin, D unit, left toggle, left wall unit
    arm_motor.spin_to_position(850, DEGREES, wait=False)
    drive_to_waypoint(-1050, 150)
    grab_pin_at_height(-600, 0, 300, 600)
    place_on_goal(-1200, -600, 600, 280, False, 150)
    log_checkpoint("rlbase")
    grab_unit(-600, -600, 350)
    place_on_goal(-1200, -600, 350, 269, False, 150)
    log_checkpoint("rl1")
    push_toggle("L")
    log_checkpoint("toggleL")
    arm_motor.spin_to_position(0, DEGREES, wait=False)
    drive_to_waypoint(-1496, -600)
    grab_unit(-1745, -600, 0)
    place_on_goal(-1200, -600, 1000, 310, True, 100)
    log_checkpoint("rl2")

    # Bottom red goal: diamond pin, D unit, bottom wall unit, then the bottom toggle
    drive_to_waypoint(-1450, -950)
    drive_to_waypoint(-850, -850)
    grab_pin_at_height(0, -600, 300, 600)
    place_on_goal(-600, -1200, 600, 280, False, 150)
    log_checkpoint("rbbase")
    grab_unit(-1200, -1200, 350)
    place_on_goal(-600, -1200, 350, 269, False, 150)
    log_checkpoint("rb1")
    arm_motor.spin_to_position(0, DEGREES, wait=False)
    drive_to_waypoint(-1000, -1496)
    drive_to_waypoint(-600, -1496)
    grab_unit(-600, -1745, 0)
    place_on_goal(-600, -1200, 1000, 310, True, 100)
    log_checkpoint("rb2")
    push_toggle("B")
    log_checkpoint("toggleB")

    # Bottom neutral goal: the pin from the wall unit at (600, -1745)
    arm_motor.spin_to_position(0, DEGREES, wait=False)
    drive_to_waypoint(600, -1496)
    grab_wall_pin(600, -1745, 300)
    back_up(100)
    place_on_goal(600, -1200, 850, 275, False, 100)
    log_checkpoint("nbbase")

    # Top blue goal (diamond pin, D unit) and top neutral goal base, then the top toggle
    drive_to_waypoint(150, -1400)
    drive_to_waypoint(150, -750)
    grab_pin_at_height(600, 0, 300, 600)
    drive_to_waypoint(150, 300)
    place_on_goal(600, 1200, 600, 280, False, 150)
    log_checkpoint("btbase")
    grab_unit(600, 600, 350)
    place_on_goal(600, 1200, 350, 269, False, 150)
    log_checkpoint("bt1")
    grab_pin_at_height(0, 600, 300, 850)
    drive_to_waypoint(0, 900)
    place_on_goal(-600, 1200, 850, 275, False, 150)
    log_checkpoint("ntbase")
    push_toggle("T")
    log_checkpoint("toggleT")

    # Wall unit (600, 1745) on the top blue goal, D unit (1200, 1200) on the top neutral goal
    arm_motor.spin_to_position(0, DEGREES, wait=False)
    drive_to_waypoint(600, 1496)
    grab_unit(600, 1745, 0)
    place_on_goal(600, 1200, 1000, 310, True, 100)
    log_checkpoint("bt2")
    grab_unit(1200, 1200, 600)
    drive_to_waypoint(750, 1500)
    place_on_goal(-600, 1200, 600, 296, False, 150)
    log_checkpoint("nt1")

    # Wall unit (-600, 1745) on the top neutral goal, only if the timer allows
    if brain.timer.time(SECONDS) < LAST_WALL_UNIT_CUTOFF:
        arm_motor.spin_to_position(0, DEGREES, wait=False)
        drive_to_waypoint(-600, 1496)
        grab_unit(-600, 1745, 0)
        place_on_goal(-600, 1200, 1150, 302, True, 100)
        log_checkpoint("nt2")

    # Park in the midfield (these moves still run after out_of_time is set)
    parking = True
    drive_to_waypoint(150, 750)
    drive_to_waypoint(250, 450)
    log_checkpoint("park")


vr_thread(time_limit_watchdog)
vr_thread(main)
