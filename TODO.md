# 901E to-do (V4 robot)

Branch `v4-cad-fixes`. Tick things off as they're done. "Paste into" says
which line of code each measured number replaces.

## Before practice: code still to write (no robot needed)

- [ ] Port finder page on the brain screen: lists what's plugged into every
      port, so ports can be picked at practice and typed straight into the code.
- [ ] Test autons in the selector: square test, 48 in straight there-and-back,
      turn test, curve tests at 3 speeds, wall-reset test, "Drive Calibration"
      (records the log that makes the simulator drive like our robot).
- [ ] Toggle / ratchet / optical code with every port, direction and angle as
      a blank to fill in at practice, plus a safe test mode.
      (`setToggle()` in `src/sub/toggle.cpp` only waits for now.)
- [ ] Log every auton run to the SD card (position, sensor readings, resets).
- [ ] Fix the old failing lift test (`test/lift_check.cpp:109`).
- [ ] Fix the routes' scoring (see "Autons" below).

## Before practice: team decisions

- [ ] How does V4 pick up the Cup+Pin stacks against the walls? A goal sits
      21.6 in in front of each one; the robot needs 19.6 in to spin and the gap
      is 17.2 in. Best path found leaves ~0.5 in on each side.
- [ ] Where do the pieces start? The manual only shows it in its field drawing.
      A photo or screenshot of that figure is enough.
- [ ] Which AWP rules our events use (v1.1: 7 pins, 3 goals with 2+ pins each,
      nobody touching the wall; Worlds-qualifying events changed it).
- [ ] Driver buttons for the ratchet / winch / toggle.
- [ ] Getting the code onto the robot: bring this Mac, or OK pushing a branch.

## At practice, in this order

1. **Ports.** Plug everything in and fill in the numbers:
   - `src/rw-config.cpp`: drive motors, IMU (port 11 is a guess), the four
     distance sensors.
   - `include/config.hpp`: lift motors and lift sensor, claw (`CLAW_PORT`),
     autograb sensor (`AUTOGRAB_SENSOR`).
   - Ratchet motor, toggle rotation sensor, optical sensor: write them down
     (code still to come).
   - Check: the controller says MOTORS 6/6 OK; X, Y and H on the brain screen
     change the right way when you push the robot.
2. **Cal turn** (controller: X for the menu, UP to step through it, A to start):
   spin the robot exactly 360 degrees, press A. Paste into `rw-config.cpp`:
   `imu_scale`.
3. **Cal dist**: push the robot exactly 48 in, press A. Paste into
   `rw-config.cpp`: `wheel_distance_in`.
4. **Turn PID**, then **Move PID** (driver control enabled, room to move).
   Paste the P/I/D shown on the brain screen into `rw-config.cpp`.
5. **Distance sensors** (brain screen, middle button = Distance page):
   - Each sensor: park its lens a tape-measured distance from a wall (set the
     distance with left/right), robot square to the wall, wait 2 s. Paste its
     "x" number into `include/resets.hpp`: `SCALE_FRONT` / `BACK` / `LEFT` /
     `RIGHT` (the terminal prints the line ready to paste).
   - Offsets: put the robot at a known spot (back against a wall, tape the
     other distance) and check the page's x/y matches. If not, the offsets in
     `rw-config.cpp` are off.
   - Clear walls (if the field has them): sensor 6, 12, 24, 36 in from the wall
     at 0, 10, 20 and 30 degrees off square. Write down where readings go wrong
     and set `RESET_MAX_OFF_SQUARE_DEG` in `resets.hpp` (12 now).
   - Front sensor vs the claw: with a pin in the claw, raise the lift until the
     front reading stops seeing the pin. Paste the "Lift" number into
     `RESET_FRONT_CLEAR_LIFT`.
6. **Lift heights** (raise by hand with R2, read "Lift" on the brain screen).
   Paste into `include/config.hpp`: `LIFT_GRAB`, `LIFT_CARRY`,
   `LIFT_ALLIANCE_GOAL`, `LIFT_NEUTRAL_GOAL`, `LIFT_PER_PIECE`, `LIFT_PRESETS`.
7. **Ratchet and toggle**: which way the ratchet motor winds the winch and which
   way it only turns the flapper; the flapper's rotation-sensor angles for
   "down" and "flipped"; how long a flip takes (`TOGGLE_FLIP_MS`); the optical
   sensor's hue on the red, blue and yellow faces.
8. **Field checks**:
   - Start position: robot's back against the wall at the toggle. Does the
     flapper touch the toggle? (Not allowed at the start, <SG1>.)
   - Tape-measure the wall-stack line: wall face to the goal's edge (RW's model
     says 20.4 in).
   - Have the driver try the wall-stack move: drive down along the wall, spin
     to face it on the stack's line, drive in.
9. **Drive Calibration** auton (once it exists), then on this Mac:
   `sh sim/fit-profile.sh rw_drive_log.csv` so the simulator matches the robot.
10. **Routes**, slow first:
    - Does the claw land on the goal? Adjust `REACH` (6.25) in
      `src/auton/autonroutes.cpp`.
    - Does the piece drop (`CLAW_OPEN_MS`), and does the robot clear the goal
      backing off (`BACK_OFF`)?
    - Time each route; the terminal prints each step's time.

## Autons: what's there and what's wrong

Auton time in RW's simulator (not yet fitted to our robot). The overhead
video's robot (youtube.com/shorts/VCsaetCuP-s) ran Red Left 4's 4 pins in 14.9 s.

| Auton | Does | Sim time | Fits 15 s? |
|---|---|---|---|
| Red Left 3 / Blue Left 3 | toggle, preload → alliance goal, corner Cup+Pin → same goal, Midfield Cup+Pin → black goal | 12.6 s | yes |
| Red Left 4 / Blue Left 4 | the video's route: preload + wall stack → alliance goal, other wall stack → black goal, Midfield Cup+Pin → alliance goal | 21.5 s | no (video robot: 14.9 s) |
| Red Left 5 / Blue Left 5 | Red Left 4 + the Midfield-edge Cup+Pin → black goal | 25.7 s | no (~19 s even at the video robot's pace) |
| Red Right 2 / Blue Right 2 | bottom toggle, preload → alliance goal, corner Cup+Pin → same goal | 8.3 s | yes |
| Skills | Red Left 3 + the bottom toggle (rest not planned) | 16.8 s of 60 | |

- [ ] Loose Cup+Pin positions were measured off the video (about +-3 in):
      check them on the field.
- [ ] Wall-stack move (Red Left 4/5): angled entry, squaring turn, push, grab,
      wall-contact reset, back out. The simulator can't show the squaring, so
      prove it on the field. With the front braces on the wall the claw sits
      0.7 in past the stack's centre: check it still closes.
- [ ] Red Left 4 needs to get ~30% faster to fit; each wall stack takes ~3.3 s
      here vs ~2 s for the video robot.
- [ ] A second video (youtube.com/shorts/pssLb3RYEmM) gets the wall stack
      by turning in the gap beside the goal and carrying it along the wall to
      the other goal. For V4 that turn has ~0.5 in of room: worth comparing.
- [ ] Resets rarely fire in Red Left 3: the robot reaches goals at 15-30 degrees
      off square. Square up before resetting, or accept it.
- [ ] Red Right 2 leaves ~6.7 s unused.
