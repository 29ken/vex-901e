# 901E practice plan (V4 robot)

Branch `v4-cad-fixes` (local, not pushed). Work down the page in order: each
step needs the ones before it. "Paste into" says which line of code a measured
number replaces. Re-upload after each block of pasted numbers.

Everything below was only ever run in the simulator. Expect the first real
runs to be off, and tune from there.

## 0. Before you leave for practice

- [ ] Bring: this Mac with the branch, a tape measure, an SD card in the brain,
      masking tape for marking where the robot stops.
- [ ] Decide how the code gets onto the robot (this Mac, or push the branch).
- [ ] Not written yet (tell Claude if you want them before practice):
      - a brain-screen page that lists what is plugged into each port;
      - logging auton runs to the SD card.

## 1. Ports and a first check (15 min)

- [ ] Plug everything in and fill in the port numbers:
      - `src/rw-config.cpp`: drive motors, IMU (port 11 is a guess), the four
        distance sensors.
      - `include/config.hpp`: lift motors, lift sensor, `CLAW_PORT`,
        `AUTOGRAB_SENSOR`.
      - `include/config.hpp`: `RATCHET_MOTOR`, `TOGGLE_SENSOR` (rotation sensor
        on the toggle shaft), `TOGGLE_OPTICAL`. While `RATCHET_MOTOR` is 0 no
        auton flips a toggle: `setToggle()` only waits 0.6 s.
- [ ] **Ratchet check, robot off:** raise the lift by hand: the toggle flapper
      should stay still (pawls clicking). Lower it by hand: the flapper should
      turn. If it is the other way round, the pawls are on backwards for the
      code.
- [ ] **Ratchet check, robot on:** tap B. The flapper should turn half a turn
      and the lift should NOT move. If the lift moves, make `RATCHET_MOTOR`
      negative.
- [ ] Turn the flapper by hand to where the flaps are clear of everything, read
      "Toggle ... deg" on the brain screen, paste into `TOGGLE_REST_DEG` in
      `include/toggle.hpp`.
- [ ] Hold a red, a blue and a yellow toggle face where the optical sensor
      looks: the brain screen should say red / blue / yellow. If it says "?",
      tell Claude the colours it does and doesn't see.
- [ ] Does one flip (half a turn of the flapper) flip a toggle back-on? And
      side-on (the side flaps are single, so it may need a full turn)?
- [ ] Check: the controller says MOTORS 6/6 OK. Push the robot by hand: X, Y
      and H on the brain screen change the right way.

## 2. Drive calibration (30 min). Do all of it before any auton

Controller menu: X opens it, UP steps through it, A starts.

- [ ] **Cal turn**: spin the robot exactly 360 degrees by hand, press A.
      Paste into `rw-config.cpp`: `imu_scale`.
- [ ] **Cal dist**: push the robot exactly 48 in, press A.
      Paste into `rw-config.cpp`: `wheel_distance_in`.
- [ ] **Turn PID**, then **Move PID** (driver control enabled, room to move).
      Paste the P/I/D from the brain screen into `rw-config.cpp`.
- [ ] Auton **`Cal: feedforward`** (auton picker), 4 ft clear in front and
      behind. Copy the three numbers from the brain into `drive_kS`,
      `drive_kV`, `drive_kA` in `rw-config.cpp`. It never ends: stop the
      program once the numbers show. If our brain pages cover them, read them
      from the PROS terminal. **The curved autons (Red Left 4/5) will not drive
      properly without this.**
- [ ] Auton **`Cal: drive log`**, 4 ft clear all round, about 15 s. Saves
      `rw_drive_log.csv` to the SD card. On the Mac:
      `sh sim/fit-profile.sh rw_drive_log.csv`. Keep the file: Claude re-runs
      the route search against it so the simulator drives like our robot.

## 3. Lift and claw (20 min)

- [ ] **Lift heights**: raise by hand with R2, read "Lift" on the brain screen.
      Paste into `include/config.hpp`:
      - `LIFT_GRAB`, `LIFT_CARRY`
      - `LIFT_ALLIANCE_GOAL`: claw over an empty red goal
      - `LIFT_NEUTRAL_GOAL`: claw over a black goal **with its starting yellow
        pin on it** (every black goal starts with one)
      - `LIFT_PER_PIECE`: how much higher for each piece already stacked
      - `LIFT_PRESETS`
- [ ] **Lift speed**: time the lift from the bottom to the third-piece height
      and back down, with a piece in the claw. Paste units per second into
      `LIFT_DPS` in `src/auton/autonroutes.cpp` (550 is a guess).
- [ ] **Auton lift gain**: autons run the lift harder than the driver presets
      (`LIFT_KP_AUTON` 2.0 in `include/lift.hpp`). Watch one auton lift move
      with a piece: if it overshoots or bounces, lower it toward 0.5.
- [ ] **Claw**: how long it takes to close on a Cup+Pin and to open
      (`CLAW_CLOSE_MS`, `CLAW_OPEN_MS` in `autonroutes.cpp`, both 150), and the
      distance at which the sensor sees a piece (`AUTOGRAB_MM` in `config.hpp`).
- [ ] With the front of the robot flat on a wall and a wall stack in the claw,
      does the claw still close on it and centre it? (The stack sits 0.66 in
      deeper than usual there.)

## 4. Distance sensors (20 min)

Brain screen, middle button = Distance page.

- [ ] Each sensor: park its lens a tape-measured distance from a wall (set the
      distance with left/right), robot square to the wall, wait 2 s. Paste its
      "x" number into `include/resets.hpp`: `SCALE_FRONT` / `BACK` / `LEFT` /
      `RIGHT` (the terminal prints the line ready to paste).
- [ ] Offsets: put the robot at a known spot and check the page's x/y. If it
      is off, the sensor offsets in `rw-config.cpp` are wrong.
- [ ] If the field has clear walls: try each sensor at 6, 12, 24, 36 in and at
      0, 10, 20, 30 degrees off square. Note where readings go wrong and set
      `RESET_MAX_OFF_SQUARE_DEG` in `resets.hpp` (12 now).

## 5. Field measurements (15 min)

- [ ] **Loose Cup+Pins**: tape-measure where each one sits from the two nearest
      walls. The code has them from a video, good to about 3 in, and the claw
      only takes 1.4 in. Paste into `autonroutes.cpp`: `DIAG_FAR`, `DIAG_NEAR`,
      `MID_EDGE`, `MID_BOTTOM` (inches from the field centre; the walls are at
      70.25).
- [ ] **Wall stacks**: wall face to the centre of the stack (`STACK_FROM_WALL`,
      1.55) and wall face to the edge of the goal in front of it (the code
      assumes 20.4 in).
- [ ] **Start position**: back of the robot on the wall at the toggle. The
      flapper must not touch the toggle at the start (<SG1>).
- [ ] Which way up are the cups in the wall stacks and loose Cup+Pins (gray
      half up or clear half up)? The scoring count assumes gray up.
- [ ] **Wall stack at an angle** (what 1698Z and 8275K do, no extended sleds):
      drive at a wall stack by hand so one front corner touches the wall, and
      find how far off square the robot can be with the claw still closing
      cleanly on the stack. The math says best at about 6 degrees, usable to
      about 10. Then set `WALL_GRAB_ANGLE_DEFAULT` in
      `src/auton/autonroutes.cpp` to that angle, run Red Left 4, and note
      where the robot is after each wall stack (the routes after the grab were
      tuned for a square robot and will need adjusting).
- [ ] How long does a real toggle flip take (B on the controller, time it)?
      Paste into `TOGGLE_FLIP_MS` in `include/toggle.hpp` (600 is a guess; the
      simulator uses it). The solo AWP route needs this.

## 6. Run the autons, slowest and simplest first

For each: run it 3-5 times, mark where the robot stops with tape, and read the
terminal (it prints each step's time and what the claw sensor saw).

- [ ] **Red Right 2** (2 pieces, sim 8.4 s) and **Red Left 3** (3 pieces, sim
      12.4 s). Straight moves with turns: the easiest check that calibration is
      good. Does the claw land on the goal (`REACH`, 6.25)? Does the piece drop
      and the robot clear the goal backing off (`BACK_OFF`)?
- [ ] **Red Left 4** (4 pieces, sim: all on by 13.4 s). Curved, with two wall
      stacks. First with `TOP_IPS` lowered from 45 to about 25 in
      `autonroutes.cpp`. Then raise it in steps until grabs stop landing every
      time, and back off one step.
      - Loose piece missed the same way every run: shift that leg's points.
      - Scattered: that curve is too fast; lower the speed number on its points.
      - Wall stack off-centre the same way every run: set `WALL_AIM_SHIFT`.
      - The corner doesn't hold while squaring up: change `WALL_TURN_RADIUS`.
      - The spin after a wall stack touches the goal or the wall: that is the
        0.3-0.4 in gap (see "Known limits").
- [ ] **Red Left 5** (5 pieces, sim: all on by 14.6 s, ends 14.7 s). Same
      tuning as Red Left 4. It has several aim points corrected by up to
      0.7 in in the simulator: expect to re-do those.
- [ ] **Red Solo AWP**: run it after Red Left 4 works. Time the bottom-toggle
      part on its own, and try the toggle side-on by hand to see what it needs.
- [ ] Blue versions: the same routes turned 180 degrees. Check one.
- [ ] **Watch log**: every auton prints `Watch ...` lines (what the distance
      sensors think the position error is). It changes nothing. Save the
      terminal output of a few runs for Claude: it decides whether sensor
      corrections can be switched on.

## 7. Bring back

- [ ] Every number you pasted (or just commit the files).
- [ ] `rw_drive_log.csv`.
- [ ] Terminal output of the auton runs, and where each run ended versus where
      it should have.
- [ ] A video of one wall-stack grab from above, if you can.

## Autons in the code (simulator numbers)

| Auton | Pieces placed | Sim result |
|---|---|---|
| Red/Blue Left 3 | preload, corner Cup+Pin → red goal; Midfield Cup+Pin → black goal | 3 placed, ends 12.4 s |
| Red/Blue Left 4 | preload, wall stack → red goal; other wall stack → black goal; Midfield Cup+Pin → red goal | 4 placed by 13.4 s (20 of 20 runs) |
| Red/Blue Left 5 | preload, wall stack, corner Cup+Pin → red goal; Midfield Cup+Pin, other wall stack → black goal | 5 placed by 14.6 s, ends 14.7 s (20 of 20 runs) |
| Red/Blue Right 2 | bottom toggle, preload, corner Cup+Pin → bottom red goal | 2 placed, ends 8.4 s |
| Skills | Red Left 3, then the bottom toggle | 16.7 s of 60; the rest isn't planned |
| Red/Blue Solo AWP | preload, Midfield Cup+Pin → red goal; Midfield-edge Cup+Pin → left black goal; bottom-edge Cup+Pin carried into the bottom toggle (back-in), then → bottom black goal; wall stack → bottom black goal | ends 14.9 s at 55 in/s and 175 in/s² (20 of 20 runs); 15.1 s at 52 / 150 |
| `Cal: feedforward`, `Cal: drive log` | set-up runs, not match autons | |

"Placed" is pieces, not points: a Scored Pin is a visible pin half. Each black
goal starts with a yellow pin; yellow halves count only when that quadrant's
toggle is ours.

## Not done yet

- [ ] **Solo AWP** (Worlds-qualifying: 7 halves, 3 goals with 2+, neither robot
      on the wall). New route: ends 14.9 s in the simulator, 20 of 20 runs.
      - The partner robot must start on the LEFT half of the bottom toggle and
        roll forward off the wall during autonomous. This robot backs onto the
        right half.
      - It needs faster drive limits (`DriveLimits fast(55, 175)` in
        `redSoloAwp`): 55 in/s and 175 in/s^2 against 45 and 100 elsewhere.
        Bring back the drive log so these can be checked. If the robot can't
        hold them, lower them: at 52 / 150 the route ends at 15.1 s.
      - Re-find the aim point for the bottom-edge Cup+Pin (the hairpin pickup
        before the bottom toggle). It is the least forgiving grab.
      - Watch the back-in at the bottom toggle: the robot should end pressed
        square on the wall ("Back on the wall" in the terminal).
- [x] **Red Left 5 under 15 s** in the simulator (ends 14.7 s). The margin is
      0.3 s and depends on the real lift, claw and toggle times.
- [ ] **Goal-seat reset** (new, never run on the robot): every score presses
      lightly onto the goal while the claw opens, then sets the robot's
      position from the goal. At practice: watch that the robot really ends
      pressed onto the goal, and measure how far the goal's centre is ahead of
      the robot's centre when seated (`GOAL_SEAT` in `autonroutes.cpp`, 6.0 is
      from the CAD). The terminal prints "Goal seat: ..." each time.
- [ ] **Wall stacks** now grab as soon as the robot is within 3 degrees of
      square (`WALL_SQUARE_EXIT_DEG`), with no push (`WALL_PUSH_MS` 0). If the
      claw closes before the stack is in, raise the push to 80.
- [ ] Toggle / ratchet / optical code is written but has never run on the
      robot (ports are 0). `RATCHET_ASSIST` (ratchet motor helps the lift up) is
      off until the hand check passes.
- [ ] Sensor corrections while driving: the watcher only logs. MCL after that.
- [ ] The old failing lift test (`test/lift_check.cpp:109`).

## Known limits

- **V4 is long for the gap beside a goal.** Every wall stack sits in front of a
  goal. After a grab the robot spins 10.1 in from the wall with 0.3-0.4 in to
  the goal and the wall. Back corners at 9.43 in from the centre instead of
  9.8 (back 0.5 in shorter, or about 0.6 in cut off each back corner) would
  give 0.75 in each side. That is a CAD decision.
- **The wall-stack grab depends on the wall.** How the corner slides while
  squaring up can't be simulated; the simulator's two wall models disagree by
  almost 2 in. A missed wall stack costs the rest of that route.
- **The claw's U-opening has 0.19 in to spare each side of a goal.** A score
  that arrives off line puts an arm on the goal.
- **No tracking wheels.** The robot can't see itself slide sideways in fast
  bends. That error builds through each curve and is only corrected at wall
  stacks.
- **RW's curve follower is new** and, by RW's own notes, not tried on many
  robots.
