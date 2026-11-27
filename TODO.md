# 901E practice plan (V4 robot)

Branch `v4-ken` on GitHub (`main` is untouched). Work down the page in order:
each step needs the ones before it. "Paste into" says which line of code a measured
number replaces. Re-upload after each block of pasted numbers.

Everything below was only ever run in the simulator. Expect the first real
runs to be off, and tune from there.

## 0. Before you leave for practice

- [ ] Bring: this Mac with the branch, a tape measure, an SD card in the brain,
      masking tape for marking where the robot stops.
- [ ] Decide how the code gets onto the robot (this Mac, or push the branch).
- [ ] **SD card**: a microSD card formatted FAT32 (16 or 32 GB cards come that
      way), in the brain before it is switched on. Every auton run then saves
      itself to `run_NNN.csv` on the card: where the robot thought it was 20
      times a second, the lift, the claw, the four distance sensors, and every
      line the terminal would have shown. Nothing to turn on. Without a card
      the robot drives the same and saves nothing. Don't pull the card out
      while a program is running.
- [ ] Not written yet (tell Claude if you want it before practice):
      - a brain-screen page that lists what is plugged into each port.
- [ ] **Rules to confirm in the current game manual** (Claude's copy is
      version 1.1 and marked obsolete):
      - May a drive team member hold the loader up while the robot takes a
        piece? Match loading is designed around that.
      - The custom plastic limit. The CAD has 13 custom polycarbonate pieces;
        the old rule allows 12.
      - The Autonomous Win Point conditions, including "neither robot
        contacting the field perimeter" at the end.
- [ ] **Fusion**: the front sleds are cut back flush with the drive channel and
      the official loader is placed in front of the robot ("Loader check").
      Neither edit is saved: save the document or undo them.

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
- [ ] **Turn PID**, then **Move PID**, by hand, with the two test autons in the
      auton picker (no pieces needed):
      - **`Test: turn`**: 90 right twice, 90 left twice, 180 right, 180 left,
        45 right, 45 left, with a pause between. About 13 s, close to Timed
        Run's 15 s, so run it in the controller's **Programming Skills** mode
        to be safe.
      - **`Test: drive`**: 24 in forward and back, twice. About 8 s; Timed Run
        is fine.
      - How teams tune (LemLib's and EZ-Template's guides agree): start with I
        and D at 0; raise P until it overshoots and bounces once or twice;
        raise D until the bounce stops; repeat, a little more P then more D,
        until no amount of D stops it, then go back to the last good pair. Use
        I only as a last resort, if it still ends short.
      - After each move the terminal (and the SD card) gets a line like
        `Test turn +90.0 deg: ended -0.65 deg off, overshot 0.00, settled
        within 1.0 deg after 0.47 s`.
      - Change one number in `rw-config.cpp` (`turn_kp`, `turn_kd`, `turn_ki`
        or `distance_kp`, `distance_kd`, `distance_ki`), upload, run again.
        Raise P until it just overshoots, raise D until the overshoot is gone,
        add a little I only if it keeps ending short. Aim for: overshoot under
        about 1 deg / 0.3 in, and the shortest settle time.
      - Don't use the numbers from the menu's Turn PID / Move PID auto-tuner:
        they come out about ten times too high.
      - `Test: drive` reports the distance the robot measures itself, so do
        Cal dist first and check one run against a tape measure.
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

How the pieces fit together (and where the robot's weight comes in):

- **The drive model** (`drive_kS`, `drive_kV`, `drive_kA`, from
  `Cal: feedforward`): how many volts give how much speed and acceleration. It
  is measured on the real robot, so its weight is included. Run it again if
  the weight changes much.
- **The limits** (top speed, acceleration, cornering): how hard the routes may
  push. They come from the drive log: the real top speed, how fast the robot
  really speeds up, and whether the wheels slip. A heavier robot gets lower
  limits. Most autons use 45 in/s and 100 in/s^2; the solo AWP needs 55 and
  175, which no run on the robot has confirmed yet.
- **The PIDs** (Turn PID, Move PID): corrections for what is left over. They
  make moves land where they should; they do not make the robot faster. The
  on-robot auto-tuner gives gains about ten times too high: tune by hand
  (P, then D, then I) until it is fixed.

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
      `LIFT_DPS` in `src/auton/auton.hpp` (550 is a guess).
- [ ] **Auton lift gain**: autons run the lift harder than the driver presets
      (`LIFT_KP_AUTON` 2.0 in `include/lift.hpp`). Watch one auton lift move
      with a piece: if it overshoots or bounces, lower it toward 0.5.
- [ ] **Claw**: how long it takes to close on a Cup+Pin and to open
      (`CLAW_CLOSE_MS`, `CLAW_OPEN_MS` in `auton.hpp`, both 150), and the
      distance at which the sensor sees a piece (`AUTOGRAB_MM` in `config.hpp`).
- [ ] With the front of the robot flat on a wall and a wall stack in the claw,
      does the claw still close on it and centre it? (The stack sits 0.66 in
      deeper than usual there. The sleds end flush with the frame now, so the
      frame's own front corner is what touches the wall: watch the first grab,
      aluminium may grip the wall more than plastic did.)
- [ ] **Claw on a cup**: close the claw on a cup by hand. A cup is about 2.3 in
      across at jaw height, an inch narrower than a pin's collar, and the CAD
      only shows the jaws open.
- [ ] **Match loading** (driver period): with a drive team member holding the
      loader up, drive in and take a piece.
      - The loader's black rail stays on the wall, 4.65 in wide and 1.3 in
        out. The jaws meet it with the claw about 1.8 to 2.2 in from the wall,
        and the piece sits 1.7 to 2.0 in out: it should reach, only just. The
        robot stops about an inch off the wall, not against it.
      - The Loader Aligner centres the robot to about an inch. The jaws need
        closer than that, so finish lining up by eye.
      - The aligner's flat plate has to flip up out of the raised loader's way.

## 4. Distance sensors (20 min)

Brain screen, middle button = Distance page.

- [ ] Each sensor: park its lens a tape-measured distance from a wall (set the
      distance with left/right), robot square to the wall, wait 2 s. Paste its
      "x" number into `include/resets.hpp`: `SCALE_FRONT` / `BACK` / `LEFT` /
      `RIGHT` (the terminal prints the line ready to paste).
- [ ] Offsets: put the robot at a known spot and check the page's x/y. If it
      is off, the sensor offsets in `rw-config.cpp` are wrong.
- [ ] **When the sensors are used** (copied from how 360H, 2145Z and 3588Y use
      theirs: robot stopped or pressed on something, square to the walls, one
      sensor per axis):
      - Red Left 4, Red Left 5 and the Solo AWP read all four sensors at every
        score, while the robot is pressed on the goal and the claw opens (no
        extra time). The goal seat sets the position; a sensor that agrees to
        within 1.5 in changes nothing, and one that disagrees by 1.5 to 6 in
        overrides the seat on its axis (the robot wasn't really seated).
      - The terminal prints a `Dist ...` line per sensor at every score. Save
        them: "agrees to +0.4 in" is the normal case; "not used (off square)"
        means the robot scored at an angle, which most scores do.
      - Readings are used out to 48 in at these stops (those teams use 20 to
        48 in), and never through a wall stack, loader, goal or toggle that
        should still be in the beam.
      - Red Left 3, Red Right 2 and Skills use the older reset at their stops.
      - RW's MCL stays off: in the simulator it made Red Left 5 miss a wall
        stack, and RW's own notes say it has only been tried in simulation.
- [ ] If the field has clear walls: try each sensor at 6, 12, 24, 36 in and at
      0, 10, 20, 30 degrees off square. Note where readings go wrong and set
      `RESET_MAX_OFF_SQUARE_DEG` in `resets.hpp` (12 now).

## 5. Field measurements (15 min)

- [ ] **Loose Cup+Pins**: tape-measure where each one sits from the two nearest
      walls. The code has them from a video, good to about 3 in, and the claw
      only takes 1.4 in. Paste into `src/auton/auton.hpp`: `DIAG_FAR`, `DIAG_NEAR`,
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
      `src/auton/wallstack.cpp` to that angle, run Red Left 4, and note
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
      `src/auton/flow.cpp`. Then raise it in steps until grabs stop landing every
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
- [ ] **Red Solo AWP** (sim: ends 14.9 s): last, after Red Left 5 works and
      the drive log has confirmed the faster limits. See "Solo AWP" below.
- [ ] Blue versions: the same routes turned 180 degrees. Check one.
- [ ] **Watch log**: every auton prints `Watch ...` lines (what the distance
      sensors think the position error is). It changes nothing. Save the
      terminal output of a few runs for Claude: it decides whether sensor
      corrections can be switched on.

## 7. Bring back

- [ ] Every number you pasted (or just commit the files).
- [ ] `rw_drive_log.csv`.
- [ ] Every `run_NNN.csv` from the SD card, with a note of which run was which
      (the first line of each file names the auton) and where each run ended
      versus where it should have. These replace copying the terminal output.
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
      on the wall). Ends 14.9 s in the simulator, 20 of 20 runs, margin 0.1 s.
      - **Partner**: it must start on the LEFT half of the bottom toggle and
        roll forward off the wall during autonomous (a partner that never
        moves loses the win point). This robot backs onto the right half and
        passes 1.5 in from where the partner would be after rolling 4 in.
      - **Drive limits**: the route uses `DriveLimits fast(55, 175)` in
        `redSoloAwp`. At 52 in/s and 150 in/s^2 it ends at 15.1 s; at 45 and
        100, over 16 s. If the drive log shows the robot can't hold them,
        lower them and expect the route to run long.
      - **Hairpin pickup** (the bottom-edge Cup+Pin, picked up heading north
        before reversing into the toggle): the least forgiving grab. Re-find
        its aim point on the field.
      - **Option to try: drive straight at that piece**, stop, turn on the
        spot to face north, then reverse into the toggle. In the simulator it
        takes the same time and the pickup is far more forgiving, but the
        robot stopped 2.6 in short of the wall, so the flip did not count.
        Test on the robot whether pressing back until the wheels stall puts
        it on the wall every time; if so, tell Claude to switch to it.
      - **Back-in at the bottom toggle**: the robot should end pressed square
        on the wall and hold a light press through the flip ("Back on the
        wall" in the terminal).
      - **If the partner can flip the bottom toggle itself**, tell Claude:
        this robot can then skip it, which is the biggest single saving left.
- [x] **Red Left 5 under 15 s** in the simulator (ends 14.7 s). The margin is
      0.3 s and depends on the real lift, claw and toggle times.
- [ ] **Goal-seat reset** (new, never run on the robot): every score presses
      lightly onto the goal while the claw opens, then sets the robot's
      position from the goal. At practice: watch that the robot really ends
      pressed onto the goal, and measure how far the goal's centre is ahead of
      the robot's centre when seated (`GOAL_SEAT` in `src/auton/flow.cpp`, 6.0 is
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

- **The solo AWP has no slack.** Five deliveries and two toggles fill the 15 s:
  it fits only at drive limits the robot has not shown yet, and with guessed
  times for the toggle flip (0.6 s), the claw (0.15 s) and the lift.
- **Goals are wider than the code assumes near the floor**: about 3.2 in from
  the centre on the diagonals, where the code uses 2.81.
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
