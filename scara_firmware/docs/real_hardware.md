# Driving the real SCARA from MoveIt

How the simulation stack is pointed at real hardware, what you have to wire, and
what to check when it misbehaves.

```
RViz / MoveIt  ->  move_group  ->  arm_controller (JointTrajectoryController)
               ->  ros2_control controller_manager
               ->  ScaraHardwareInterface      (scara_firmware, C++)
               ->  USB serial, 115200
               ->  scara_moveit_control.ino    (Arduino Uno + CNC Shield V3)
               ->  A4988 drivers  ->  NEMA-17  ->  the arm
```

---

## 1. Quick start

```bash
# once: let your user talk to the serial port, then log out and back in
sudo usermod -aG dialout $USER

# flash firmware/scara_moveit_control/scara_moveit_control.ino
#   Arduino IDE -> Library Manager -> install "AccelStepper"
#   Board: Arduino Uno

# park the arm at its zero pose BY HAND, then power the drivers
ros2 launch scara_bringup real_bringup.launch.py serial_port:=/dev/ttyACM0
```

Plan and execute in the RViz MotionPlanning panel; the arm follows.

Add `use_slider:=True` for a Joint State Publisher GUI that drives the real arm
through the same controllers.

The simulation is unchanged and still lives at:

```bash
ros2 launch scara_bringup simulation_bringup.launch.py
```

---

## 2. This is an open-loop system

There are no encoders. Nothing ever tells ROS where the arm really is.

`ScaraHardwareInterface::read()` is, in full:

```cpp
position_states_ = position_commands_;
```

That single line is the whole model: **ROS believes the arm is wherever it last
told it to go.** RViz, `/joint_states`, `/tf` and MoveIt's start state are all
downstream of that belief.

Consequences worth internalising:

- **You must park the arm at zero before power-on.** The firmware calls
  `setCurrentPosition(0)` on boot, so whatever pose it is in becomes "zero". If
  it was somewhere else, every later pose is offset by that error, permanently.
- **Skipped steps are silent and cumulative.** If a motor stalls, RViz keeps
  showing the arm where it *should* be. The only fix is to re-park and restart.
- **Execution always reports success.** Even with the drivers unplugged, MoveIt
  will report the trajectory as executed. Watch the arm, not the log.
- **`/joint_states` is fiction.** `scara_vision`, `scara_conveyor` and
  `scara_voice` all consume it. A blind pick that works in simulation can miss on
  hardware if the arm has drifted.

If you later add limit switches or AS5600 encoders, this is the one function that
has to change.

---

## 3. Wiring

CNC Shield V3 on an Arduino Uno. Same pin map as the other two sketches in this
package.

| Joint | Type | Shield axis | STEP | DIR | Units |
|---|---|---|---|---|---|
| `column_joint` | revolute | X | D2 | D5 | rad |
| `shoulder_joint` | prismatic | Y | D3 | D6 | m (leadscrew) |
| `forearm_joint` | revolute | Z | D4 | D7 | rad |
| `wrist_joint` | revolute | A | D12 | D13 | rad |
| `left_finger_joint` | servo | — | **D11** (Z+ header) | — | m |
| conveyor belt relay | on/off | — | **D10** (Y+ header) | — | — |

`ENABLE` is D8 and is **active LOW** — LOW energises the drivers.

The A axis needs the D12/D13 jumpers fitted on the shield.

### The gripper servo

The shield only breaks out four STEP/DIR channels and the arm uses all four, so
the gripper is a hobby servo. Its signal wire goes to the **Z+ endstop header**,
which is **Arduino pin D11**:

| Endstop header | Arduino pin |
|---|---|
| X+ / X− | D9 |
| Y+ / Y− | D10 |
| Z+ / Z− | **D11** |

The `+` and `−` pads of one axis are the same Arduino pin, wired in parallel.
The endstop headers carry only signal and GND, so the servo's 5V comes from the
shield's power header.

**To find the open and closed angles**, flash `firmware/servo_test/servo_test.ino`
and open the Serial Monitor at 115200 with "Newline" set. Type an angle, or `+`
and `-` to nudge it, or `s` to sweep slowly end to end. It prints the angle
alongside the gripper opening in metres that ROS would call it, so you can read
off the two numbers for `GRIPPER_OPEN_ANGLE` and `GRIPPER_CLOSED_ANGLE`.

> A servo has no position sensor. `Servo.read()` returns the last value you
> *wrote*, not a measurement — so if the gripper jams, the printed angle still
> looks perfect. The test sketch prints both numbers next to each other to make
> that obvious.

Two consequences worth remembering:

- **D11 is now taken.** Z-axis limit-switch homing would need a different pin.
- **Do not power a stalling servo from the Uno's regulator.** A gripper that
  clamps hard and holds can pull enough current to brown out the board, which
  looks exactly like random resets — the firmware reprints its banner and `R`,
  and ROS logs `Firmware ready` in the middle of a move. Give the servo its own
  5V supply (grounds tied together) if that happens.

`right_finger_joint` is a URDF `<mimic>` of the left one, so it has no motor —
`robot_state_publisher` draws it for you.

---

## 4. Serial protocol

Line-based ASCII at 115200 8N1. Deliberately the same frame format as
`manual_teaching_mode.ino`, just travelling the other way.

**ROS to Arduino**

| Message | Meaning |
|---|---|
| `J <column> <shoulder> <forearm> <wrist> <gripper>\n` | Target pose, SI units (rad, m, rad, rad, m) |
| `H\n` | Treat the current pose as zero (re-home in place) |
| `F\n` | Free the drivers — the arm can be pushed by hand |
| `L\n` | Lock the drivers — holding torque on (power-on default) |
| `C 1\n` / `C 0\n` | Run / stop the conveyor belt relay |
| `?\n` | Print the banner |

**Arduino to ROS**

| Message | Meaning |
|---|---|
| `R\n` | Booted, ready for targets |
| `B\n` | Target accepted, motors moving (BUSY) |
| `D\n` | Every axis has arrived (DONE) |
| `# ...` | Human-readable comment, logged by the ROS node |

You can talk to the board by hand with any serial monitor at 115200:

```
J 0.5 -0.05 0.0 0.0 0.0
```

### How the handshake works

`write()` runs inside the `controller_manager` update loop. Blocking there on a
serial reply would stall every controller in the process, so it never waits — it
**skips**:

```cpp
if (busy_) return;                       // Arduino still moving: try next cycle
if (command unchanged) return;           // keep the wire quiet while idle
send("J ...");  busy_ = true;            // 'D' from the firmware clears busy_
```

The result is genuine point-by-point synchronisation — the Arduino is never asked
to do two things at once — with zero blocking in ROS. If the firmware goes quiet,
a 5 second timeout releases the gate so a dead sketch cannot freeze the arm
forever; you will see:

```
[WARN] No 'DONE' from the Arduino after 5 s -- is the right sketch flashed?
```

---

## 5. Calibration

Everything mechanical lives in one block near the top of the sketch:

```cpp
#define MICROSTEPS      4.0f     // jumper on M1 only -> 1/4 step
#define COLUMN_GEAR    19.0f     // motor revs per joint rev
#define FOREARM_GEAR   16.0f     // motor revs per joint rev
#define WRIST_GEAR      1.0f     // UNVERIFIED placeholder
#define SHOULDER_LEAD_M 0.008f   // leadscrew travel per motor rev
```

which produce:

| Joint | Gearing | Steps per unit |
|---|---|---|
| column | 19:1 | 2419 steps/rad |
| shoulder | 8 mm lead | 100000 steps/m |
| forearm | 16:1 | 2037 steps/rad |
| wrist | 1:1 **(TBC)** | 127 steps/rad |

Each is `800 x gearing / 2pi`, or `800 / lead` for the leadscrew.

### Shoulder lead: measured, but left at 8 mm

Driving **4600 microsteps** = 4600/800 = **5.75 motor turns** moved the nut
**45 mm**, which implies 45/5.75 = **7.83 mm per turn**, 2.2% short of the 8 mm
the sketch assumes (it predicts 46 mm).

The constant is deliberately *not* changed. 7.83 mm is not a lead that is
manufactured — a T8 screw is 2 mm pitch x 4 starts = exactly 8 mm — and 1 mm out
of 45 is inside ruler resolution plus backlash. To settle it, drive **12000
steps** (15 turns) in one direction only, without reversing, and measure:

| Measured | Verdict |
|---|---|
| ~120 mm | 8 mm lead is correct, leave everything alone |
| ~117.4 mm | lead really is 7.83 mm — see below |

If it turns out to be 7.83 mm, three numbers change together:
`SHOULDER_MM_PER_TURN` 0.008 -> 0.00783, steps/m 100000 -> 102222, and
`shoulder_joint: max_velocity` in `joint_limits.yaml` 0.012 -> 0.0117 m/s.

> **The wrist ratio is a placeholder.** Until it is measured the wrist will move
> by the wrong amount. Everything else is set from measured hardware.

### Shoulder travel: the URDF is deliberately the smaller number

The screw is 400 mm long with ~250 mm of usable travel, but `shoulder_joint` in
the URDF spans only `[-0.15, 0.02]` = **0.17 m**. That gap is intentional — the
robot model is kept as it is in simulation, and the extra ~80 mm of screw is
simply never driven.

Keeping it that way has two advantages:

- **Mechanical margin at both ends.** An open-loop arm has no idea where its hard
  stops are; running the nut into one is how you lose steps silently. Leaving
  travel unused means a miscount has somewhere to go.
- **Nothing downstream has to be re-derived.** Extending the range would change
  reachability, and every existing pose in `scara_conveyor` and `scara_vision`
  was tuned against the current model.

The limit is enforced in three places, all agreeing on `[-0.15, 0.02]`: the URDF
joint, `JOINT_MIN/JOINT_MAX` in this sketch, and the manual-teaching bridge.

> One pre-existing inconsistency, harmless today: the `<command_interface>` in
> `ros2control.xacro` annotates the shoulder as `max = 0.0` rather than `0.02`,
> so it advertises 0.15 m where the URDF joint allows 0.17 m. This interface does
> not read those params, and the firmware clamps to the URDF values, so nothing
> is affected. Left alone rather than "fixed", because `gz_ros2_control` may
> enforce them and changing it would alter simulation behaviour.

### If a joint turns the wrong way

Change that joint's direction constant in the sketch from `1` to `-1`:

```cpp
const int COLUMN_DIRECTION   =  1;
const int SHOULDER_DIRECTION = -1;   // measured on this robot: needs reversing
const int FOREARM_DIRECTION  =  1;
const int WRIST_DIRECTION    =  1;
```

Prefer this to swapping the DIR wire on the shield: it is a one-character change,
it cannot be done wrong, and it keeps the wiring table above honest.

The direction is folded into the steps-per-unit constants, so a negative value
just means the motor spins the other way. It does **not** affect the joint
limits, which are checked in joint units before the conversion happens.

### Microstepping jumpers

Set by the M0/M1/M2 jumpers under each driver. This robot has a jumper on
**M1 only** on all four axes — 1/4 step, on both the A4988 and the DRV8825:

| M0 | M1 | M2 | Resolution |
|---|---|---|---|
| — | — | — | full step |
| ✔ | — | — | 1/2 |
| — | **✔** | — | **1/4** — this robot |
| ✔ | ✔ | — | 1/8 |
| ✔ | ✔ | ✔ | 1/16 (A4988) |

200 full steps/rev x 4 = **800 microsteps/rev**, which is what
`Scara_Stepper_Test.ino` already assumed (`microstepCount / 800.0` revolutions).

If you change a jumper you must change `MICROSTEPS` **and** revisit the shoulder
speed limit in `joint_limits.yaml`, since steps/m scales with it. Dropping the
shoulder to full step, for example, gives 25000 steps/m instead of 100000, so the
same 1200 steps/s becomes 0.048 m/s instead of 0.012 — four times faster, at the
cost of coarser resolution and more vibration.

For comparison, the same robot at **full step** (no jumpers, 200 steps/rev)
would give a quarter of the resolution everywhere:

| Joint | 1/4 step (this robot) | full step |
|---|---|---|
| column | 2419 steps/rad | 605 steps/rad |
| shoulder | 100000 steps/m | 25000 steps/m |
| forearm | 2037 steps/rad | 509 steps/rad |
| wrist | 127 steps/rad | 32 steps/rad |

**Check one axis at a time.** Send `H`, then command exactly one revolution and
see whether the joint agrees:

```
H
J 6.2832 0 0 0 0
```

If the column turns further than one full turn, `COLUMN_GEARING` is too large.

### The easy way: use the control panel

```bash
ros2 run scara_firmware scara_gui.py       # or: python3 tools/scara_gui.py
```

A small window with a slider per joint (limited to the URDF's range), an
open/close toggle for the gripper, and buttons for `H` / `F` / `L`. It talks
straight to the Arduino — no ROS, no MoveIt — so you can use it before anything
else is running. **Close it before launching `real_bringup.launch.py`**: only one
program can hold the serial port at a time.

Section 5 of the window does the gearing sum for you. Move a joint, measure how
far it *really* went, type both numbers in, and it prints the corrected value:

```
asked for 90, really moved 72
  -> COLUMN_GEARING should be about 23.75  (it is 19.0)   ->  3024 steps per radian
```

For the shoulder it corrects `SHOULDER_LEAD_M` instead, since that axis has a
leadscrew rather than a gear.

The joint limits and gearings are repeated at the top of `scara_gui.py`. Keep
them in step with the sketch.

---

## 6. Speed limits — read this before raising anything

An Uno running AccelStepper tops out around **4000 steps/s in total** across all
axes. The per-axis caps in the sketch keep the arm inside that budget:

```cpp
static const float MAX_SPEED[] = {1000, 1200, 800, 500};   // steps/s
```

The ceiling is enforced **in the firmware, not in MoveIt**, so the simulation
still runs at full speed. If a move looks slower on the real arm than in RViz,
this is why.

### Three joints carry a simulation/real swap

Gazebo will move a joint as fast as you ask; the real arm cannot. With 19:1 on
the column and 16:1 on the forearm, three of the four joints are far outside what
an Uno can step at their simulated limits, so
`scara_moveit/config/joint_limits.yaml` carries both values and you uncomment the
one you are running:

```yaml
  column_joint:
    # max_velocity: 1.5      # simulation
    max_velocity: 0.41       # real hardware  (19:1, 1000 steps/s cap)
  forearm_joint:
    # max_velocity: 1.5      # simulation
    max_velocity: 0.39       # real hardware  (16:1, 800 steps/s cap)
  shoulder_joint:
    # max_velocity: 0.5      # simulation
    max_velocity: 0.012      # real hardware  (8 mm lead, 1200 steps/s cap)
```

**Real hardware is active.** Every value is derived from the firmware's own cap:

| Joint | Steps per unit | Firmware cap | = real limit | Simulated | Overstated by |
|---|---|---|---|---|---|
| column | 2419 /rad | 1000 steps/s | **0.41 rad/s** | 1.5 | 3.6x |
| shoulder | 100000 /m | 1200 steps/s | **0.012 m/s** | 0.5 | 42x |
| forearm | 2037 /rad | 800 steps/s | **0.39 rad/s** | 1.5 | 3.8x |
| wrist | 127 /rad | 500 steps/s | 3.93 rad/s | 1.5 | — (no swap needed) |

The wrist needs no swap **only because its ratio is assumed to be 1:1**. If it
turns out to be geared, recompute it: at 10:1 the real limit would be
500 / 1273 = 0.39 rad/s and it would need a swap like the others.

Leaving a simulation value active on the real arm does not break anything — the
firmware still clamps the motion — but MoveIt then reports a move finished while
the arm is still travelling.

Full-travel times at the real caps: column 2 rad in ~5 s, forearm 2 rad in ~5 s,
shoulder 0.17 m in ~14 s.

### Watch out for the scaling factor

The top of the same file has:

```yaml
default_velocity_scaling_factor: 0.1
```

MoveIt multiplies the limits above by this, so the shoulder actually plans at
0.012 x 0.1 = **0.0012 m/s** — over two minutes for its full travel, and the
column and forearm are ten times slower than the table above. That is almost
certainly slower than you want. Raise the **Velocity Scaling** slider in the RViz
MotionPlanning panel (or pass a higher `max_velocity_scaling_factor` in your
motion requests) rather than inflating the limits — the limit describes the
hardware, the slider describes how hard you want to push it.

### If the arm is too slow, change the hardware, not the numbers

The firmware caps sum to 3500 steps/s, close to the ~4000 steps/s an Uno can
sustain across four axes, so there is little headroom to simply raise them. The
real levers are:

- **Coarser microstepping** on the slow axes. Full step instead of 1/4 quarters
  the steps per unit, so the same cap gives 4x the joint speed — at the cost of
  resolution and more vibration. Update `MICROSTEPS` and these limits together.
- **A faster board.** An ESP32 steps far quicker and the ROS side does not change
  at all.

**Raise a speed only as far as the motors follow without skipping steps.** A
skipped step is invisible to ROS and corrupts every pose after it.

---

## 7. Troubleshooting

| Symptom | Cause |
|---|---|
| `Cannot open /dev/ttyUSB0` | Wrong port, or you are not in `dialout`. An Uno is usually `/dev/ttyACM0`. Check with `ls /dev/tty{USB,ACM}*` |
| `No 'DONE' from the Arduino after 5 s` | Wrong sketch flashed, or the serial monitor in the Arduino IDE is holding the port |
| RViz moves, arm does not | Drivers not powered, `ENABLE` jumper missing, or current-limit pots turned all the way down |
| A joint moves the wrong way | Change its `*_DIRECTION` from `1` to `-1` in the sketch (see below) |
| Motion is jerky or stalls | `MAX_SPEED` / `ACCELERATION` too high, or driver current too low |
| Everything is offset after a while | Skipped steps. Re-park at zero and restart |
| Arm and RViz disagree from the start | The arm was not at its zero pose at power-on |

Useful checks:

```bash
ros2 control list_hardware_components     # expect RobotSystem -> active
ros2 control list_controllers             # all three active
ros2 topic echo /joint_states             # the commands, echoed back
```

---

## 8. The conveyor belt

The belt is a mains motor behind a relay, so the only thing to say to it is on or
off. Its signal wire goes to the **Y+ endstop header, Arduino D10** — free
because the four steppers use D2/D5, D3/D6, D4/D7 and D12/D13, and the gripper
servo has D11.

### Why it goes down the arm's cable

`ScaraHardwareInterface` opens `/dev/ttyACM0` in `on_activate()` and holds it for
as long as the controllers run. A serial port has exactly one owner, so no
separate node can open it to say "stop the belt".

The hardware interface therefore carries the message itself. It runs a small
node of its own on a background thread (`src/conveyor_relay.cpp`), listens on
`/conveyor/run`, and writes `C 1` / `C 0` on the wire it already owns:

```
conveyor_state_machine --> /conveyor/run --> ScaraHardwareInterface
                            (std_msgs/Bool)      |  "C 0"
                                                 v
                                          Arduino D10 --> relay --> belt
```

Three things about it are deliberate:

- **The belt command goes out first, before every early return in `write()`.**
  Stopping the belt must not have to wait behind an idle arm or the `DONE`
  handshake — it is the one urgent message on this wire.
- **`/conveyor/run` is latched** (reliable + transient-local, depth 1), matching
  `conveyor_state_machine.py`. So the interface learns the current belt state the
  moment it comes up, without waiting for the next change. **Any publisher on
  this topic must be latched too**, or the messages never arrive.
- **The belt is stopped on activate and on deactivate.** Nothing should be left
  running once the controllers have gone away, because then nothing is left that
  could switch it off. The sketch also boots with the relay off.

### Testing the relay on its own

No ROS needed. `ros2 run scara_firmware scara_gui.py`, connect, and use the
conveyor button. If it runs when it says stopped, flip `CONVEYOR_ACTIVE_LOW` in
the sketch — most little relay boards switch on when pulled LOW, but not all.

### The sorting cell

```bash
ros2 launch scara_bringup real_conveyor.launch.py serial_port:=/dev/ttyACM0
ros2 run scara_conveyor hardware_pick_executor --ros-args -p target_color:=R
```

`conveyor_color_gate` (in `scara_vision`) watches the belt camera and publishes
`/conveyor/box_at_pickup` when a box of the wanted colour has its middle within
`center_tolerance_px` of the middle of the picture, for a few frames running.
That is the same message the simulated belt sends, so `conveyor_state_machine.py`
runs **completely unchanged** — freeze the belt, trigger the pick, wait for
`/scara/pick_completed`, start the belt, repeat. The simulation and the real cell
are one pipeline with a different pair of eyes.

HSV ranges live in `scara_vision/hardware_color_publisher.py`, which has sliders
for tuning them; the gate reads the same table.

### The pick: blind, because it can afford to be

`hardware_pick_executor.py` never looks at the box it is picking, and does not
need to — the gate stops every box in the same place, under the camera centre.
That suits real hardware better than looking would: the arm covers the box as it
comes down, so a camera would lose sight of it at the worst possible moment.

**The drop positions are the simulation's**, imported from
`conveyor_pick_executor.py` rather than copied, so retuning Red there moves the
real cell too — and Yellow and Blue stay exact mirrors of Red and Green.

The pick pose comes from one of two places:

| `pick_source` | Where the pose comes from |
|---|---|
| `ik` (default) | MoveIt solves for the same belt pickup point the simulation uses, `common.pickup_grasp_point_base()` |
| `taught` | The four numbers in `pick_approach` — park the arm over a box with the sliders or `scara_gui.py` and read them off |

```bash
ros2 run scara_conveyor hardware_pick_executor --ros-args \
    -p target_color:=R -p pick_source:=taught \
    -p pick_approach:='[0.0, 0.0, 0.0, 0.0]'
```

The grasp copies the simulation's trick: hold the approach pose and wind only
the shoulder down to the bottom of its travel, so nothing can swing sideways
into the box on the way down.

Two parameters worth knowing about:

- `grip_closed` defaults to the simulation's −0.03 m. The firmware maps −0.06 m
  to a fully shut gripper, so a real box may well need more than the simulated
  one. Measure it and set it.
- `velocity_scaling` defaults to **1.0**, not the 0.1 in `joint_limits.yaml`.
  That file already holds the real stepper speeds, so 1.0 means "as fast as the
  motors can manage", and 0.1 would be ten times slower again.

---

## 9. Making RViz move at the arm's speed

The arm in RViz has no motors, so it arrives instantly. The real one does not:
the shoulder's full 0.15 m takes about **twelve and a half seconds**, because
1200 steps/s over a 100000 steps/m leadscrew is 0.012 m/s.

`scara_controller/realistic_slider_controller.py` closes the gap. It puts a
pretend arm between the sliders and the controllers, obeying the same top speeds
and speed-up rates as the sketch (`stepper_profile.py` holds the numbers, copied
from it):

```bash
# real arm; RViz and the metal now arrive together
ros2 launch scara_bringup real_bringup.launch.py serial_port:=/dev/ttyACM0 \
    use_moveit:=False use_slider:=True

# no robot at all, just to see how long things really take
ros2 launch scara_description display.launch.py realistic:=True
```

**This is not feedback.** Nothing asks the arm where it is — it has no encoders
and cannot answer. It is the Arduino's own arithmetic, run on this side of the
cable. If the arm skips steps, RViz will not know, exactly as before.

Add `realistic:=False` for the old instant behaviour, or `speed_scale:=4.0` to
preview a long move without waiting for it. MoveIt needs none of this: it already
time-parameterises against the real velocities in
`scara_moveit/config/joint_limits.yaml`.

`stepper_profile.py` must be kept in step with the sketch — change a gear ratio
or a top speed there and change it here, or RViz starts lying again.

---

## 10. Where things live

| Path | What |
|---|---|
| `scara_firmware/src/scara_hardware_interface.cpp` | The interface. `read()`/`write()` are the whole story |
| `scara_firmware/firmware/scara_moveit_control/` | The sketch to flash |
| `scara_description/urdf/ros2control.xacro` | The `is_sim` / `is_real` switch that picks the backend |
| `scara_bringup/launch/real_bringup.launch.py` | Real robot |
| `scara_bringup/launch/simulation_bringup.launch.py` | Gazebo |
| `scara_firmware/src/conveyor_relay.cpp` | Listens on `/conveyor/run`, so the belt can share the arm's port |
| `scara_controller/scara_controller/stepper_profile.py` | The stepper speeds, mirrored from the sketch |
| `scara_vision/scara_vision/conveyor_color_gate.py` | Says when a wanted box is centred in the picture |
| `scara_conveyor/scara_conveyor/hardware_pick_executor.py` | Blind pick at the belt centre; sim's drop positions |
| `scara_bringup/launch/real_conveyor.launch.py` | The whole sorting cell on real hardware |

Related: `manual_teaching_mode.ino` streams joint values *out* of the arm using
the same `J` frame, for lead-through teaching.

---

## 11. Possible next steps

- **Limit switches for homing.** The shield has endstop headers on D9 and D10 —
  though D10 now drives the conveyor relay, so an X endstop on D9 is the one
  going spare. This would remove the park-at-zero ritual, the single biggest
  weakness here.
- **A hardware E-stop** in series with `ENABLE`. Software cannot guarantee a stop.
- **Encoders (AS5600 + TCA9548A)** would turn `read()` into a real measurement
  and make this a closed-loop robot.
- **An ESP32** if you run out of step rate; the ROS side would not change at all.
