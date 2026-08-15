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
#define COLUMN_GEAR    20.0f     // motor revs per joint rev
#define FOREARM_GEAR   17.0f     // "roughly 17:1" -- verify
#define WRIST_GEAR      1.0f     // UNVERIFIED placeholder
#define SHOULDER_LEAD_M 0.008f   // leadscrew travel per motor rev
```

which produce:

| Joint | Gearing | Steps per unit |
|---|---|---|
| column | 20:1 | 2546 steps/rad |
| shoulder | 8 mm lead | 100000 steps/m |
| forearm | 17:1 | 2164 steps/rad |
| wrist | 1:1 **(TBC)** | 127 steps/rad |

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

which produce, with the defaults:

| Joint | Steps per unit |
|---|---|
| column | 637 steps/rad |
| shoulder | 100000 steps/m |
| forearm | 382 steps/rad |
| wrist | 127 steps/rad |

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
  -> COLUMN_GEARING should be about 25.00  (it is 20.0)   ->  3183 steps per radian
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

Gazebo will move a joint as fast as you ask; the real arm cannot. With 20:1 on
the column and 17:1 on the forearm, three of the four joints are far outside what
an Uno can step at their simulated limits, so
`scara_moveit/config/joint_limits.yaml` carries both values and you uncomment the
one you are running:

```yaml
  column_joint:
    # max_velocity: 1.5      # simulation
    max_velocity: 0.39       # real hardware  (20:1, 1000 steps/s cap)
  forearm_joint:
    # max_velocity: 1.5      # simulation
    max_velocity: 0.37       # real hardware  (17:1, 800 steps/s cap)
  shoulder_joint:
    # max_velocity: 0.5      # simulation
    max_velocity: 0.012      # real hardware  (8 mm lead, 1200 steps/s cap)
```

**Real hardware is active.** Every value is derived from the firmware's own cap:

| Joint | Steps per unit | Firmware cap | = real limit | Simulated | Overstated by |
|---|---|---|---|---|---|
| column | 2546 /rad | 1000 steps/s | **0.39 rad/s** | 1.5 | 3.8x |
| shoulder | 100000 /m | 1200 steps/s | **0.012 m/s** | 0.5 | 42x |
| forearm | 2164 /rad | 800 steps/s | **0.37 rad/s** | 1.5 | 4.1x |
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

## 8. Where things live

| Path | What |
|---|---|
| `scara_firmware/src/scara_hardware_interface.cpp` | The interface. `read()`/`write()` are the whole story |
| `scara_firmware/firmware/scara_moveit_control/` | The sketch to flash |
| `scara_description/urdf/ros2control.xacro` | The `is_sim` / `is_real` switch that picks the backend |
| `scara_bringup/launch/real_bringup.launch.py` | Real robot |
| `scara_bringup/launch/simulation_bringup.launch.py` | Gazebo |

Related: `manual_teaching_mode.ino` streams joint values *out* of the arm using
the same `J` frame, for lead-through teaching.

---

## 9. Possible next steps

- **Limit switches for homing.** The shield has endstop headers on D9–D11. This
  would remove the park-at-zero ritual, the single biggest weakness here.
- **A hardware E-stop** in series with `ENABLE`. Software cannot guarantee a stop.
- **Encoders (AS5600 + TCA9548A)** would turn `read()` into a real measurement
  and make this a closed-loop robot.
- **An ESP32** if you run out of step rate; the ROS side would not change at all.
