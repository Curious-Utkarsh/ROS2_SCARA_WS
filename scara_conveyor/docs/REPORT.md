# Conveyor Pick-and-Place — Technical Report

**Project title:** Event-Driven Conveyor Pick-and-Place for a Simulated SCARA Cell

**Objective:** Extend the existing vision pick-and-place SCARA project into an
industrial-style **colour-sorting** cell: randomly coloured boxes ride a
conveyor (perpendicular to the arm, on the table) to a pickup point. The
operator picks one colour to keep; that colour is picked into its bin and every
other colour rides past and drops off the far edge — the belt and robot
coordinated by ROS events, not timers.

## Features implemented
- Kinematic conveyor in Gazebo (deterministic, pausable) running along Y, fully
  on the table, carrying boxes to a central pickup point.
- Random R/G/B box spawning at a configurable cadence from existing box models.
- Natural multi-box queueing behind the pickup point.
- Operator-selected keep-colour via a separately launched pick node; belt stays
  idle until a colour is chosen.
- Per-box keep/pass decision: keep → freeze + pick + colour-bin drop; pass →
  ride to the far edge and drop off the table.
- Gated, single-shot perception-driven pick that rejects queued boxes.
- A clean state machine with a completion handshake (`/scara/pick_completed`)
  and a watchdog so a failed pick never wedges the line.
- One-command launch of the cell; runs indefinitely.

## ROS 2 concepts used
Publishers/subscribers, latched (transient-local) QoS for command/handshake
topics, service clients, bridged services (`ros_gz_bridge` service bridge),
node parameters for geometry, timers for spawn cadence and the control tick,
and a single-spinner execution loop that coexists with MoveIt's internal spins.

## Gazebo (Sim / Fortress) concepts used
`UserCommands` services `create` / `set_pose` / `remove` driven from ROS via the
`ros_gz_interfaces` `SpawnEntity` / `SetEntityPose` / `DeleteEntity` service
types; `GZ_SIM_RESOURCE_PATH` model resolution; a purpose-built world with
explicit Physics / UserCommands / SceneBroadcaster systems; RGBD sensor topics
bridged to ROS.

## PyMoveit2 concepts used
`MoveIt2` joint-space goals via the MoveGroup action, `compute_ik` for turning a
camera-derived 3D point into a reachable joint configuration, `GripperInterface`
open/close, and blocking `wait_until_executed` sequencing.

## Key learnings
- Decoupling *policy* (the state machine) from *mechanism* (spawner, controller,
  executor) keeps each node small and independently testable.
- A kinematic teleport belt is more controllable and demo-stable than a
  friction-driven one, and pausing becomes trivial.
- Position-gating the pick (accept only a detection at the pickup point) is what
  makes multi-box queues safe with a single shared camera.
- Latched QoS removes node start-order races on command/handshake topics.

## Four-colour update (Red / Green / Blue / Yellow)
- Darkened the Blue box to a deep navy so the RGB camera stops confusing it with
  the robot's blue links (they were nearly the same colour); tightened the blue
  HSV band to a low-value, pure-blue range to match.
- Added a Yellow box model (reusing the existing box geometry/collision/inertia)
  and a Yellow drop bin defined as the mirror of Red (opposite side, same reach).
- Spawner, drop map, and detector all key off dictionaries, so the colour set
  extends with one entry per table -- no logic changes.

## Future improvements
- Continuous tracking (servo to a moving box) instead of stop-to-pick.
- Bin-fullness feedback and multi-arm balancing.
- Swap the HSV detector for a learned detector / VLM for arbitrary objects.
- Detatchable-joint grasp for physically reliable carrying under load.

## GitHub-ready summary
> **scara_conveyor** — an event-driven conveyor pick-and-place cell for a
> simulated SCARA arm (ROS 2 Humble + Gazebo Fortress + MoveIt 2). Random
> coloured boxes are spawned onto a kinematic belt, queue to a pickup point, are
> detected by a fixed RGBD camera, and sorted into colour bins by the arm. The
> belt and robot are synchronised entirely by ROS events through a compact state
> machine — no timer-driven hacks.
