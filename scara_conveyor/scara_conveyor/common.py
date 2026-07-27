"""Shared constants for the SCARA conveyor mini-project.

One source of truth for topic/service names and the belt geometry so the
spawner, controller, executor, and state machine never drift apart. Geometry
values are also declared as ROS parameters on each node (defaults below), so
they can be retuned from the launch file without editing code.
"""

# ---------------------------------------------------------------- topics ----
# state_machine -> object_spawner : request one random box be spawned.
TOPIC_SPAWN_REQUEST = "/conveyor/spawn_request"
# object_spawner -> controller : a box was spawned (name + colour + x).
TOPIC_SPAWNED = "/conveyor/spawned"
# state_machine -> controller : run (True) or freeze (False) the belt.
TOPIC_RUN = "/conveyor/run"
# controller -> state_machine : the front box has settled at the pickup point.
TOPIC_BOX_AT_PICKUP = "/conveyor/box_at_pickup"
# state_machine -> controller/executor : the front box has been removed.
TOPIC_BOX_REMOVED = "/conveyor/box_removed"

# Reused, existing scara topics (detector + pick handshake).
TOPIC_PICK_COMMAND = "/scara/pick_command"        # SM  -> executor (colour)
TOPIC_PICK_TARGET_PIXEL = "/scara/pick_target_pixel"  # detector -> executor
TOPIC_PICK_COMPLETED = "/scara/pick_completed"    # executor -> SM (bool)

# -------------------------------------------------------------- services ----
# Bridged (ros_gz parameter_bridge) to the Gazebo UserCommands system.
# Names must match the running world name (see WORLD_NAME).
def spawn_service(world):
    return f"/world/{world}/create"


def set_pose_service(world):
    return f"/world/{world}/set_pose"


def remove_service(world):
    return f"/world/{world}/remove"


# -------------------------------------------------------------- geometry ----
WORLD_NAME = "conveyor"

# Belt runs along world +x; boxes spawn far out and advance toward the robot.
BELT_Y = -0.65          # camera centre-line, matches the fixed camera aim
BELT_Z = 0.3657         # proven box rest height on the coffee table
SPAWN_X = 0.85          # tail of the belt (a box is born here)
PICKUP_X = 0.40         # front of the belt: reachable + under the camera
SLOT_SPACING = 0.14     # gap between queued boxes (box is 0.10 wide in x)
BELT_SPEED = 0.12       # m/s the boxes advance while the belt is running
TICK_HZ = 30.0          # controller update rate

# A detection is only accepted as "the front box" if it lands this close to
# the pickup point, so a same-colour box queued behind can never be grabbed.
PICKUP_XY_TOLERANCE = 0.08

# Colours the cell handles. Keys match the detector / pick_command letters.
COLORS = ("R", "G", "B")
COLOR_MODEL = {"R": "Red_Box", "G": "Green_Box", "B": "Blue_Box"}
