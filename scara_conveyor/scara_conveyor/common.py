"""Shared constants for the SCARA conveyor mini-project.

One source of truth for topic/service names and the belt geometry so the
spawner, controller, executor, and state machine never drift apart. Geometry
values are also declared as ROS parameters on each node (defaults below), so
they can be retuned from the launch file without editing code.

Layout: the belt runs along world Y (perpendicular to the arm's reach), fully
on the table. Boxes are born at the near edge, advance (decreasing Y) to the
pickup point at the table centre, and -- if not the wanted colour -- keep going
and drop off the far edge.
"""

# ---------------------------------------------------------------- topics ----
# state_machine -> object_spawner : request one random box be spawned.
TOPIC_SPAWN_REQUEST = "/conveyor/spawn_request"
# object_spawner -> controller : a box was spawned (name;color).
TOPIC_SPAWNED = "/conveyor/spawned"
# state_machine -> controller : run (True) or freeze (False) the belt.
TOPIC_RUN = "/conveyor/run"
# controller -> state_machine : the front box has settled at pickup (name;color).
TOPIC_BOX_AT_PICKUP = "/conveyor/box_at_pickup"
# state_machine -> controller : remove a just-picked box (name).
TOPIC_BOX_REMOVED = "/conveyor/box_removed"
# state_machine -> controller : let the front box pass through and drop off (name).
TOPIC_RELEASE_FRONT = "/conveyor/release_front"

# executor -> state_machine : the colour the operator asked to pick (latched).
TOPIC_TARGET_COLOR = "/conveyor/target_color"
# state_machine -> executor : pick the box now at the pickup point (name;color).
TOPIC_EXECUTE_PICK = "/conveyor/execute_pick"

# Reused, existing scara topics.
TOPIC_PICK_COMMAND = "/scara/pick_command"            # executor -> detector (colour)
TOPIC_PICK_TARGET_PIXEL = "/scara/pick_target_pixel"  # detector  -> executor
TOPIC_PICK_COMPLETED = "/scara/pick_completed"        # executor  -> state_machine (bool)


# -------------------------------------------------------------- services ----
# Bridged (ros_gz parameter_bridge) to the Gazebo UserCommands system.
def spawn_service(world):
    return f"/world/{world}/create"


def set_pose_service(world):
    return f"/world/{world}/set_pose"


def remove_service(world):
    return f"/world/{world}/remove"


# -------------------------------------------------------------- geometry ----
WORLD_NAME = "conveyor"

# Belt runs along world Y at a fixed X (perpendicular to the arm's reach).
BELT_X = 0.35           # reachable + under the fixed camera, fully on the table
BELT_Z = 0.3657         # proven box rest height on the coffee table
SPAWN_Y = -0.38         # near edge of the table: a box is born here
PICKUP_Y = -0.65        # table centre: reachable pick point, under the camera
PASS_END_Y = -1.05      # past the far table edge: unwanted boxes drop off here
SLOT_SPACING = 0.12     # gap between queued boxes (box is 0.10 deep in Y)
BELT_SPEED = 0.10       # m/s the boxes advance while the belt is running
TICK_HZ = 30.0          # controller update rate

# A detection is only accepted as "the front box" if it lands this close to
# the pickup point, so a same-colour box queued behind can never be grabbed.
# (Queued boxes are >= SLOT_SPACING away, so this stays comfortably below that.)
PICKUP_XY_TOLERANCE = 0.11

# Robot base pose in the Gazebo world (scara_description/launch/gazebo.launch.py,
# gz_spawn_entity). base_link coincides with it (base_joint is identity), so a
# Gazebo-world point (x, y, z) maps to base_link (x - ROBOT_BASE_X, ...).
# The vision pipeline and MoveIt both work in base_link, so the pick point must too.
ROBOT_BASE_X = 0.0
ROBOT_BASE_Y = -0.65
ROBOT_BASE_Z = 0.37

BOX_HEIGHT = 0.095      # box mesh is 0.10 x 0.10 x 0.095 (base at the origin)

# Box-top height at the pickup, in base_link, as the depth camera actually
# reports it (~0.025). The naive analytic value (BELT_Z + BOX_HEIGHT -
# ROBOT_BASE_Z = 0.09) is wrong because base_link sits higher in the world than
# the spawn's -z suggests; z=0.09 approaches above the shoulder's reach and IK
# fails. This is only a fallback -- the executor prefers the live vision value.
GRASP_Z_BASE = 0.025


def pickup_in_base():
    """Pickup point (x, y) in the base_link frame (what vision reports)."""
    return (BELT_X - ROBOT_BASE_X, PICKUP_Y - ROBOT_BASE_Y)


def pickup_grasp_point_base():
    """Fallback box-top grasp point at the pickup, in base_link (x, y, z)."""
    x, y = pickup_in_base()
    return (x, y, GRASP_Z_BASE)


# Colours the cell handles. Keys match the detector / pick_command letters.
COLORS = ("R", "G", "B")
COLOR_MODEL = {"R": "Red_Box", "G": "Green_Box", "B": "Blue_Box"}
