from typing import List

MOVE_GROUP_ARM = "arm"
MOVE_GROUP_GRIPPER = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS = [0.0]

# Fully shut, the same as the control panel's gripper button and the firmware's
# GRIPPER_CLOSED_METRES. The simulation used to close to -0.03, half the travel,
# which on the real gripper looked like a twitch rather than a grip.
CLOSED_GRIPPER_JOINT_POSITIONS = [-0.06]

def joint_names() -> List[str]:
    return [
        "column_joint",
        "shoulder_joint",
        "forearm_joint",
        "wrist_joint",
    ]

def base_link_name() -> str:
    return "base_link"

def end_effector_name() -> str:
    return "tool0"

def gripper_joint_names() -> List[str]:
    return [
        "left_finger_joint",
    ]
