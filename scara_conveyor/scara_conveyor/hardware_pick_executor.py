#!/usr/bin/env python3
"""The pick node for the REAL conveyor cell -- a blind pick at the belt centre.

    ros2 run scara_conveyor hardware_pick_executor --ros-args -p target_color:=R

Same job as conveyor_pick_executor.py, same drop positions, but it never looks
at the box it is picking. It does not have to:

    conveyor_color_gate stops the belt when a box of the wanted colour has its
    middle on the middle of the camera picture. Every box therefore comes to
    rest at the SAME PLACE, under the camera centre. So the pick pose is a fixed
    pose, worked out once.

That is a better fit for real hardware than looking, for two reasons. The arm
covers the box as it comes down, so a camera would lose it at the worst moment;
and this robot has no depth camera on the bench anyway.

    conveyor_state_machine --> /conveyor/execute_pick --> [this node]
                                                              |
                    open, down, close, up, drop, home  (MoveIt)
                                                              |
                        /scara/pick_completed --> belt runs again

WHERE THE POSES COME FROM

    drop positions   IDENTICAL to the simulation. They are imported from
                     conveyor_pick_executor.py, not copied, so the two can
                     never drift apart. Red and Green are the calibrated pair;
                     Yellow and Blue are their exact mirror images.

    pick position    two ways, chosen with pick_source:

                     "ik" (the default) asks MoveIt for the joint angles that
                     reach the same belt pickup point the simulation uses,
                     scara_conveyor/common.py's pickup_grasp_point_base(). If
                     your real belt sits where the simulated one does, this is
                     already right.

                     "taught" uses the four numbers in pick_approach instead.
                     Park the arm over a box with the sliders or scara_gui.py,
                     read the joint values off, and pass them in.

The grasp itself copies the simulation's trick: keep the approach pose and just
wind the shoulder down to the bottom of its travel. One joint moves, so nothing
can swing into the box on the way down.

    *** THE ARM IS OPEN LOOP. Park it at zero before powering the drivers, or
    every pose in this file points somewhere else. ***
"""

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from pymoveit2 import GripperInterface, MoveIt2
from pymoveit2.robots import scara

from scara_conveyor import common

# The drop positions, straight from the simulation's pick node. Importing them
# rather than copying is the whole point: retune Red there and the real cell
# follows, mirrors and all.
from scara_conveyor.conveyor_pick_executor import (
    DROP_TARGETS,
    GRASP_JOINT_INDEX,
    GRASP_SHOULDER_POSITION,
)

LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)

# How far above the box to line up before coming down, in metres.
APPROACH_HEIGHT = 0.10


class HardwarePickExecutor(Node):

    def __init__(self):
        super().__init__("hardware_pick_executor")

        # Dynamic typing: ROS 2 arg parsing reads a bare `Y` (also y/yes/no/on/
        # off/true...) as a YAML boolean, so `-p target_color:=Y` arrives as
        # True. Allow it, then coerce back to the colour letter below.
        self.declare_parameter(
            "target_color", "R", ParameterDescriptor(dynamic_typing=True)
        )
        self.declare_parameter("pick_source", "ik")       # "ik" or "taught"
        # column, shoulder, forearm, wrist -- only read when pick_source=taught.
        # Shoulder is ignored: the grasp always winds it to grasp_shoulder.
        self.declare_parameter("pick_approach", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("approach_height", APPROACH_HEIGHT)
        self.declare_parameter("grasp_shoulder", GRASP_SHOULDER_POSITION)

        # MoveIt scales every move by these. joint_limits.yaml already holds the
        # REAL stepper speeds, so 1.0 means "as fast as the motors can manage",
        # not "dangerously fast". The file's own default of 0.1 would be ten
        # times slower again, which on this arm is a very long wait.
        self.declare_parameter("velocity_scaling", 1.0)
        self.declare_parameter("acceleration_scaling", 1.0)

        # How far the gripper closes, in metres. The simulation uses -0.03; the
        # real firmware maps -0.06 to a fully shut gripper, so a real box may
        # need more than the simulated one. Measure it and set it here.
        self.declare_parameter("grip_closed", scara.CLOSED_GRIPPER_JOINT_POSITIONS[0])

        raw_color = self.get_parameter("target_color").value
        # `Y`/`y`/`yes` came in as boolean True (see above) -> map back to Y.
        if isinstance(raw_color, bool):
            raw_color = "Y" if raw_color else ""
        self.target_color = str(raw_color).strip().upper()
        if self.target_color not in DROP_TARGETS:
            raise ValueError(f"target_color must be one of {sorted(DROP_TARGETS)}")

        self.pick_source = str(self.get_parameter("pick_source").value).lower()
        if self.pick_source not in ("ik", "taught"):
            raise ValueError("pick_source must be 'ik' or 'taught'")

        self.taught_approach = [
            float(v) for v in self.get_parameter("pick_approach").value
        ]
        if len(self.taught_approach) != len(scara.joint_names()):
            raise ValueError(
                f"pick_approach needs {len(scara.joint_names())} numbers "
                f"({', '.join(scara.joint_names())})"
            )
        self.approach_height = float(self.get_parameter("approach_height").value)
        self.grasp_shoulder = float(self.get_parameter("grasp_shoulder").value)
        self.grip_closed = float(self.get_parameter("grip_closed").value)

        self.busy = False
        self.pick_requested = False
        # Worked out on the first pick and kept, so the second box does not wait
        # for the IK solver again.
        self.approach_joints = None

        group = ReentrantCallbackGroup()
        self.arm = MoveIt2(
            node=self,
            joint_names=scara.joint_names(),
            base_link_name=scara.base_link_name(),
            end_effector_name=scara.end_effector_name(),
            group_name=scara.MOVE_GROUP_ARM,
            callback_group=group,
            use_move_group_action=True,
        )
        self.arm.max_velocity = float(self.get_parameter("velocity_scaling").value)
        self.arm.max_acceleration = float(
            self.get_parameter("acceleration_scaling").value
        )

        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=scara.gripper_joint_names(),
            open_gripper_joint_positions=scara.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=[self.grip_closed],
            gripper_group_name=scara.MOVE_GROUP_GRIPPER,
            callback_group=group,
        )

        # Announce the wanted colour. The colour gate watches for it and the
        # state machine only stops the belt for it. Latched, so both of them get
        # it whenever they start -- before or after this node.
        self.target_pub = self.create_publisher(
            String, common.TOPIC_TARGET_COLOR, LATCHED
        )
        self.completed_pub = self.create_publisher(
            Bool, common.TOPIC_PICK_COMPLETED, LATCHED
        )
        self.target_pub.publish(String(data=self.target_color))

        self.create_subscription(
            String, common.TOPIC_EXECUTE_PICK, self.on_execute_pick, 10
        )

        where = (
            f"the joint pose {self.taught_approach}" if self.pick_source == "taught"
            else f"the belt pickup point {common.pickup_grasp_point_base()} (by IK)"
        )
        self.get_logger().info(
            f"Hardware pick node ready. Picking {self.target_color} blind at "
            f"{where}; every other colour rides past."
        )

    # ----------------------------------------------------------- callbacks --

    def on_execute_pick(self, message):
        """The state machine has stopped the belt and wants this box picked."""
        if self.busy:
            self.get_logger().warning(
                f"Still busy; ignoring the request to pick {message.data}"
            )
            return
        self.pick_requested = True

    # --------------------------------------------------------------- poses --

    def solve_ik(self, position):
        """Joint angles that put the tool at this point, or None."""
        result = self.arm.compute_ik(position=position, quat_xyzw=[0.0, 0.0, 0.0, 1.0])
        if result is None:
            return None
        return [result.position[result.name.index(name)] for name in scara.joint_names()]

    def find_approach(self):
        """The pose to line up in, above the box. Worked out once and kept."""
        if self.approach_joints is not None:
            return self.approach_joints

        if self.pick_source == "taught":
            self.approach_joints = list(self.taught_approach)
            return self.approach_joints

        x, y, z = common.pickup_grasp_point_base()
        above = [x, y, z + self.approach_height]
        joints = self.solve_ik(above)
        if joints is None:
            self.get_logger().error(
                f"MoveIt cannot reach {tuple(round(v, 3) for v in above)}. Either "
                "the belt is not where common.py says, or the arm cannot get "
                "there -- teach the pose instead: -p pick_source:=taught "
                "-p pick_approach:='[column, shoulder, forearm, wrist]'"
            )
            return None

        self.get_logger().info(
            "Pick pose solved once and kept: "
            + ", ".join(f"{n}={v:.3f}" for n, v in zip(scara.joint_names(), joints))
        )
        self.approach_joints = joints
        return joints

    def grasp_from(self, approach):
        """The same pose with the shoulder wound down to the belt.

        Only the vertical slide moves, so nothing can swing sideways into the
        box on the way down. This is what the simulation does too.
        """
        grasp = list(approach)
        grasp[GRASP_JOINT_INDEX] = self.grasp_shoulder
        return grasp

    # -------------------------------------------------------------- motion --

    def move_to(self, joint_positions, what):
        self.get_logger().info(f"-> {what}")
        self.arm.move_to_configuration(joint_positions, joint_names=scara.joint_names())
        if self.arm.wait_until_executed():
            return True
        self.get_logger().error(f"Could not move to {what}.")
        return False

    def work_the_gripper(self, close):
        if close:
            self.gripper.close()
        else:
            self.gripper.open()
        if self.gripper.wait_until_executed():
            return True
        self.get_logger().error(f"Gripper would not {'close' if close else 'open'}.")
        return False

    def run_pick(self):
        """One box: down, grab, up, over to its bin, let go, home.

        Whatever happens, /scara/pick_completed goes out at the end -- true if
        the box was dealt with, false if anything went wrong. The state machine
        is waiting on it either way, and if it never arrives the belt stays
        frozen until the watchdog gives up.
        """
        picked = False
        drop = DROP_TARGETS[self.target_color]

        try:
            approach = self.find_approach()
            if approach is None:
                return

            if not self.work_the_gripper(close=False):
                return
            if not self.move_to(approach, "above the box"):
                return
            if not self.move_to(self.grasp_from(approach), "down onto the box"):
                return
            if not self.work_the_gripper(close=True):
                return
            if not self.move_to(approach, "lift clear"):
                return
            if not self.move_to(drop["drop_approach"], f"over the {self.target_color} bin"):
                return
            if not self.move_to(drop["drop_release"], "down to release"):
                return
            if not self.work_the_gripper(close=False):
                return
            self.move_to(drop["drop_home"], "home")

            picked = True
            self.get_logger().info(f"{self.target_color} pick-and-drop complete.")
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Pick failed: {error}")
        finally:
            self.completed_pub.publish(Bool(data=picked))


def main():
    rclpy.init()
    node = HardwarePickExecutor()
    try:
        # The pick runs here rather than in the callback: it blocks for a long
        # time on this arm, and blocking inside a callback would stop this node
        # hearing anything else while it did.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pick_requested and not node.busy:
                node.busy = True
                node.pick_requested = False
                node.run_pick()
                node.busy = False
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
