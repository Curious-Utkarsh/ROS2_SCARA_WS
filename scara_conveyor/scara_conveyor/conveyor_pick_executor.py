#!/usr/bin/env python3
"""Gated, single-shot pick-and-drop for the conveyor cell.

Reuses the exact perception-to-motion approach of the standalone vision demo
(detector pixel + real depth + TF -> base-frame point -> IK -> joint-space
pick), but is driven by the state machine instead of running autonomously:

  * It only acts on the colour it was told to pick (/scara/pick_command), and
  * only if the detected box is at the pickup point (rejects queued boxes), and
  * it performs ONE pick, then reports done on /scara/pick_completed and waits.

This keeps the standalone vision_pick_and_place.py untouched while giving the
state machine the completion handshake it needs to resume the belt.
"""

import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from pymoveit2 import GripperInterface, MoveIt2
from pymoveit2.robots import scara

from scara_conveyor import common

# Fixed camera FOV (scara_description/urdf/gazebo.xacro).
HORIZONTAL_FOV = 1.3962634

# shoulder_joint lower limit -- fully lowered to grasp at the belt surface.
GRASP_SHOULDER_POSITION = -0.15
GRASP_JOINT_INDEX = scara.joint_names().index("shoulder_joint")

# Per-colour drop bins (joint targets), matching pick_and_drop.py / the vision
# demo. Joint order: column, shoulder, forearm, wrist.
DROP_TARGETS = {
    "R": {
        "drop_approach": [-1.611, 0.0, -2.00, 0.0],
        "drop_release": [-1.611, -0.05, -2.00, 0.0],
        "drop_home": [0.0, 0.0, 0.0, 0.0],
    },
    "G": {
        "drop_approach": [-1.611, 0.0, 2.00, -0.184],
        "drop_release": [-1.611, 0.0, 0.0, 0.0],
        "drop_home": [0.0, 0.0, 0.0, 0.0],
    },
    "B": {
        "drop_approach": [1.611, 0.0, -2.00, 0.184],
        "drop_release": [1.611, 0.0, 0.0, 0.0],
        "drop_home": [0.0, 0.0, 0.0, 0.0],
    },
}

# Latched command channel so a colour set before we subscribe is still received.
PICK_COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


class ConveyorPickExecutor(Node):
    def __init__(self):
        super().__init__("conveyor_pick_executor")

        self.declare_parameter("grasp_z_source", "fixed")
        self.declare_parameter("pickup_x", common.PICKUP_X)
        self.declare_parameter("pickup_y", common.BELT_Y)
        self.declare_parameter("pickup_xy_tolerance", common.PICKUP_XY_TOLERANCE)
        self.grasp_z_source = str(self.get_parameter("grasp_z_source").value).lower()
        if self.grasp_z_source not in ("fixed", "camera"):
            raise ValueError("grasp_z_source must be 'fixed' or 'camera'")
        self.pickup_x = float(self.get_parameter("pickup_x").value)
        self.pickup_y = float(self.get_parameter("pickup_y").value)
        self.tol = float(self.get_parameter("pickup_xy_tolerance").value)

        self.bridge = CvBridge()
        self.depth_image = None
        self.busy = False
        self.target_color = None
        self.pending_pick = None

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
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=scara.gripper_joint_names(),
            open_gripper_joint_positions=scara.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=scara.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=scara.MOVE_GROUP_GRIPPER,
            callback_group=group,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.completed_pub = self.create_publisher(
            Bool, common.TOPIC_PICK_COMPLETED, PICK_COMMAND_QOS
        )
        self.create_subscription(
            Image, "/camera_fixed/depth_image", self.depth_callback, 10
        )
        self.create_subscription(
            PointStamped, common.TOPIC_PICK_TARGET_PIXEL, self.target_callback, 10
        )
        self.create_subscription(
            String, common.TOPIC_PICK_COMMAND, self.command_callback, PICK_COMMAND_QOS
        )
        self.get_logger().info("Conveyor pick executor ready (gated, single-shot).")

    # ----------------------------------------------------------- callbacks --
    def command_callback(self, msg):
        color = msg.data.strip().upper()
        if color not in DROP_TARGETS:
            self.get_logger().warning(f"Ignoring command '{msg.data}'.")
            return
        if self.busy:
            return
        self.target_color = color
        self.get_logger().info(f"Commanded to pick {color}; watching pickup point.")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def target_callback(self, pixel):
        if self.busy or self.target_color is None or self.depth_image is None:
            return

        u, v = int(pixel.point.x), int(pixel.point.y)
        if u < 0 or v < 0 or v >= self.depth_image.shape[0] or u >= self.depth_image.shape[1]:
            return
        depth = float(self.depth_image[v, u])
        if self.depth_image.dtype == np.uint16:
            depth /= 1000.0
        if not np.isfinite(depth) or depth <= 0.0:
            return

        height, width = self.depth_image.shape[:2]
        fx = width / (2.0 * math.tan(HORIZONTAL_FOV / 2.0))
        fy = fx
        cx, cy = width / 2.0, height / 2.0

        camera_point = PointStamped()
        camera_point.header.stamp = pixel.header.stamp
        camera_point.header.frame_id = pixel.header.frame_id
        camera_point.point.x = (u - cx) * depth / fx
        camera_point.point.y = (v - cy) * depth / fy
        camera_point.point.z = depth

        try:
            camera_to_base = self.tf_buffer.lookup_transform(
                "base_link", camera_point.header.frame_id, Time()
            )
            box = do_transform_point(camera_point, camera_to_base)
        except TransformException as error:
            self.get_logger().warning(f"No camera-to-base TF yet: {error}")
            return

        # Gate: only pick a box that is actually at the pickup point. A box
        # queued further back (even the same colour) is rejected here.
        if (
            abs(box.point.x - self.pickup_x) > self.tol
            or abs(box.point.y - self.pickup_y) > self.tol
        ):
            return

        self.get_logger().info(
            f"{self.target_color} box at pickup: "
            f"base=({box.point.x:.3f}, {box.point.y:.3f}, {box.point.z:.3f})."
        )
        self.busy = True
        self.pending_pick = (box, self.target_color)

    # --------------------------------------------------------------- motion --
    def solve_ik(self, position):
        ik_result = self.arm.compute_ik(position=position, quat_xyzw=[0.0, 0.0, 0.0, 1.0])
        if ik_result is None:
            return None
        return [
            ik_result.position[ik_result.name.index(name)]
            for name in scara.joint_names()
        ]

    def move_to(self, joint_positions):
        self.arm.move_to_configuration(joint_positions, joint_names=scara.joint_names())
        return self.arm.wait_until_executed()

    def pick_box(self, box, color):
        x, y, z = box.point.x, box.point.y, box.point.z
        picked = False
        try:
            self.gripper.open()
            if not self.gripper.wait_until_executed():
                return
            approach = self.solve_ik([x, y, z + 0.10])
            if approach is None or not self.move_to(approach):
                self.get_logger().error(f"No reachable approach above {(x, y, z)}.")
                return
            if self.grasp_z_source == "camera":
                grasp = self.solve_ik([x, y, z])
                if grasp is None:
                    self.get_logger().error(f"No reachable grasp at {(x, y, z)}.")
                    return
            else:
                grasp = list(approach)
                grasp[GRASP_JOINT_INDEX] = GRASP_SHOULDER_POSITION
            if not self.move_to(grasp):
                return
            self.gripper.close()
            if not self.gripper.wait_until_executed():
                return
            if not self.move_to(approach):
                return
            drop = DROP_TARGETS[color]
            if not self.move_to(drop["drop_approach"]):
                return
            if not self.move_to(drop["drop_release"]):
                return
            self.gripper.open()
            if not self.gripper.wait_until_executed():
                return
            self.move_to(drop["drop_home"])
            picked = True
            self.get_logger().info(f"{color} pick-and-drop complete.")
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Pick failed: {error}")
        finally:
            self.busy = False
            self.target_color = None
            # Report the outcome so the state machine can resume the belt.
            self.completed_pub.publish(Bool(data=picked))


def main():
    rclpy.init()
    node = ConveyorPickExecutor()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pending_pick is not None:
                box, color = node.pending_pick
                node.pending_pick = None
                node.pick_box(box, color)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
