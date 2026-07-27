#!/usr/bin/env python3
"""The "final" pick-and-place node -- run this separately with a target colour.

    ros2 run scara_conveyor conveyor_pick_executor --ros-args -p target_color:=R

It announces the wanted colour to the detector (so it locates that colour) and
to the state machine (so the belt only stops for that colour). It does NOT act
on its own: it picks exactly one box each time the state machine triggers it
(/conveyor/execute_pick), grasps the box the camera sees at the pickup point,
drops it in that colour's bin, and reports /scara/pick_completed. Boxes of every
other colour are passed down the belt by the state machine, never touched here.
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

LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


class ConveyorPickExecutor(Node):
    def __init__(self):
        super().__init__("conveyor_pick_executor")

        # Pickup point in the base_link frame -- vision reports boxes in
        # base_link, and the robot base is offset from the Gazebo world origin,
        # so the gate/pick must be in base_link too (not world coordinates).
        pickup_base_x, pickup_base_y = common.pickup_in_base()
        self.fixed_grasp = common.pickup_grasp_point_base()

        self.declare_parameter("target_color", "R")
        self.declare_parameter("grasp_z_source", "fixed")
        # Boxes always stop at the same spot, and the arm occludes the camera
        # there. So by default pick blindly at the known pickup point; set
        # use_vision:=true to instead grasp the camera-detected position.
        self.declare_parameter("use_vision", False)
        self.declare_parameter("pickup_x", pickup_base_x)
        self.declare_parameter("pickup_y", pickup_base_y)
        self.declare_parameter("pickup_xy_tolerance", common.PICKUP_XY_TOLERANCE)
        self.declare_parameter("detection_timeout_s", 4.0)
        self.use_vision = bool(self.get_parameter("use_vision").value)

        self.target_color = str(self.get_parameter("target_color").value).strip().upper()
        if self.target_color not in DROP_TARGETS:
            raise ValueError("target_color must be R, G, or B")
        self.grasp_z_source = str(self.get_parameter("grasp_z_source").value).lower()
        if self.grasp_z_source not in ("fixed", "camera"):
            raise ValueError("grasp_z_source must be 'fixed' or 'camera'")
        self.pickup_x = float(self.get_parameter("pickup_x").value)
        self.pickup_y = float(self.get_parameter("pickup_y").value)
        self.tol = float(self.get_parameter("pickup_xy_tolerance").value)
        self.detect_timeout = float(self.get_parameter("detection_timeout_s").value)

        self.bridge = CvBridge()
        self.depth_image = None
        self.busy = False
        self.pick_requested = False
        self.latest_box = None   # (x, y, z) in base_link, within the pickup window
        self.latest_stamp = None
        # Belt line as measured by vision anywhere along the belt (X and Z are
        # constant; only Y changes). Used for the blind pick so occlusion at the
        # pickup does not matter and the grasp Z is auto-calibrated.
        self.belt_x = None
        self.belt_z = None
        # Diagnostics so a rejected detection explains itself.
        self.pixels_seen = 0
        self.last_base = None
        self.last_reject = "no pixel received yet"
        self._last_diag = self.get_clock().now()

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

        # Announce the wanted colour: to the detector (locate it) and to the
        # state machine (only stop the belt for it). Latched so late nodes get it.
        self.command_pub = self.create_publisher(
            String, common.TOPIC_PICK_COMMAND, LATCHED
        )
        self.target_pub = self.create_publisher(
            String, common.TOPIC_TARGET_COLOR, LATCHED
        )
        self.completed_pub = self.create_publisher(
            Bool, common.TOPIC_PICK_COMPLETED, LATCHED
        )
        self.command_pub.publish(String(data=self.target_color))
        self.target_pub.publish(String(data=self.target_color))

        self.create_subscription(
            Image, "/camera_fixed/depth_image", self.depth_callback, 10
        )
        self.create_subscription(
            PointStamped, common.TOPIC_PICK_TARGET_PIXEL, self.target_callback, 10
        )
        self.create_subscription(
            String, common.TOPIC_EXECUTE_PICK, self.on_execute_pick, 10
        )
        mode = "camera-detected position" if self.use_vision else "fixed pickup point"
        self.get_logger().info(
            f"Pick node ready. Picking {self.target_color} at the {mode}; "
            "all other colours pass."
        )

    # ----------------------------------------------------------- callbacks --
    def on_execute_pick(self, _msg):
        if not self.busy:
            self.pick_requested = True

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def diag(self, msg):
        # Throttle noisy per-frame diagnostics to ~1 Hz.
        now = self.get_clock().now()
        if (now - self._last_diag).nanoseconds / 1e9 >= 1.0:
            self._last_diag = now
            self.get_logger().info(msg)

    def target_callback(self, pixel):
        # Keep the most recent detection that sits at the pickup point.
        self.pixels_seen += 1
        if self.depth_image is None:
            self.last_reject = "no depth image yet"
            self.diag("detector pixel received but no /camera_fixed/depth_image yet.")
            return
        u, v = int(pixel.point.x), int(pixel.point.y)
        if u < 0 or v < 0 or v >= self.depth_image.shape[0] or u >= self.depth_image.shape[1]:
            self.last_reject = f"pixel ({u},{v}) out of image bounds"
            return
        depth = float(self.depth_image[v, u])
        if self.depth_image.dtype == np.uint16:
            depth /= 1000.0
        if not np.isfinite(depth) or depth <= 0.0:
            self.last_reject = f"invalid depth {depth} at pixel ({u},{v})"
            self.diag(self.last_reject)
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
            self.last_reject = f"no TF base_link<-{camera_point.header.frame_id}: {error}"
            self.diag(self.last_reject)
            return

        bx, by, bz = box.point.x, box.point.y, box.point.z
        self.last_base = (bx, by, bz)
        # Anywhere along the belt line (X near pickup), capture X and Z. These
        # are constant for every box, so the blind pick uses them + the known
        # pickup Y -- no need to ever see the box at the (occluded) pickup.
        if abs(bx - self.pickup_x) <= self.tol:
            self.belt_x = bx
            self.belt_z = bz
        dx, dy = abs(bx - self.pickup_x), abs(by - self.pickup_y)
        in_window = dx <= self.tol and dy <= self.tol
        self.diag(
            f"belt line seen: base=({bx:.3f},{by:.3f},{bz:.3f}) depth={depth:.3f} "
            f"(blind pick will grasp x={self.belt_x:.3f} z={self.belt_z:.3f} "
            f"at pickup y={self.pickup_y:.2f})"
        )
        if not in_window:
            self.last_reject = (
                f"detection ({bx:.3f},{by:.3f}) is dx={dx:.3f} dy={dy:.3f} "
                f"from pickup (tol={self.tol:.2f})"
            )
            return
        self.latest_box = (bx, by, bz)
        self.latest_stamp = self.get_clock().now()

    # --------------------------------------------------------------- motion --
    def fresh_detection(self):
        if self.latest_box is None or self.latest_stamp is None:
            return None
        age = (self.get_clock().now() - self.latest_stamp).nanoseconds / 1e9
        return self.latest_box if age <= 1.0 else None

    def run_pick(self):
        if not self.use_vision:
            # Give vision a moment to report the belt line if it hasn't yet.
            start = self.get_clock().now()
            while self.belt_z is None:
                if (self.get_clock().now() - start).nanoseconds / 1e9 > 2.0:
                    break
                rclpy.spin_once(self, timeout_sec=0.1)
            # Blind pick at the belt line X/Z (measured during travel) and the
            # known pickup Y. Falls back to the configured pickup if unseen.
            gx = self.belt_x if self.belt_x is not None else self.fixed_grasp[0]
            gz = self.belt_z if self.belt_z is not None else self.fixed_grasp[2]
            grasp = (gx, self.pickup_y, gz)
            src = "vision-measured belt line" if self.belt_z is not None else "fallback"
            self.get_logger().info(
                f"Picking at {tuple(round(v, 3) for v in grasp)} ({src})."
            )
            self.pick_box(grasp, self.target_color)
            return

        # Vision mode: wait briefly for a fresh detection of the box at pickup.
        start = self.get_clock().now()
        box = self.fresh_detection()
        while box is None:
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > self.detect_timeout:
                self.get_logger().error(
                    "No box seen at pickup; reporting failure. "
                    f"[pixels_seen={self.pixels_seen}, "
                    f"depth_image={'yes' if self.depth_image is not None else 'NONE'}, "
                    f"last_base={self.last_base}, last_reject: {self.last_reject}]"
                )
                self.completed_pub.publish(Bool(data=False))
                return
            rclpy.spin_once(self, timeout_sec=0.1)
            box = self.fresh_detection()
        self.pick_box(box, self.target_color)

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
        x, y, z = box
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
            self.latest_box = None
            self.completed_pub.publish(Bool(data=picked))


def main():
    rclpy.init()
    node = ConveyorPickExecutor()
    try:
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
