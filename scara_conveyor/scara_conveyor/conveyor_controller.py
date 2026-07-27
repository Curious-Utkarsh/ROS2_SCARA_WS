#!/usr/bin/env python3
"""Kinematic conveyor: advance boxes along Y, stop the front one at pickup.

The belt has no friction physics -- each tracked box is teleported forward a
little every tick via /world/<world>/set_pose. Boxes ease toward their slot
(pickup_y + rank * spacing) so they queue naturally behind the front one.

Two ways a box leaves the belt:
  * picked   -> the state machine sends /conveyor/box_removed (name); we delete
                it from the world.
  * passed   -> the state machine sends /conveyor/release_front (name); the box
                switches to PASS mode, rides to the far edge, drops off, and is
                deleted after a short fall.

This node owns all box motion AND removal; the state machine only decides.
"""

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose
from std_msgs.msg import Bool, String

from scara_conveyor import common

QUEUE = "QUEUE"
PASS = "PASS"


class Box:
    __slots__ = ("name", "color", "y", "mode", "released_at")

    def __init__(self, name, color, y):
        self.name = name
        self.color = color
        self.y = y
        self.mode = QUEUE
        self.released_at = None  # set when a PASS box reaches the drop-off point


class ConveyorController(Node):
    def __init__(self):
        super().__init__("conveyor_controller")

        self.declare_parameter("world_name", common.WORLD_NAME)
        self.declare_parameter("belt_x", common.BELT_X)
        self.declare_parameter("spawn_y", common.SPAWN_Y)
        self.declare_parameter("pickup_y", common.PICKUP_Y)
        self.declare_parameter("pass_end_y", common.PASS_END_Y)
        self.declare_parameter("belt_z", common.BELT_Z)
        self.declare_parameter("slot_spacing", common.SLOT_SPACING)
        self.declare_parameter("belt_speed", common.BELT_SPEED)
        self.declare_parameter("tick_hz", common.TICK_HZ)
        self.declare_parameter("fall_grace_s", 1.0)

        self.world = self.get_parameter("world_name").value
        self.belt_x = float(self.get_parameter("belt_x").value)
        self.spawn_y = float(self.get_parameter("spawn_y").value)
        self.pickup_y = float(self.get_parameter("pickup_y").value)
        self.pass_end_y = float(self.get_parameter("pass_end_y").value)
        self.belt_z = float(self.get_parameter("belt_z").value)
        self.spacing = float(self.get_parameter("slot_spacing").value)
        self.speed = float(self.get_parameter("belt_speed").value)
        self.tick_hz = float(self.get_parameter("tick_hz").value)
        self.fall_grace = float(self.get_parameter("fall_grace_s").value)

        self.step = self.speed / self.tick_hz
        self.settle_eps = 0.005

        # Ordered front -> back. Index 0 is nearest the robot (the pickup slot).
        self.boxes = []
        self.running = False
        self.announced_front = None

        self.pose_client = self.create_client(
            SetEntityPose, common.set_pose_service(self.world)
        )
        self.remove_client = self.create_client(
            DeleteEntity, common.remove_service(self.world)
        )
        self.at_pickup_pub = self.create_publisher(
            String, common.TOPIC_BOX_AT_PICKUP, 10
        )
        self.create_subscription(String, common.TOPIC_SPAWNED, self.on_spawned, 10)
        self.create_subscription(Bool, common.TOPIC_RUN, self.on_run, 10)
        self.create_subscription(
            String, common.TOPIC_BOX_REMOVED, self.on_removed, 10
        )
        self.create_subscription(
            String, common.TOPIC_RELEASE_FRONT, self.on_release_front, 10
        )
        self.create_timer(1.0 / self.tick_hz, self.on_tick)
        self.get_logger().info(
            f"Conveyor controller ready; waiting for {common.set_pose_service(self.world)}."
        )

    # ----------------------------------------------------------- callbacks --
    def on_spawned(self, msg):
        try:
            name, color = msg.data.split(";")
        except ValueError:
            self.get_logger().warning(f"Bad /conveyor/spawned payload: {msg.data!r}")
            return
        self.boxes.append(Box(name, color, self.spawn_y))
        self.get_logger().info(f"Tracking {name} ({color}); queue={len(self.boxes)}.")

    def on_run(self, msg):
        self.running = bool(msg.data)
        self.get_logger().info(f"Belt {'RUNNING' if self.running else 'FROZEN'}.")

    def on_removed(self, msg):
        name = msg.data.strip()
        self.delete_entity(name)
        self.boxes = [b for b in self.boxes if b.name != name]
        if self.announced_front == name:
            self.announced_front = None
        self.get_logger().info(f"Picked box {name} removed; queue={len(self.boxes)}.")

    def on_release_front(self, msg):
        name = msg.data.strip()
        for box in self.boxes:
            if box.name == name:
                box.mode = PASS
                self.get_logger().info(f"Releasing {name} ({box.color}) to pass off.")
                break
        if self.announced_front == name:
            self.announced_front = None

    # --------------------------------------------------------------- motion --
    def on_tick(self):
        # Fall-then-delete for passed boxes runs regardless of belt run state.
        self.reap_dropped()
        if not self.running or not self.boxes:
            return

        rank = 0
        for box in self.boxes:
            if box.mode == PASS:
                if box.released_at is None:
                    if box.y - self.pass_end_y > 1e-4:
                        box.y = max(self.pass_end_y, box.y - self.step)
                        self.set_pose(box)
                    else:
                        # Reached the edge: let gravity drop it, then reap.
                        box.released_at = self.get_clock().now()
                continue
            target_y = self.pickup_y + rank * self.spacing
            if box.y - target_y > 1e-4:  # advance (y decreases toward pickup)
                box.y = max(target_y, box.y - self.step)
                self.set_pose(box)
            rank += 1

        self.announce_front()

    def announce_front(self):
        front = next((b for b in self.boxes if b.mode == QUEUE), None)
        if front is None:
            return
        if (
            self.announced_front != front.name
            and abs(front.y - self.pickup_y) <= self.settle_eps
        ):
            self.announced_front = front.name
            self.at_pickup_pub.publish(String(data=f"{front.name};{front.color}"))
            self.get_logger().info(
                f"{front.name} ({front.color}) settled at pickup y={self.pickup_y:.2f}."
            )

    def reap_dropped(self):
        now = self.get_clock().now()
        survivors = []
        for box in self.boxes:
            if box.released_at is not None:
                elapsed = (now - box.released_at).nanoseconds / 1e9
                if elapsed >= self.fall_grace:
                    self.delete_entity(box.name)
                    self.get_logger().info(f"{box.name} ({box.color}) dropped off belt.")
                    continue
            survivors.append(box)
        self.boxes = survivors

    # -------------------------------------------------------------- gazebo ---
    def set_pose(self, box):
        if not self.pose_client.service_is_ready():
            return
        req = SetEntityPose.Request()
        req.entity = Entity(name=box.name, type=Entity.MODEL)
        pose = Pose()
        pose.position.x = self.belt_x
        pose.position.y = box.y
        pose.position.z = self.belt_z
        pose.orientation.w = 1.0
        req.pose = pose
        self.pose_client.call_async(req)

    def delete_entity(self, name):
        if not self.remove_client.service_is_ready():
            self.get_logger().warning(f"Remove service not ready; leaving {name}.")
            return
        req = DeleteEntity.Request()
        req.entity = Entity(name=name, type=Entity.MODEL)
        self.remove_client.call_async(req)


def main():
    rclpy.init()
    node = ConveyorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
