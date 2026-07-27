#!/usr/bin/env python3
"""Kinematic conveyor: advance queued boxes and stop the front one at pickup.

The belt is modelled without friction physics -- each tracked box is teleported
forward a little every tick via Gazebo's /world/<world>/set_pose service. Each
box eases toward its slot (pickup_x + rank * spacing), so boxes queue up
naturally behind the front one. "Stop the belt" is simply "stop advancing",
which makes the whole thing deterministic and trivially pausable.

Flow:
  /conveyor/spawned (name;color)  -> start tracking a box at the spawn tail
  /conveyor/run (bool)            -> advance (True) or freeze (False)
  /conveyor/box_removed (name)    -> drop a picked box from tracking
  /conveyor/box_at_pickup (color) -> published once the front box settles
"""

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool, String

from scara_conveyor import common


class Box:
    __slots__ = ("name", "color", "x")

    def __init__(self, name, color, x):
        self.name = name
        self.color = color
        self.x = x


class ConveyorController(Node):
    def __init__(self):
        super().__init__("conveyor_controller")

        self.declare_parameter("world_name", common.WORLD_NAME)
        self.declare_parameter("spawn_x", common.SPAWN_X)
        self.declare_parameter("pickup_x", common.PICKUP_X)
        self.declare_parameter("belt_y", common.BELT_Y)
        self.declare_parameter("belt_z", common.BELT_Z)
        self.declare_parameter("slot_spacing", common.SLOT_SPACING)
        self.declare_parameter("belt_speed", common.BELT_SPEED)
        self.declare_parameter("tick_hz", common.TICK_HZ)

        self.world = self.get_parameter("world_name").value
        self.spawn_x = float(self.get_parameter("spawn_x").value)
        self.pickup_x = float(self.get_parameter("pickup_x").value)
        self.belt_y = float(self.get_parameter("belt_y").value)
        self.belt_z = float(self.get_parameter("belt_z").value)
        self.spacing = float(self.get_parameter("slot_spacing").value)
        self.speed = float(self.get_parameter("belt_speed").value)
        self.tick_hz = float(self.get_parameter("tick_hz").value)

        self.step = self.speed / self.tick_hz
        self.settle_eps = 0.005  # within 5 mm of pickup_x counts as "settled"

        # Ordered front -> back. Index 0 is nearest the robot (the pickup slot).
        self.boxes = []
        self.running = False
        self.announced_front = None  # name of the box we've already announced

        self.pose_client = self.create_client(
            SetEntityPose, common.set_pose_service(self.world)
        )
        self.at_pickup_pub = self.create_publisher(
            String, common.TOPIC_BOX_AT_PICKUP, 10
        )
        self.create_subscription(String, common.TOPIC_SPAWNED, self.on_spawned, 10)
        self.create_subscription(Bool, common.TOPIC_RUN, self.on_run, 10)
        self.create_subscription(
            String, common.TOPIC_BOX_REMOVED, self.on_removed, 10
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
        self.boxes.append(Box(name, color, self.spawn_x))
        self.get_logger().info(f"Tracking {name} ({color}); queue={len(self.boxes)}.")

    def on_run(self, msg):
        self.running = bool(msg.data)
        self.get_logger().info(f"Belt {'RUNNING' if self.running else 'FROZEN'}.")

    def on_removed(self, msg):
        name = msg.data.strip()
        self.boxes = [b for b in self.boxes if b.name != name]
        if self.announced_front == name:
            self.announced_front = None
        self.get_logger().info(f"Removed {name}; queue={len(self.boxes)}.")

    # --------------------------------------------------------------- motion --
    def on_tick(self):
        if not self.running or not self.boxes:
            return

        for rank, box in enumerate(self.boxes):
            target_x = self.pickup_x + rank * self.spacing
            if box.x - target_x > 1e-4:  # still needs to advance (x decreases)
                box.x = max(target_x, box.x - self.step)
                self.set_pose(box)

        # Announce the front box once it has settled at the pickup point.
        front = self.boxes[0]
        if (
            self.announced_front != front.name
            and abs(front.x - self.pickup_x) <= self.settle_eps
        ):
            self.announced_front = front.name
            # "name;color" so the state machine knows which entity to remove.
            self.at_pickup_pub.publish(String(data=f"{front.name};{front.color}"))
            self.get_logger().info(
                f"{front.name} ({front.color}) settled at pickup x={self.pickup_x:.2f}."
            )

    def set_pose(self, box):
        if not self.pose_client.service_is_ready():
            return
        req = SetEntityPose.Request()
        req.entity = Entity(name=box.name, type=Entity.MODEL)
        pose = Pose()
        pose.position.x = box.x
        pose.position.y = self.belt_y
        pose.position.z = self.belt_z
        pose.orientation.w = 1.0
        req.pose = pose
        self.pose_client.call_async(req)  # fire-and-forget; next tick corrects


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
