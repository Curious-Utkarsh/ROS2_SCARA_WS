#!/usr/bin/env python3
"""Spawn a random coloured box at the tail of the belt, on request.

Purely event-driven: it spawns exactly one box each time the state machine
publishes on /conveyor/spawn_request, then announces the new box (name +
colour) on /conveyor/spawned so the controller can start advancing it. The
boxes themselves are the existing scara_description models -- nothing new.
"""

import os
import random

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
from std_msgs.msg import Empty, String

from scara_conveyor import common
from scara_conveyor.common import COLOR_MODEL, COLORS


class ObjectSpawner(Node):
    def __init__(self):
        super().__init__("object_spawner")

        self.declare_parameter("world_name", common.WORLD_NAME)
        self.declare_parameter("belt_x", common.BELT_X)
        self.declare_parameter("spawn_y", common.SPAWN_Y)
        self.declare_parameter("belt_z", common.BELT_Z)

        self.world = self.get_parameter("world_name").value
        self.belt_x = float(self.get_parameter("belt_x").value)
        self.spawn_y = float(self.get_parameter("spawn_y").value)
        self.belt_z = float(self.get_parameter("belt_z").value)

        models_dir = os.path.join(
            get_package_share_directory("scara_description"), "models"
        )
        self.model_files = {
            c: os.path.join(models_dir, COLOR_MODEL[c], "model.sdf") for c in COLORS
        }
        for path in self.model_files.values():
            if not os.path.exists(path):
                self.get_logger().error(f"Missing box model: {path}")

        self.count = 0
        self.spawn_client = self.create_client(
            SpawnEntity, common.spawn_service(self.world)
        )
        self.spawned_pub = self.create_publisher(String, common.TOPIC_SPAWNED, 10)
        self.create_subscription(
            Empty, common.TOPIC_SPAWN_REQUEST, self.on_spawn_request, 10
        )
        self.get_logger().info(
            f"Object spawner ready; waiting for {common.spawn_service(self.world)}."
        )

    def on_spawn_request(self, _msg):
        if not self.spawn_client.service_is_ready():
            self.get_logger().warning("Spawn service not ready yet; skipping request.")
            return

        color = random.choice(COLORS)
        self.count += 1
        name = f"box_{color}_{self.count:04d}"

        pose = Pose()
        pose.position.x = self.belt_x
        pose.position.y = self.spawn_y
        pose.position.z = self.belt_z

        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.sdf_filename = self.model_files[color]
        req.entity_factory.pose = pose
        req.entity_factory.relative_to = "world"
        req.entity_factory.allow_renaming = False

        future = self.spawn_client.call_async(req)
        future.add_done_callback(
            lambda f, n=name, c=color: self.on_spawn_done(f, n, c)
        )

    def on_spawn_done(self, future, name, color):
        try:
            ok = future.result().success
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Spawn call failed for {name}: {error}")
            return
        if not ok:
            self.get_logger().error(f"Gazebo refused to spawn {name}.")
            return
        self.get_logger().info(f"Spawned {name} ({color}) at y={self.spawn_y:.2f}.")
        # Announce as "name;color" so the controller starts tracking it.
        self.spawned_pub.publish(String(data=f"{name};{color}"))


def main():
    rclpy.init()
    node = ObjectSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
