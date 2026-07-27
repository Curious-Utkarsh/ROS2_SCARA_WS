#!/usr/bin/env python3
"""Event-driven orchestrator for the conveyor pick-and-place cell.

Ties the spawner, controller, detector, and pick executor together with a
small state machine. Every transition is triggered by a ROS event (a topic
message or a service response) -- the only timer is the 1 Hz spawn cadence
that keeps fresh boxes entering the belt.

    WAIT_FOR_OBJECT
        seed the belt: run it and request the first box
    MOVE_CONVEYOR
        belt advancing, boxes queueing; keep feeding new boxes (<= max_on_belt)
        <- /conveyor/box_at_pickup (name;color)
    STOP_AT_PICKUP
        freeze the belt (run = False)
    DETECT_OBJECT + PICK_OBJECT + DROP_OBJECT + RETURN_HOME
        publish the colour on /scara/pick_command; the executor detects, picks,
        drops in the colour bin, and returns home
        <- /scara/pick_completed
    RESUME_CONVEYOR / NEXT_OBJECT
        delete the picked box, tell the controller, run the belt again, loop
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity
from std_msgs.msg import Bool, Empty, String

from scara_conveyor import common

LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


class ConveyorStateMachine(Node):
    # States
    WAIT = "WAIT_FOR_OBJECT"
    MOVING = "MOVE_CONVEYOR"
    PICKING = "PICK_OBJECT"

    def __init__(self):
        super().__init__("conveyor_state_machine")

        self.declare_parameter("world_name", common.WORLD_NAME)
        self.declare_parameter("max_on_belt", 3)
        self.declare_parameter("spawn_period_s", 1.0)
        self.world = self.get_parameter("world_name").value
        self.max_on_belt = int(self.get_parameter("max_on_belt").value)
        spawn_period = float(self.get_parameter("spawn_period_s").value)

        self.state = self.WAIT
        self.on_belt = 0
        self.pending_spawns = 0
        self.last_spawn_request = self.get_clock().now()
        self.current_name = None
        self.current_color = None

        # Publishers.
        self.spawn_pub = self.create_publisher(Empty, common.TOPIC_SPAWN_REQUEST, 10)
        self.run_pub = self.create_publisher(Bool, common.TOPIC_RUN, 10)
        self.command_pub = self.create_publisher(
            String, common.TOPIC_PICK_COMMAND, LATCHED
        )
        self.removed_pub = self.create_publisher(String, common.TOPIC_BOX_REMOVED, 10)

        # Service client (bridged to Gazebo UserCommands).
        self.remove_client = self.create_client(
            DeleteEntity, common.remove_service(self.world)
        )

        # Subscriptions (events that drive the machine).
        self.create_subscription(String, common.TOPIC_SPAWNED, self.on_spawned, 10)
        self.create_subscription(
            String, common.TOPIC_BOX_AT_PICKUP, self.on_box_at_pickup, 10
        )
        self.create_subscription(
            Bool, common.TOPIC_PICK_COMPLETED, self.on_pick_completed, LATCHED
        )

        self.create_timer(spawn_period, self.on_spawn_tick)
        # Seed the cell shortly after startup so subscribers are connected.
        self.create_timer(2.0, self.seed_once)
        self.get_logger().info("Conveyor state machine ready.")

    # -------------------------------------------------------------- seeding --
    def seed_once(self):
        if self.state != self.WAIT:
            return
        self.get_logger().info("Seeding belt: RUN + first spawn.")
        self.set_running(True)
        self.request_spawn()
        self.state = self.MOVING

    def on_spawn_tick(self):
        # Keep the belt fed while it is moving and not over-full.
        if self.state != self.MOVING:
            return
        # Self-heal: if a requested spawn was never confirmed (e.g. the create
        # service wasn't ready yet at startup), don't let pending_spawns wedge
        # the feeder shut.
        if self.pending_spawns > 0:
            elapsed = (self.get_clock().now() - self.last_spawn_request).nanoseconds / 1e9
            if elapsed > 3.0:
                self.get_logger().warning("Pending spawn timed out; resetting.")
                self.pending_spawns = 0
        if self.on_belt + self.pending_spawns >= self.max_on_belt:
            return
        self.request_spawn()

    # -------------------------------------------------------------- events ---
    def on_spawned(self, _msg):
        self.on_belt += 1
        self.pending_spawns = max(0, self.pending_spawns - 1)

    def on_box_at_pickup(self, msg):
        if self.state != self.MOVING:
            return
        try:
            name, color = msg.data.split(";")
        except ValueError:
            self.get_logger().warning(f"Bad box_at_pickup payload: {msg.data!r}")
            return
        self.current_name = name
        self.current_color = color
        self.state = self.PICKING
        self.get_logger().info(f"[STOP_AT_PICKUP] {name} ({color}); freezing belt.")
        self.set_running(False)
        self.get_logger().info(f"[DETECT+PICK] commanding pick of {color}.")
        self.command_pub.publish(String(data=color))

    def on_pick_completed(self, msg):
        if self.state != self.PICKING:
            return
        if not msg.data:
            self.get_logger().warning(
                f"Pick of {self.current_name} reported failure; removing anyway."
            )
        self.get_logger().info(f"[RETURN_HOME done] removing {self.current_name}.")
        self.delete_box(self.current_name)
        self.removed_pub.publish(String(data=self.current_name))
        self.on_belt = max(0, self.on_belt - 1)
        self.current_name = None
        self.current_color = None
        self.state = self.MOVING
        self.get_logger().info("[RESUME_CONVEYOR] belt running; next object.")
        self.set_running(True)

    # -------------------------------------------------------------- helpers --
    def request_spawn(self):
        self.pending_spawns += 1
        self.last_spawn_request = self.get_clock().now()
        self.spawn_pub.publish(Empty())

    def set_running(self, running):
        self.run_pub.publish(Bool(data=running))

    def delete_box(self, name):
        if name is None:
            return
        if not self.remove_client.service_is_ready():
            self.get_logger().warning(
                f"Remove service not ready; leaving {name} in the world."
            )
            return
        req = DeleteEntity.Request()
        req.entity = Entity(name=name, type=Entity.MODEL)
        self.remove_client.call_async(req)


def main():
    rclpy.init()
    node = ConveyorStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
