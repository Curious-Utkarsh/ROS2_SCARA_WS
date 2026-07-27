#!/usr/bin/env python3
"""Event-driven orchestrator for the conveyor sorting cell.

The belt stays idle until the operator starts the pick node (which announces the
wanted colour on /conveyor/target_color). From then on, one box at a time:

    WAIT_FOR_OBJECT   idle until a target colour is announced
    MOVE_CONVEYOR     belt running + feeding boxes (<= max_on_belt)
        <- /conveyor/box_at_pickup (name;color)
    decide:
      colour == target  -> STOP_AT_PICKUP  freeze belt
                           PICK/DROP/HOME   trigger the pick node, wait for it
                           <- /scara/pick_completed
                           RESUME           remove box, run belt again
      colour != target  -> PASS             release the box; it rides off the
                                            far edge and drops -- belt keeps going

Every transition is triggered by a ROS event; the only timer is the spawn
cadence. This node decides; the controller moves and removes boxes.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String

from scara_conveyor import common

LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


class ConveyorStateMachine(Node):
    WAIT = "WAIT_FOR_OBJECT"
    MOVING = "MOVE_CONVEYOR"
    PICKING = "PICK_OBJECT"

    def __init__(self):
        super().__init__("conveyor_state_machine")

        self.declare_parameter("max_on_belt", 3)
        self.declare_parameter("spawn_period_s", 3.0)
        self.declare_parameter("pick_timeout_s", 45.0)
        self.max_on_belt = int(self.get_parameter("max_on_belt").value)
        spawn_period = float(self.get_parameter("spawn_period_s").value)
        self.pick_timeout = float(self.get_parameter("pick_timeout_s").value)

        self.state = self.WAIT
        self.target_color = None
        self.on_belt = 0
        self.pending_spawns = 0
        self.last_spawn_request = self.get_clock().now()
        self.pick_started = None
        self.current_name = None
        self.current_color = None

        self.spawn_pub = self.create_publisher(Empty, common.TOPIC_SPAWN_REQUEST, 10)
        self.run_pub = self.create_publisher(Bool, common.TOPIC_RUN, LATCHED)
        self.execute_pub = self.create_publisher(String, common.TOPIC_EXECUTE_PICK, 10)
        self.release_pub = self.create_publisher(String, common.TOPIC_RELEASE_FRONT, 10)
        self.removed_pub = self.create_publisher(String, common.TOPIC_BOX_REMOVED, 10)

        self.create_subscription(
            String, common.TOPIC_TARGET_COLOR, self.on_target_color, LATCHED
        )
        self.create_subscription(String, common.TOPIC_SPAWNED, self.on_spawned, 10)
        self.create_subscription(
            String, common.TOPIC_BOX_AT_PICKUP, self.on_box_at_pickup, 10
        )
        self.create_subscription(
            Bool, common.TOPIC_PICK_COMPLETED, self.on_pick_completed, LATCHED
        )

        self.create_timer(spawn_period, self.on_spawn_tick)
        self.create_timer(1.0, self.on_watchdog)
        self.get_logger().info(
            "State machine idle; start the pick node to choose a colour and begin."
        )

    # ------------------------------------------------------------- startup ---
    def on_target_color(self, msg):
        color = msg.data.strip().upper()
        if color not in common.COLORS:
            return
        self.target_color = color
        if self.state == self.WAIT:
            self.get_logger().info(f"Target = {color}. Starting the belt.")
            self.set_running(True)
            self.request_spawn()
            self.state = self.MOVING

    # -------------------------------------------------------------- feeding --
    def on_spawn_tick(self):
        if self.state != self.MOVING:
            return
        if self.pending_spawns > 0:
            elapsed = (self.get_clock().now() - self.last_spawn_request).nanoseconds / 1e9
            if elapsed > 3.0:
                self.pending_spawns = 0  # a dropped request; don't wedge the feeder
        if self.on_belt + self.pending_spawns >= self.max_on_belt:
            return
        self.request_spawn()

    def on_spawned(self, _msg):
        self.on_belt += 1
        self.pending_spawns = max(0, self.pending_spawns - 1)

    # -------------------------------------------------------------- sorting --
    def on_box_at_pickup(self, msg):
        if self.state != self.MOVING:
            return
        try:
            name, color = msg.data.split(";")
        except ValueError:
            return

        if color == self.target_color:
            self.current_name = name
            self.current_color = color
            self.state = self.PICKING
            self.pick_started = self.get_clock().now()
            self.get_logger().info(f"[KEEP] {name} ({color}): freeze + pick.")
            self.set_running(False)
            self.execute_pub.publish(String(data=f"{name};{color}"))
        else:
            self.get_logger().info(f"[PASS] {name} ({color}): let it drop off.")
            self.release_pub.publish(String(data=name))
            self.on_belt = max(0, self.on_belt - 1)

    def on_pick_completed(self, msg):
        if self.state != self.PICKING:
            return
        if not msg.data:
            self.get_logger().warning(f"Pick of {self.current_name} failed.")
        self.get_logger().info(f"[RESUME] removing {self.current_name}; belt runs.")
        self.removed_pub.publish(String(data=self.current_name))
        self.on_belt = max(0, self.on_belt - 1)
        self.finish_pick()

    def on_watchdog(self):
        if self.state != self.PICKING or self.pick_started is None:
            return
        elapsed = (self.get_clock().now() - self.pick_started).nanoseconds / 1e9
        if elapsed > self.pick_timeout:
            self.get_logger().error(
                f"Pick of {self.current_name} timed out; removing and resuming."
            )
            self.removed_pub.publish(String(data=self.current_name))
            self.on_belt = max(0, self.on_belt - 1)
            self.finish_pick()

    def finish_pick(self):
        self.current_name = None
        self.current_color = None
        self.pick_started = None
        self.state = self.MOVING
        self.set_running(True)

    # -------------------------------------------------------------- helpers --
    def request_spawn(self):
        self.pending_spawns += 1
        self.last_spawn_request = self.get_clock().now()
        self.spawn_pub.publish(Empty())

    def set_running(self, running):
        self.run_pub.publish(Bool(data=running))


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
