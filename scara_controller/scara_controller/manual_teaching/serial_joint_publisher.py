#!/usr/bin/env python3
"""Manual teaching mode: show the real arm's position in RViz.

You push the arm around by hand, the Arduino measures where the joints are and
sends the numbers down the USB cable, and this program passes them on to RViz.

    Arduino  ->  serial cable  ->  this program  ->  /joint_states  ->  RViz

The sketch on the other end is
scara_firmware/firmware/manual_teaching_mode/manual_teaching_mode.ino and it
keeps the motors switched off, so the arm is free to move.

This program only *displays* the arm. It never commands anything. Run it
INSTEAD of joint_state_publisher_gui -- if both are running they fight over
/joint_states and RViz flickers between them.

What arrives on the cable:
    "J 0.5 -0.05 0.0 0.0 0.0"   the five joint positions, in radians and metres
    "# something"               a message for a human, we just print it
"""

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

# The joints, in the order the Arduino sends them.
# right_finger_joint is missing on purpose: it is a copy of left_finger_joint,
# so RViz works it out for itself.
JOINT_NAMES = [
    "column_joint",
    "shoulder_joint",
    "forearm_joint",
    "wrist_joint",
    "left_finger_joint",
]

# How far each joint is allowed to go, straight from the robot model (URDF).
# If the Arduino miscounts, this stops RViz drawing an impossible robot.
JOINT_MIN = [-2.0, -0.15, -2.0, -2.0, -0.06]
JOINT_MAX = [2.0, 0.02, 2.0, 2.0, 0.0]


class SerialJointPublisher(Node):
    def __init__(self):
        super().__init__("serial_joint_publisher")

        # Settings you can change from the launch file or the command line.
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("zero_on_start", True)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        rate = float(self.get_parameter("publish_rate").value)

        self.arduino = None          # the open serial port, once we have one
        self.text = ""               # characters that arrived but had no newline yet
        self.positions = [0.0] * len(JOINT_NAMES)
        self.messages_received = 0
        self.complained = False

        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.create_service(Trigger, "~/zero", self.zero_callback)

        # Everything happens in here, once every 1/rate seconds.
        self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info(
            f"Manual teaching started. Reading {self.port}, publishing at {rate:.0f} Hz"
        )

    # -- this runs over and over -------------------------------------------

    def timer_callback(self):
        if self.arduino is None:
            self.open_port()
        else:
            self.read_from_arduino()
        self.publish_joint_states()

    def open_port(self):
        """Try to open the serial port. It is fine if the board is not plugged in yet."""
        try:
            self.arduino = serial.Serial(self.port, self.baud, timeout=0)
        except (serial.SerialException, OSError) as error:
            self.get_logger().warn(f"Cannot open {self.port}: {error}", once=True)
            return

        self.arduino.reset_input_buffer()      # skip whatever was already there
        self.get_logger().info(f"Opened {self.port}")

        if self.get_parameter("zero_on_start").value:
            self.arduino.write(b"Z\n")

    def read_from_arduino(self):
        """Collect whatever has arrived and deal with any complete lines.

        timeout=0 means reading never waits, so this cannot hold up the program.
        Characters can turn up in the middle of a line, which is why we keep the
        leftovers in self.text until a newline finishes them off.
        """
        try:
            waiting = self.arduino.in_waiting
            if waiting > 0:
                self.text += self.arduino.read(waiting).decode("ascii", errors="ignore")
        except (serial.SerialException, OSError) as error:
            self.get_logger().warn(f"Lost the connection, will try again: {error}")
            self.close_port()
            return

        while "\n" in self.text:
            line, self.text = self.text.split("\n", 1)
            self.handle_line(line.strip())

    def handle_line(self, line):
        """Deal with one complete line from the Arduino."""
        if line.startswith("#"):
            self.get_logger().info(f"arduino: {line[1:].strip()}")
            return

        if not line.startswith("J"):
            return                              # not a message we understand

        numbers = line[1:].split()
        if len(numbers) != len(JOINT_NAMES):
            self.get_logger().warn(f"Ignoring a broken message: {line}", once=True)
            return

        try:
            values = [float(number) for number in numbers]
        except ValueError:
            self.get_logger().warn(f"Ignoring a message with bad numbers: {line}", once=True)
            return

        # Keep every joint inside its limits before we believe it.
        for i in range(len(JOINT_NAMES)):
            self.positions[i] = min(max(values[i], JOINT_MIN[i]), JOINT_MAX[i])

        self.messages_received += 1

    def publish_joint_states(self):
        """Send the latest positions to RViz.

        We publish even when nothing new arrived, so the robot freezes on screen
        instead of disappearing.
        """
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = JOINT_NAMES
        message.position = list(self.positions)
        self.publisher.publish(message)

        if self.messages_received == 0 and not self.complained:
            self.get_logger().warn("Nothing from the Arduino yet, is the right sketch flashed?")
            self.complained = True

    def close_port(self):
        try:
            self.arduino.close()
        except (serial.SerialException, OSError):
            pass
        self.arduino = None
        self.text = ""

    # -- "ros2 service call /serial_joint_publisher/zero std_srvs/srv/Trigger" --

    def zero_callback(self, request, response):
        """Tell the Arduino that wherever the arm is now counts as zero."""
        del request
        if self.arduino is None:
            response.success = False
            response.message = "the serial port is not open"
            return response

        try:
            self.arduino.write(b"Z\n")
        except (serial.SerialException, OSError) as error:
            response.success = False
            response.message = f"could not write to the Arduino: {error}"
            return response

        response.success = True
        response.message = "this position is now zero"
        return response


def main():
    rclpy.init()
    node = SerialJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
