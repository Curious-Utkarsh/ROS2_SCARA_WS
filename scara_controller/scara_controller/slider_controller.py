#!/usr/bin/env python3
"""Let the Joint State Publisher GUI drive the robot.

The slider window publishes where it thinks the joints should be, but the
controllers do not listen to that. They want a trajectory: "go to this position,
and take this long about it". This program does the translation.

    slider GUI  ->  /joint_commands  ->  this program  ->  arm_controller
                                                       ->  gripper_controller

It works exactly the same in Gazebo and on the real robot, because both use the
same controllers. Drag a slider and the real arm follows.

The GUI is told to publish on /joint_commands instead of /joint_states (see the
launch file) so that it does not argue with the joint_state_broadcaster about
where the robot really is.
"""

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ["column_joint", "shoulder_joint", "forearm_joint", "wrist_joint"]
GRIPPER_JOINTS = ["left_finger_joint"]

# How long to give the robot to get there. It must not be zero: a trajectory
# that says "arrive immediately" is impossible and the controller rejects it.
#
# Keep it SHORT. Every message restarts the controller's ramp from a standstill,
# so a long time here means the arm keeps setting off slowly and being
# interrupted, which feels like jerking. A quarter of a second is long enough to
# be accepted and short enough that dragging a slider looks smooth.
TRAVEL_TIME = Duration(sec=0, nanosec=250_000_000)

# The slider window republishes about 40 times a second even when nobody is
# touching it. Ignore anything that has not really moved.
MOVED_ENOUGH = 1e-4


class SliderController(Node):
    def __init__(self):
        super().__init__("slider_controller")

        self.arm_publisher = self.create_publisher(
            JointTrajectory, "arm_controller/joint_trajectory", 10
        )
        self.gripper_publisher = self.create_publisher(
            JointTrajectory, "gripper_controller/joint_trajectory", 10
        )
        self.create_subscription(JointState, "joint_commands", self.slider_callback, 10)

        # What we sent last time, so we can keep quiet when nothing has changed.
        self.last_arm = None
        self.last_gripper = None

        self.get_logger().info("Slider controller started, waiting for the GUI")

    def slider_callback(self, message):
        """A slider moved, so send both controllers where to go."""
        arm_positions = self.pick_out(message, ARM_JOINTS)
        gripper_positions = self.pick_out(message, GRIPPER_JOINTS)

        if arm_positions is None or gripper_positions is None:
            return                       # the GUI is not sending all our joints

        # Only speak when something actually moved.
        #
        # This matters more than it looks. The slider window republishes about
        # 40 times a second whether or not you touch it, and every message we
        # send on a controller's trajectory topic CANCELS whatever that
        # controller was doing. Without this check the sliders would quietly
        # cancel MoveIt's motion 40 times a second and nothing would ever run.
        if self.same_as_last(arm_positions, gripper_positions):
            return

        self.last_arm = arm_positions
        self.last_gripper = gripper_positions

        self.arm_publisher.publish(self.make_trajectory(ARM_JOINTS, arm_positions))
        self.gripper_publisher.publish(
            self.make_trajectory(GRIPPER_JOINTS, gripper_positions)
        )

    def same_as_last(self, arm_positions, gripper_positions):
        """Has anything moved far enough to be worth sending?"""
        if self.last_arm is None or self.last_gripper is None:
            return False                 # nothing sent yet, so send this one

        everything_now = arm_positions + gripper_positions
        everything_before = self.last_arm + self.last_gripper

        for now, before in zip(everything_now, everything_before):
            if abs(now - before) > MOVED_ENOUGH:
                return False
        return True

    def pick_out(self, message, wanted_joints):
        """Find these joints in the message and return their positions in order.

        We look each joint up by name rather than by counting along the list,
        because nothing promises the GUI sends them in the order we expect.
        """
        positions = []
        for joint in wanted_joints:
            if joint not in message.name:
                self.get_logger().warn(f"The GUI did not send {joint}", once=True)
                return None
            positions.append(message.position[message.name.index(joint)])
        return positions

    def make_trajectory(self, joint_names, positions):
        """Wrap positions up as a trajectory with a single stop on the way."""
        goal = JointTrajectoryPoint()
        goal.positions = positions
        goal.time_from_start = TRAVEL_TIME

        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        trajectory.points.append(goal)
        return trajectory


def main():
    rclpy.init()
    node = SliderController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
