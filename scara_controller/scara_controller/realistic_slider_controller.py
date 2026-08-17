#!/usr/bin/env python3
"""Make RViz take as long as the real arm does.

The problem this solves: drag a slider and the robot in RViz is already there,
while the real arm is still stepping its way over. The shoulder is the worst of
it -- its full 0.15 m of travel takes about twelve and a half seconds on the
metal and a quarter of a second on the screen.

So we put a pretend arm in between:

    slider GUI  ->  /joint_commands  ->  [pretend arm]  ->  arm_controller
                                          50 times/s        gripper_controller
                                                              ->  the Arduino

The slider says where to END UP. The pretend arm (scara_stepper_profile) works
out where the real one would have got to after another twentieth of a second,
using the very same top speeds and speed-up rates as the Arduino sketch, and
that is what we send on. RViz follows those, so it crawls at the true speed --
and because the Arduino is fed the same crawl, the two stay together.

    *** THIS IS NOT FEEDBACK ***
    Nothing here asks the robot where it is. The arm has no encoders and cannot
    answer. This is a copy of the Arduino's arithmetic, run on this side of the
    cable. If the arm skips steps, RViz will not know -- same as before.

Two ways to run it:

    output:=trajectory    (the default) send to the controllers, exactly like
                          slider_controller.py does. Use this with the real arm
                          or with mock_components.

    output:=joint_states  publish /joint_states ourselves. For
                          display.launch.py, where there are no controllers at
                          all and nothing else is publishing.

    ros2 run scara_controller realistic_slider_controller.py
    ros2 run scara_controller realistic_slider_controller.py --ros-args \
        -p output:=joint_states -p speed_scale:=4.0
"""

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from scara_controller import stepper_profile

ARM_JOINTS = ["column_joint", "shoulder_joint", "forearm_joint", "wrist_joint"]
GRIPPER_JOINTS = ["left_finger_joint"]
ALL_JOINTS = ARM_JOINTS + GRIPPER_JOINTS

# How often we move the pretend arm on and say where it got to. Fast enough to
# look continuous, slow enough not to flood the controllers.
DEFAULT_RATE_HZ = 50.0

# How far ahead of itself each trajectory point is dated. Two ticks: long enough
# that the controller (running at 100 Hz) always has somewhere to aim, short
# enough that it never lags visibly behind the model.
TICKS_AHEAD = 2


class RealisticSliderController(Node):

    def __init__(self):
        super().__init__("realistic_slider_controller")

        self.declare_parameter("output", "trajectory")
        self.declare_parameter("rate_hz", DEFAULT_RATE_HZ)
        self.declare_parameter("speed_scale", 1.0)

        self.output = str(self.get_parameter("output").value)
        if self.output not in ("trajectory", "joint_states"):
            raise ValueError("output must be 'trajectory' or 'joint_states'")

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / rate_hz
        self.speed_scale = float(self.get_parameter("speed_scale").value)

        # One pretend motor per joint, all starting at zero -- the same pose the
        # firmware and the hardware interface assume at power-up.
        self.axes = dict(zip(ALL_JOINTS, stepper_profile.axes_for(ALL_JOINTS)))

        # Where the sliders last said to end up. None until the GUI speaks, so
        # we do not command a move to zero before we have been told anything.
        self.targets = None

        if self.output == "trajectory":
            self.arm_publisher = self.create_publisher(
                JointTrajectory, "arm_controller/joint_trajectory", 10
            )
            self.gripper_publisher = self.create_publisher(
                JointTrajectory, "gripper_controller/joint_trajectory", 10
            )
            self.state_publisher = None
        else:
            self.arm_publisher = None
            self.gripper_publisher = None
            self.state_publisher = self.create_publisher(JointState, "joint_states", 10)

        self.create_subscription(JointState, "joint_commands", self.slider_callback, 10)
        self.create_timer(self.dt, self.tick)

        self.was_moving = False
        self.report_limits()

    # -- listening --------------------------------------------------------

    def slider_callback(self, message):
        """Remember where the sliders are now. We do not act on it here."""
        targets = {}
        for joint in ALL_JOINTS:
            if joint not in message.name:
                self.get_logger().warn(f"The GUI did not send {joint}", once=True)
                return                   # a half-read pose is worse than none
            targets[joint] = message.position[message.name.index(joint)]
        self.targets = targets

    # -- the pretend arm --------------------------------------------------

    def tick(self):
        """Move every pretend motor on by one tick, then say where they are."""
        if self.targets is None:
            if self.output == "joint_states":
                self.publish_joint_states()   # hold the robot still, visibly
            return

        moving = False
        for joint, axis in self.axes.items():
            axis.advance(self.targets[joint], self.dt, self.speed_scale)
            if not axis.at(self.targets[joint]):
                moving = True

        if self.output == "joint_states":
            # TF goes stale if we ever stop, so this one always publishes.
            self.publish_joint_states()
            self.log_arrival(moving)
            return

        # Trajectory mode. Keep talking while the pretend arm is travelling, and
        # send one last point when it settles so the controller ends up exactly
        # on the target. Then go quiet.
        #
        # Going quiet matters: every message on a controller's trajectory topic
        # cancels whatever that controller was doing, so a node that chattered
        # while idle would cancel MoveIt's motion fifty times a second.
        if moving or self.was_moving:
            self.publish_trajectories()

        self.log_arrival(moving)

    # -- speaking ---------------------------------------------------------

    def publish_trajectories(self):
        self.arm_publisher.publish(self.make_trajectory(ARM_JOINTS))
        self.gripper_publisher.publish(self.make_trajectory(GRIPPER_JOINTS))

    def make_trajectory(self, joint_names):
        """Where the pretend arm is now, dated a couple of ticks into the future.

        It must not be dated zero: a trajectory that says "arrive immediately"
        is impossible and the controller rejects it.
        """
        ahead = self.dt * TICKS_AHEAD

        point = JointTrajectoryPoint()
        point.positions = [self.axes[joint].position for joint in joint_names]
        point.velocities = [self.axes[joint].velocity for joint in joint_names]
        point.time_from_start = Duration(
            sec=int(ahead), nanosec=int((ahead % 1.0) * 1e9)
        )

        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_names)
        trajectory.points.append(point)
        return trajectory

    def publish_joint_states(self):
        """Stand in for the joint_state_broadcaster when there is no controller.

        right_finger_joint is left out on purpose: the URDF makes it a mimic of
        left_finger_joint, so robot_state_publisher works it out for itself.
        """
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(ALL_JOINTS)
        message.position = [self.axes[joint].position for joint in ALL_JOINTS]
        message.velocity = [self.axes[joint].velocity for joint in ALL_JOINTS]
        self.state_publisher.publish(message)

    # -- talking to the human ---------------------------------------------

    def report_limits(self):
        """Print what the pretend arm believes, so a wrong ratio is obvious."""
        self.get_logger().info(
            f"Pretend arm running at {1.0 / self.dt:.0f} Hz, output={self.output}"
            + (f", speed x{self.speed_scale:g}" if self.speed_scale != 1.0 else "")
        )
        for joint in ALL_JOINTS:
            axis = self.axes[joint]
            self.get_logger().info(
                f"  {joint:<18} {axis.max_velocity:.3f} per second "
                f"(speeding up at {axis.max_acceleration:.3f})"
            )

    def log_arrival(self, moving):
        if self.was_moving and not moving:
            self.get_logger().info("arrived")
        self.was_moving = moving


def main():
    rclpy.init()
    node = RealisticSliderController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
