"""Bring up the REAL SCARA robot.

The hardware swap happens in one place -- is_real:=True makes
scara_description/urdf/ros2control.xacro load
scara_firmware/ScaraHardwareInterface instead of a Gazebo plugin.

There are two ways to drive it, and you pick ONE:

    # MoveIt in RViz (the default)
    ros2 launch scara_bringup real_bringup.launch.py serial_port:=/dev/ttyACM0

    # Joint sliders with the plain RViz view, no MoveIt
    ros2 launch scara_bringup real_bringup.launch.py serial_port:=/dev/ttyACM0 \
        use_slider:=True use_moveit:=False

By default the sliders go through realistic_slider_controller.py, which models
the Arduino's stepper speeds so RViz takes as long to arrive as the metal does.
Add realistic:=False for the old behaviour, where the picture is there and the
arm is still on its way.

DO NOT RUN BOTH AT ONCE. The slider window republishes about 40 times a second,
and every message on a controller's trajectory topic cancels whatever that
controller was doing -- so the sliders would cancel MoveIt's motion over and
over and nothing would ever complete. Keeping them apart is why use_moveit
exists.

OPEN LOOP -- there are no encoders. Park the arm at its zero pose BEFORE
powering the drivers, because that is the position both the firmware and ROS
assume at startup. See scara_firmware/docs/real_hardware.md.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyACM0",
        description="Serial port of the Arduino (an Uno is usually /dev/ttyACM0)"
    )
    use_moveit_arg = DeclareLaunchArgument(
        "use_moveit",
        default_value="True",
        description="Start MoveIt and its RViz. Turn OFF when using the sliders."
    )
    use_slider_arg = DeclareLaunchArgument(
        "use_slider",
        default_value="False",
        description="Open the Joint State Publisher GUI. Turn MoveIt OFF when using it."
    )
    wait_for_done_arg = DeclareLaunchArgument(
        "wait_for_done",
        default_value="False",
        description=(
            "Wait for the Arduino's DONE after every setpoint. Off gives smooth "
            "motion; on gives strict point-to-point moves that stop at each step."
        )
    )

    realistic_arg = DeclareLaunchArgument(
        "realistic",
        default_value="True",
        description=(
            "Sliders move the model at the real arm's stepper speed instead of "
            "jumping straight there. Only affects the use_slider path."
        )
    )
    speed_scale_arg = DeclareLaunchArgument(
        "speed_scale",
        default_value="1.0",
        description="Multiply the modelled speed, e.g. 4.0 to preview faster"
    )

    serial_port = LaunchConfiguration("serial_port")
    use_moveit = LaunchConfiguration("use_moveit")
    use_slider = LaunchConfiguration("use_slider")
    wait_for_done = LaunchConfiguration("wait_for_done")
    realistic = LaunchConfiguration("realistic")
    # A launch argument is text; the node declares speed_scale as a number.
    speed_scale = ParameterValue(LaunchConfiguration("speed_scale"), value_type=float)

    # ------------------- Controllers + real hardware -------------------
    # Always started. This owns robot_state_publisher and the controller_manager,
    # and it is what actually talks to the Arduino.
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scara_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={
            "is_sim": "False",
            "is_ignition": "False",
            "is_real": "True",
            "serial_port": serial_port,
            "wait_for_done": wait_for_done,
        }.items()
    )

    # ------------------- Option A: MoveIt + its own RViz -------------------
    # is_sim:=False also sets use_sim_time:=False -- there is no /clock on a real
    # robot, so everything must run on wall time.
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scara_moveit"),
                "launch",
                "moveit.launch.py"
            )
        ),
        launch_arguments={
            "is_sim": "False",
            "is_ignition": "False",
            "is_real": "True",
            "serial_port": serial_port,
        }.items(),
        condition=IfCondition(use_moveit),
    )

    # ------------------- Option B: the plain RViz view -------------------
    # The same window display.launch.py would give you, using the same RViz
    # config. We start rviz2 directly rather than including display.launch.py,
    # because that file also starts its own robot_state_publisher and two of
    # those fighting over /tf makes the model flicker.
    plain_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": False}],
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("scara_description"), "rviz", "display.rviz"
            ),
        ],
        condition=UnlessCondition(use_moveit),
    )

    # ------------------- Joint sliders (optional) -------------------
    # The GUI publishes on /joint_commands, not /joint_states, so it never
    # competes with the joint_state_broadcaster. slider_controller turns each
    # change into a short trajectory for the controllers -- the same path MoveIt
    # uses, so the real arm follows.
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        remappings=[("/joint_states", "/joint_commands")],
        condition=IfCondition(use_slider),
    )

    # Two ways to turn slider changes into controller commands, and use_slider
    # picks between them with `realistic`:
    #
    #   slider_controller.py            straight there in a fixed 0.25 s
    #   realistic_slider_controller.py  at the speed the steppers can manage,
    #                                   so RViz and the arm arrive together
    slider_controller_node = Node(
        package="scara_controller",
        executable="slider_controller.py",
        output="screen",
        condition=IfCondition(
            AndSubstitution(use_slider, NotSubstitution(realistic))
        ),
    )

    realistic_slider_controller_node = Node(
        package="scara_controller",
        executable="realistic_slider_controller.py",
        output="screen",
        parameters=[{"output": "trajectory", "speed_scale": speed_scale}],
        condition=IfCondition(AndSubstitution(use_slider, realistic)),
    )

    return LaunchDescription([
        serial_port_arg,
        use_moveit_arg,
        use_slider_arg,
        wait_for_done_arg,
        realistic_arg,
        speed_scale_arg,
        controller,
        moveit,
        plain_rviz,
        joint_state_publisher_gui_node,
        slider_controller_node,
        realistic_slider_controller_node,
    ])
