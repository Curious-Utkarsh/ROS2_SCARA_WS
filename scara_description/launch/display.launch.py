"""RViz and the sliders, with no Gazebo, no controllers and no robot.

    # sliders that take as long as the real arm would (recommended)
    ros2 launch scara_description display.launch.py realistic:=True

    # sliders that move the model the instant you drag them
    ros2 launch scara_description display.launch.py

With realistic:=True the GUI publishes on /joint_commands instead of
/joint_states, and realistic_slider_controller.py becomes the one thing feeding
/joint_states -- easing each joint across at the speed its stepper could really
manage. Watch the shoulder: 0.15 m takes about twelve seconds, which is the
truth the instant version was hiding.
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    AndSubstitution, Command, LaunchConfiguration, NotSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    scara_description_dir = get_package_share_directory("scara_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(scara_description_dir, "urdf", "scara.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    # Off when something else owns /joint_states -- e.g. the manual teaching
    # serial bridge.  Two publishers make RViz flicker between them.
    use_gui_arg = DeclareLaunchArgument(
        name="use_gui",
        default_value="True",
        description="Run joint_state_publisher_gui as the /joint_states source"
    )

    realistic_arg = DeclareLaunchArgument(
        name="realistic",
        default_value="False",
        description=(
            "Ease the model across at the real arm's stepper speed instead of "
            "jumping there. Puts realistic_slider_controller.py between the GUI "
            "and /joint_states."
        )
    )
    speed_scale_arg = DeclareLaunchArgument(
        name="speed_scale",
        default_value="1.0",
        description="Multiply the modelled speed, e.g. 4.0 to preview faster"
    )

    use_gui = LaunchConfiguration("use_gui")
    realistic = LaunchConfiguration("realistic")
    # A launch argument is text; the node declares speed_scale as a number.
    speed_scale = ParameterValue(LaunchConfiguration("speed_scale"), value_type=float)

    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_sim:=False",       # no Gazebo — mock_components/GenericSystem
            " is_ignition:=False",  # must always be passed to avoid xacro error
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,  # no sim clock in display-only mode
        }]
    )

    # Two versions of the same window. The plain one owns /joint_states itself;
    # the realistic one hands its output to the pretend arm below, which then
    # owns /joint_states. Exactly one of them ever runs, because two publishers
    # on /joint_states make the model flicker between them.
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(AndSubstitution(use_gui, NotSubstitution(realistic)))
    )

    realistic_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        remappings=[("/joint_states", "/joint_commands")],
        condition=IfCondition(AndSubstitution(use_gui, realistic))
    )

    realistic_slider_controller_node = Node(
        package="scara_controller",
        executable="realistic_slider_controller.py",
        output="screen",
        parameters=[{"output": "joint_states", "speed_scale": speed_scale}],
        condition=IfCondition(realistic),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        # Wall time, matching robot_state_publisher above. There is no /clock in
        # display-only mode, so a node left on sim time would sit at t=0 and
        # discard every transform as far too old.
        parameters=[{"use_sim_time": False}],
        arguments=["-d", os.path.join(scara_description_dir, "rviz", "display.rviz")],
    )

    return LaunchDescription([
        model_arg,
        use_gui_arg,
        realistic_arg,
        speed_scale_arg,
        joint_state_publisher_gui_node,
        realistic_gui_node,
        realistic_slider_controller_node,
        robot_state_publisher_node,
        rviz_node,
    ])