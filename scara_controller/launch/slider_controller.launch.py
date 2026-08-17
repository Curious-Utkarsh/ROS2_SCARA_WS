"""Joint sliders driving the controllers, with no Gazebo and no real robot.

    # sliders that move at the real arm's speed (the default)
    ros2 launch scara_controller slider_controller.launch.py

    # sliders that jump straight there, the old behaviour
    ros2 launch scara_controller slider_controller.launch.py realistic:=False

"realistic" puts realistic_slider_controller.py in the loop instead of
slider_controller.py. It models the Arduino's stepper speeds, so the robot takes
as long to arrive here as it would on the bench. See that file for how.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    realistic_arg = DeclareLaunchArgument(
        name="realistic",
        default_value="True",
        description="Move at the real arm's stepper speed instead of jumping there"
    )
    speed_scale_arg = DeclareLaunchArgument(
        name="speed_scale",
        default_value="1.0",
        description="Multiply the modelled speed, e.g. 4.0 to preview faster"
    )

    realistic = LaunchConfiguration("realistic")
    # A launch argument is text; the node declares speed_scale as a number, so
    # it has to be told which type to parse it as.
    speed_scale = ParameterValue(LaunchConfiguration("speed_scale"), value_type=float)

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("scara_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        remappings=[
            ("/joint_states", "/joint_commands"),
        ]
    )

    slider_control_node = Node(
        package="scara_controller",
        executable="slider_controller.py",
        condition=UnlessCondition(realistic),
    )

    realistic_slider_control_node = Node(
        package="scara_controller",
        executable="realistic_slider_controller.py",
        output="screen",
        parameters=[{"output": "trajectory", "speed_scale": speed_scale}],
        condition=IfCondition(realistic),
    )

    return LaunchDescription([
        realistic_arg,
        speed_scale_arg,
        controller,
        joint_state_publisher_gui_node,
        slider_control_node,
        realistic_slider_control_node,
    ])
