"""Manual teaching mode: mirror the hand-moved physical SCARA in RViz.

Brings up the existing display pipeline (robot_state_publisher + RViz) with the
slider GUI suppressed, and feeds /joint_states from the Arduino serial stream
instead.  No controller_manager, no MoveIt, no trajectory execution.

    ros2 launch scara_controller manual_teaching.launch.py port:=/dev/ttyUSB0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    publish_rate = LaunchConfiguration("publish_rate")

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Serial port the teaching firmware streams on",
    )
    baud_arg = DeclareLaunchArgument("baud", default_value="115200")
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="30.0",
        description="/joint_states publish rate in Hz",
    )

    # Reuse display.launch.py verbatim -- only the slider GUI is turned off, so
    # that this node is the single publisher on /joint_states.
    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scara_description"),
                "launch",
                "display.launch.py",
            )
        ),
        launch_arguments={"use_gui": "False"}.items(),
    )

    serial_joint_publisher = Node(
        package="scara_controller",
        executable="serial_joint_publisher.py",
        name="serial_joint_publisher",
        output="screen",
        parameters=[{
            # Launch arguments arrive as strings; the node declares int/double.
            "port": ParameterValue(port, value_type=str),
            "baud": ParameterValue(baud, value_type=int),
            "publish_rate": ParameterValue(publish_rate, value_type=float),
        }],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        publish_rate_arg,
        display,
        serial_joint_publisher,
    ])
