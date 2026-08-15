"""Bring up the SCARA in simulation: Gazebo + ros2_control + MoveIt + RViz.

This is the launch file to use when there is no real robot attached.  The
hardware behind ros2_control is Gazebo (see is_sim in
scara_description/urdf/ros2control.xacro).

    ros2 launch scara_bringup simulation_bringup.launch.py

Its real-robot twin is real_bringup.launch.py.  The two files include the same
three sub-launch files and differ only in the arguments they pass.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Optional Joint State Publisher GUI for driving the joints by hand.
    # Off by default so that MoveIt is the only thing commanding the arm.
    use_slider_arg = DeclareLaunchArgument(
        "use_slider",
        default_value="False",
        description="Also open the Joint State Publisher GUI to move joints by hand"
    )
    use_slider = LaunchConfiguration("use_slider")

    # ------------------- Gazebo -------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scara_description"),
                "launch",
                "gazebo.launch.py"
            )
        )
    )

    # ------------------- Controllers -------------------
    # is_sim:=True -> Gazebo hosts the controller_manager, so this only spawns
    # the controllers onto it.
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scara_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ------------------- MoveIt + RViz -------------------
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scara_moveit"),
                "launch",
                "moveit.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ------------------- Joint State Publisher GUI (optional) -------------------
    # Publishes on /joint_commands, not /joint_states, so it never fights with
    # the joint_state_broadcaster.  slider_controller turns those values into
    # trajectories for the controllers.
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        remappings=[("/joint_states", "/joint_commands")],
        condition=IfCondition(use_slider),
    )

    slider_controller_node = Node(
        package="scara_controller",
        executable="slider_controller.py",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slider),
    )

    return LaunchDescription([
        use_slider_arg,
        gazebo,
        controller,
        moveit,
        joint_state_publisher_gui_node,
        slider_controller_node,
    ])
