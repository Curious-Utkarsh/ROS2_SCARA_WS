import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    is_sim = LaunchConfiguration("is_sim")
    is_ignition = LaunchConfiguration("is_ignition")
    is_real = LaunchConfiguration("is_real")
    serial_port = LaunchConfiguration("serial_port")
    wait_for_done = LaunchConfiguration("wait_for_done")

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="False"       # False = mock_components (slider GUI)
    )
    is_ignition_arg = DeclareLaunchArgument(
        "is_ignition",
        default_value="False"       # True only when launching inside Ignition Fortress
    )
    # Real robot: only consulted when is_sim is False.  Together the two flags
    # pick the hardware plugin in scara_description/urdf/ros2control.xacro.
    is_real_arg = DeclareLaunchArgument(
        "is_real",
        default_value="False",
        description="True = drive the real arm through scara_firmware over serial"
    )
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port the Arduino enumerates on (often /dev/ttyACM0 for an Uno)"
    )
    wait_for_done_arg = DeclareLaunchArgument(
        "wait_for_done",
        default_value="False",
        description="Wait for the Arduino's DONE after every setpoint (off = smoother)"
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("scara_description"),
                    "urdf",
                    "scara.urdf.xacro",
                ),
                " is_sim:=", is_sim,
                " is_ignition:=", is_ignition,
                " is_real:=", is_real,
                " serial_port:=", serial_port,
                " wait_for_done:=", wait_for_done,
            ]
        ),
        value_type=str,
    )

    # In sim, Gazebo's own gz_ros2_control/ign_ros2_control system plugin (loaded via
    # scara_description/urdf/gazebo.xacro) already hosts robot_state_publisher and the
    # controller_manager -- a second copy here would just fight over the same names, and
    # its ros2_control_node crashes on startup because GazeboSimSystem/IgnitionSystem
    # aren't loadable outside of Gazebo's own process. Only the spawners are needed to
    # activate controllers on whichever controller_manager is actually running.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": is_sim,
        }],
        condition=UnlessCondition(is_sim),
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": is_sim,
            },
            os.path.join(
                get_package_share_directory("scara_controller"),
                "config",
                "scara_controllers.yaml",
            ),
        ],
        condition=UnlessCondition(is_sim),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    return LaunchDescription([
        is_sim_arg,
        is_ignition_arg,
        is_real_arg,
        serial_port_arg,
        wait_for_done_arg,
        robot_state_publisher_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])