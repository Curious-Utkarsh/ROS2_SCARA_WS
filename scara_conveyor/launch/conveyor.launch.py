import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_name = LaunchConfiguration("world_name")
    world_name_arg = DeclareLaunchArgument("world_name", default_value="conveyor")

    scara_description = get_package_share_directory("scara_description")
    scara_controller = get_package_share_directory("scara_controller")
    scara_moveit = get_package_share_directory("scara_moveit")

    # ---- Reuse the existing sim stack unchanged (just point at conveyor world).
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scara_description, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world_name": world_name}.items(),
    )
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scara_controller, "launch", "controller.launch.py")
        ),
        launch_arguments={"is_sim": "True"}.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scara_moveit, "launch", "moveit.launch.py")
        ),
        launch_arguments={"is_sim": "True"}.items(),
    )

    # ---- ROS<->Gazebo SERVICE bridge for spawn / move / delete (UserCommands).
    #      Started a little after Gazebo so the world's services exist.
    service_bridge = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="conveyor_service_bridge",
                output="screen",
                arguments=[
                    "/world/conveyor/create@ros_gz_interfaces/srv/SpawnEntity",
                    "/world/conveyor/set_pose@ros_gz_interfaces/srv/SetEntityPose",
                    "/world/conveyor/remove@ros_gz_interfaces/srv/DeleteEntity",
                ],
            )
        ],
    )

    # ---- Perception (reused as-is) + the four conveyor nodes.
    #      Delayed so Gazebo, controllers, MoveIt, and the bridge are all up.
    detector = Node(
        package="scara_vision",
        executable="fixed_camera_color_detector",
        name="fixed_camera_color_detector",
        output="screen",
    )
    # View-only feed from the robot's arm-mounted camera (no role in the logic).
    arm_camera = Node(
        package="scara_conveyor",
        executable="arm_camera_viewer",
        name="arm_camera_viewer",
        output="screen",
    )
    spawner = Node(
        package="scara_conveyor",
        executable="object_spawner",
        name="object_spawner",
        output="screen",
    )
    conveyor = Node(
        package="scara_conveyor",
        executable="conveyor_controller",
        name="conveyor_controller",
        output="screen",
    )
    state_machine = Node(
        package="scara_conveyor",
        executable="conveyor_state_machine",
        name="conveyor_state_machine",
        output="screen",
    )

    # NOTE: the pick node (conveyor_pick_executor) is intentionally NOT launched
    # here. Start it yourself with the colour you want to keep, e.g.:
    #   ros2 run scara_conveyor conveyor_pick_executor --ros-args -p target_color:=R
    # The belt stays idle until you do; then that colour is picked and every
    # other colour rides off the far edge of the table.
    delayed_nodes = TimerAction(
        period=8.0,
        actions=[detector, arm_camera, spawner, conveyor, state_machine],
    )

    return LaunchDescription(
        [world_name_arg, gazebo, controller, moveit, service_bridge, delayed_nodes]
    )
