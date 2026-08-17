"""The colour-sorting conveyor cell, on the real robot.

    ros2 launch scara_bringup real_conveyor.launch.py serial_port:=/dev/ttyACM0

    # then, in another terminal, say which colour to keep:
    ros2 run scara_conveyor hardware_pick_executor --ros-args -p target_color:=R

Nothing moves until you run that second command, exactly as in simulation.

WHAT HAPPENS THEN

    belt runs
      -> a box of the wanted colour drifts under the camera
      -> conveyor_color_gate sees its middle cross the middle of the picture
      -> conveyor_state_machine freezes the belt and triggers the pick
      -> the arm picks and drops it, and reports /scara/pick_completed
      -> the belt starts again and waits for the next one

Boxes of every other colour are never announced, so they simply ride past.

WHICH PICK NODE

    hardware_pick_executor    this cell. Picks blind at the belt centre, because
                              the gate parks every box in the same spot. The
                              drop positions are imported from the simulation's
                              node, so all four colours land where they always
                              did.

    conveyor_pick_executor    the SIMULATION's node. It reads
                              /camera_fixed/depth_image and the Gazebo camera
                              FOV, so it cannot locate a box out here.

Teach the pick pose if your belt is not where the simulated one is:

    ros2 run scara_conveyor hardware_pick_executor --ros-args \
        -p target_color:=R -p pick_source:=taught \
        -p pick_approach:='[0.0, 0.0, 0.0, 0.0]' 

HOW THE BELT ACTUALLY STOPS

The relay is wired to the same Arduino as the arm, and ScaraHardwareInterface
holds that serial port open for as long as the controllers are running -- so
nothing else can open it to say "stop". Instead the hardware interface listens
on /conveyor/run itself and puts "C 1" / "C 0" on the wire it already owns. See
scara_firmware/src/conveyor_relay.cpp.

    conveyor_state_machine -> /conveyor/run -> ScaraHardwareInterface
                                                 -> "C 0" -> Arduino D10 -> relay

WHAT IS DIFFERENT FROM THE SIMULATION

Only the eyes. conveyor_state_machine.py runs unchanged; its Gazebo-only
outputs (spawn_request, release_front, box_removed) simply go nowhere, because
on a real belt nobody spawns or deletes boxes. max_on_belt is 0 so it stops
asking for boxes to be spawned.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyACM0",
        description="Serial port of the Arduino (an Uno is usually /dev/ttyACM0)"
    )
    camera_arg = DeclareLaunchArgument(
        "camera",
        default_value="4",
        description="Which /dev/video device watches the belt (v4l2-ctl --list-devices)"
    )
    center_tolerance_arg = DeclareLaunchArgument(
        "center_tolerance_px",
        default_value="40",
        description=(
            "How near the middle of the picture a box has to be, in pixels, "
            "before the belt is stopped. Widen it if boxes overshoot."
        )
    )
    pick_timeout_arg = DeclareLaunchArgument(
        "pick_timeout_s",
        default_value="120.0",
        description=(
            "Give up on a pick after this long and restart the belt. Longer than "
            "the simulation default, because the real arm is far slower."
        )
    )

    serial_port = LaunchConfiguration("serial_port")
    # Launch arguments are text; these nodes declare the matching parameters as
    # numbers, so each one has to be told which type to parse it as.
    camera = ParameterValue(LaunchConfiguration("camera"), value_type=int)
    center_tolerance = ParameterValue(
        LaunchConfiguration("center_tolerance_px"), value_type=int
    )
    pick_timeout = ParameterValue(
        LaunchConfiguration("pick_timeout_s"), value_type=float
    )

    # ---- The real robot: controllers, the hardware interface (which owns the
    #      serial port and therefore the belt relay), MoveIt and its RViz.
    real_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scara_bringup"),
                "launch",
                "real_bringup.launch.py"
            )
        ),
        launch_arguments={"serial_port": serial_port}.items(),
    )

    # ---- The eyes: says when a wanted box is in the middle of the picture.
    color_gate = Node(
        package="scara_vision",
        executable="conveyor_color_gate",
        name="conveyor_color_gate",
        output="screen",
        parameters=[{
            "camera": camera,
            "center_tolerance_px": center_tolerance,
        }],
    )

    # ---- The brain: reused from the simulation, completely unchanged.
    #      The pick node is NOT started here, exactly as in conveyor.launch.py:
    #      starting it is how you choose the colour and set the cell going.
    state_machine = Node(
        package="scara_conveyor",
        executable="conveyor_state_machine",
        name="conveyor_state_machine",
        output="screen",
        parameters=[{
            # Nobody spawns boxes on a real belt -- a person puts them on it.
            "max_on_belt": 0,
            "pick_timeout_s": pick_timeout,
        }],
    )

    # Give the controllers and MoveIt time to come up first: the state machine
    # latches /conveyor/run as soon as it starts, and the hardware interface has
    # to be listening by then to pass it on to the Arduino.
    delayed = TimerAction(period=10.0, actions=[color_gate, state_machine])

    return LaunchDescription([
        serial_port_arg,
        camera_arg,
        center_tolerance_arg,
        pick_timeout_arg,
        real_robot,
        delayed,
    ])
