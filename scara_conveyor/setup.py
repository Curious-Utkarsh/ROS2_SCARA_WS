import os
from glob import glob

from setuptools import find_packages, setup

package_name = "scara_conveyor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "docs"), glob("docs/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="utk",
    maintainer_email="kutkarsh706@gmail.com",
    description="Event-driven conveyor pick-and-place mini-project for the SCARA cell.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "object_spawner = scara_conveyor.object_spawner:main",
            "conveyor_controller = scara_conveyor.conveyor_controller:main",
            "conveyor_pick_executor = scara_conveyor.conveyor_pick_executor:main",
            "hardware_pick_executor = scara_conveyor.hardware_pick_executor:main",
            "conveyor_state_machine = scara_conveyor.conveyor_state_machine:main",
            "arm_camera_viewer = scara_conveyor.arm_camera_viewer:main",
        ],
    },
)
