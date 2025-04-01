from dataclasses import dataclass

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

AvailableTypes = str | list | float | int


@dataclass
class Parameter:
    name: str
    value: AvailableTypes

    def dict(self) -> dict[str, AvailableTypes]:
        return {self.name: self.value}


def generate_launch_description():
    pkg_dvl_driver = get_package_share_directory("waterlinked_dvl_driver")
    pkg_vectornav_imu_driver = get_package_share_directory("vectornav")

    dvl_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dvl_driver, "launch", "dvl.launch.py"),
        ),
        launch_arguments={}.items(),
    )

    vectornav_imu_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vectornav_imu_driver, "launch", "vectornav.launch.py"),
        ),
        launch_arguments={}.items(),
    )

    parameters: list[Parameter] = [
        Parameter("imu0", "/vectornav/imu"),
        Parameter(
            "imu0_config",
            [
                False,
                False,
                False,
                True,
                True,
                True,
                False,
                False,
                False,
                True,
                True,
                True,
                True,
                True,
                True,
            ],
        ),
        Parameter("imu0_differential", False),
        Parameter("imu0_relative", True),
        Parameter("imu0_remove_gravitational_acceleration", True),
        Parameter("imu0_queue_size", 10),
        Parameter("odom0", "/waterlinked_dvl_driver/odom"),
        Parameter(
            "odom0_config",
            [
                False,
                False,
                False,
                False,
                False,
                False,
                True,
                True,
                True,
                False,
                False,
                False,
                False,
                False,
                False,
            ],
        ),
        Parameter("odom0_differential", False),
        Parameter("odom0_relative", True),
        Parameter("odom0_queue_size", 10),
        #        Parameter("pose0", "/depth/pose"),
        #        Parameter("pose0_config", [False, False, True,
        #                                   False, False, False,
        #                                   False, False, False,
        #                                   False, False, False,
        #                                   False, False, False]),
        #        Parameter("pose0_differential", True),
        #        Parameter("pose0_relative", True),
        #        Parameter("pose0_queue_size", 10),
        Parameter("print_diagnostics", True),
        Parameter("publish_acceleration", True),
        # Parameter("debug", True),
        # Parameter("debug_out_file", os.path.expanduser("~/debug.txt")),
        Parameter("frequency", 10.0),
        # Parameter("initial_state", [0.0] * 15),
    ]
    import rich

    rich.print([p.dict() for p in parameters])
    return LaunchDescription(
        [
            dvl_driver,
            vectornav_imu_driver,
            Node(
                package="robot_localization",
                namespace="subjugator_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[p.dict() for p in parameters],
            ),
        ],
    )
