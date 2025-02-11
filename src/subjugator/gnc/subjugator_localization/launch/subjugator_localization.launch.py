from dataclasses import asdict, dataclass

from launch import LaunchDescription
from launch_ros.actions import Node

AvailableTypes = str | list | float | int


@dataclass
class Parameter:
    name: str
    value: AvailableTypes

    dict = asdict


def generate_launch_description():
    parameters: list[Parameter] = [
        Parameter("imu0", "/imu/data_raw/imu"),
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
                False,
                True,
                True,
                True,
            ],
        ),
        Parameter("imu0_differential", True),
        Parameter("imu0_relative", True),
        Parameter("imu0_remove_gravitational_acceleration", True),
        Parameter("imu0_queue_size", 10),
        Parameter("odom0", "/dvl/odom"),
        Parameter(
            "odom0_config",
            [
                True,
                True,
                False,
                False,
                False,
                True,
                True,
                False,
                False,
                False,
                False,
                True,
                False,
                False,
                False,
            ],
        ),
        Parameter("odom0_differential", True),
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
    ]
    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                namespace="subjugator_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[asdict(p) for p in parameters],
            ),
        ],
    )
