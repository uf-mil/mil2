from dataclasses import dataclass

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
    parameters: list[Parameter] = [
        Parameter("imu0", "/imu/data_raw"),
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
        Parameter("odom0", "/dvl/velocity"),
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
            Node(
                package="robot_localization",
                namespace="subjugator_localization",
                executable="ukf_node",
                name="ukf_filter_node",
                parameters=[p.dict() for p in parameters],
            ),
        ],
    )
