import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    lidar_launch = os.path.join(
        get_package_share_directory("prop_bringup"),
        "launch",
        "lidar.launch.py",
    )

    infix_sensors_launch = os.path.join(
        get_package_share_directory("prop_bringup"),
        "launch",
        "infix_sensors.launch.py",
    )

    acoustic_modem = Node(
        package="mil_acoustic_modem",
        executable="mil_acoustic_modem",
        output="both",
        remappings=[("/heartbeat_received", "/heartbeats")],
    )

    return LaunchDescription(
        [
            # start THE lidar lol
            IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch)),
            # start infix and rip it's sensor readings
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(infix_sensors_launch),
            ),
            acoustic_modem,
        ],
    )
