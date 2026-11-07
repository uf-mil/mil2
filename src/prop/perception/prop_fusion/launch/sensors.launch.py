"""
Place the lidar and cameras on the hull, from config/sensors.yaml.

    ros2 launch prop_fusion sensors.launch.py
    ros2 launch prop_fusion sensors.launch.py sensors_file:=/path/to/other.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from prop_fusion.mounts import default_sensors_file


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensors_file",
                default_value=default_sensors_file(),
                description="Where each sensor is mounted.",
            ),
            Node(
                package="prop_fusion",
                executable="sensor_tf",
                name="sensor_tf",
                parameters=[{"sensors_file": LaunchConfiguration("sensors_file")}],
                output="screen",
            ),
        ],
    )
