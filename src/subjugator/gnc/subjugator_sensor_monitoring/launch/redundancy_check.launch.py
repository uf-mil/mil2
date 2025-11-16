"""Launch file for redundancy check node."""

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for redundancy check."""
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("subjugator_localization"),
                        "launch",
                        "subjugator_localization.launch.py",
                    ],
                ),
            ],
        ),
    )
    # Service call to enable localization
    enable_localization = TimerAction(
        period=5.0,  # Wait 2 seconds
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/subjugator_localization/enable",
                    "std_srvs/srv/Empty",
                    "{}",
                ],
                output="screen",
            ),
        ],
    )
    return LaunchDescription(
        [
            Node(
                package="subjugator_sensor_monitoring",
                executable="redundancy_check.py",
                name="redundancy_check_node",
                output="screen",
            ),
            localization_launch,
            enable_localization,
        ],
    )
