from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Args
    config_file_cmd = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("thrust_and_kill_board"),
                "config",
                "config.yaml",
            ],
        ),
        description="Path to the config file",
    )
    driver_cmd = Node(
        package="thrust_and_kill_board",
        executable="driver.py",
        name="thrust_and_kill_board",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )
    # Launch description
    ld = LaunchDescription()
    ld.add_action(config_file_cmd)
    ld.add_action(driver_cmd)
    return ld
