import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("subjugator_controller")
    default_param = os.path.join(pkg, "config", "pid_controller.yaml")

    # Node launch argument
    params_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value=default_param,
    )

    node = Node(
        package="subjugator_controller",
        executable="subjugator_controller",
        name="pid_controller",
        parameters=[LaunchConfiguration("param_file")],
        output="screen",
    )

    return LaunchDescription([params_file_arg, node])
