import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    board_package = get_package_share_directory("thrust_and_kill_board")

    # Launch $(thrust_and_kill_board)/launch/dvl.aunch.py
    board_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(board_package, "launch", "thrust_and_kill_board.launch.py"),
        ),
    )

    ld = LaunchDescription()
    ld.add_action(board_launch)
    return ld
