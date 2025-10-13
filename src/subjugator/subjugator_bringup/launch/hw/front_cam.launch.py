import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    front_cam_package = get_package_share_directory("front_cam")

    front_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(front_cam_package, "launch", "front_cam.launch.py"),
        ),
    )

    ld = LaunchDescription()
    ld.add_action(front_cam_launch)
    return ld
