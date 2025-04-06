import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    vectornav_package = get_package_share_directory("vectornav")

    # Launch $(vectornav)/launch/vectornav.aunch.py
    vectornav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vectornav_package, "launch", "vectornav.launch.py"),
        ),
    )

    ld = LaunchDescription()
    ld.add_action(vectornav_launch)
    return ld
