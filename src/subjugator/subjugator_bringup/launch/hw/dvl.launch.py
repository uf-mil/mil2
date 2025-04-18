import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    dvl_package = get_package_share_directory("waterlinked_dvl_driver")

    # Launch $(waterlinked_dvl_driver)/launch/dvl.aunch.py
    dvl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dvl_package, "launch", "dvl.launch.py"),
        ),
    )

    ld = LaunchDescription()
    ld.add_action(dvl_launch)
    return ld
