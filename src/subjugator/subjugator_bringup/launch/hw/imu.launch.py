import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    vectornav_package = get_package_share_directory("vectornav")
    bringup_package = get_package_share_directory("subjugator_bringup")
    magnetic_compensation_package = get_package_share_directory("magnetic_compensation")

    # Launch $(vectornav)/launch/vectornav.aunch.py
    vectornav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vectornav_package, "launch", "vectornav.launch.py"),
        ),
    )

    # Launch $(magnetic_compensation)/launch/mag_comp.launch.py
    # with param config_file
    mag_comp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(magnetic_compensation_package, "launch", "mag_comp.launch.py"),
        ),
        launch_arguments={
            "config_file": os.path.join(
                bringup_package,
                "config",
                "sensors",
                "hardsoft.yaml",
            ),
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(vectornav_launch)
    ld.add_action(mag_comp_launch)
    return ld
