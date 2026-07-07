import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def generate_launch_description():
    dvl_launch = IncludeLaunchDescription(
        pkg_share("waterlinked_dvl_driver", "launch", "dvl.launch.py"),
    )

    front_cam_launch = IncludeLaunchDescription(
        pkg_share("front_cam", "launch", "front_cam.launch.py"),
    )

    # IMU
    # vectornav_launch = IncludeLaunchDescription(
    #     pkg_share("vectornav", "launch", "vectornav.launch.py"),
    # )
    # mag_comp_launch = IncludeLaunchDescription(
    #     pkg_share("magnetic_compensation", "launch", "mag_comp.launch.py"),
    #     launch_arguments={
    #         "config_file": pkg_share(
    #             "subjugator_bringup",
    #             "config",
    #             "sensors",
    #             "hardsoft.yaml",
    #         ),
    #     }.items(),
    # )

    thrust_launch = IncludeLaunchDescription(
        pkg_share("thrust_and_kill_board", "launch", "thrust_and_kill_board.launch.py"),
    )

    new_depth = Node(
        package="new_depth_driver",
        executable="new_depth_driver",
        name="new_depth_driver",
        output="both",
    )

    # Launch subjugator_setup.launch.py
    subjugator_setup = IncludeLaunchDescription(
        pkg_share("subjugator_bringup", "launch", "common.launch.py"),
        launch_arguments={
            "xacro_file": pkg_share(
                "subjugator_description",
                "urdf",
                "sub9.urdf.xacro",
            ),
            "gui": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            new_depth,
            dvl_launch,
            front_cam_launch,
            # vectornav_launch,
            # mag_comp_launch,
            thrust_launch,
            subjugator_setup,
        ],
    )
