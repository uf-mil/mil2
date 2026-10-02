import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    rf2o_launch = os.path.join(
        get_package_share_directory("rf2o_laser_odometry"),
        "launch",
        "rf2o_laser_odometry.launch.py",
    )

    lidar_hw_launch = os.path.join(
        os.path.expanduser("~/mil2/src/prop/simulation/prop_gazebo/launch"),
        "lidar_hw.launch.py",
    )

    return LaunchDescription(
        [
            # Start lidar_mapper
            Node(
                package="lidar_mapper",
                executable="lidar_mapper",
                name="lidar_mapper",
                output="screen",
            ),
            # start THE lidar lol
            IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_hw_launch)),
            # Start RF2O laser odometry
            IncludeLaunchDescription(PythonLaunchDescriptionSource(rf2o_launch)),
        ],
    )
