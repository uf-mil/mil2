"""
The VLP-16 on the mast: driver, point cloud, and the flattened /scan rf2o reads.

    ros2 launch prop_bringup lidar.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    calibration = os.path.join(
        get_package_share_directory("velodyne_pointcloud"),
        "params",
        "VLP16db.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="velodyne_driver",
                executable="velodyne_driver_node",
                parameters=[{"device_ip": "192.168.37.51", "model": "VLP16"}],
                output="screen",
            ),
            Node(
                package="velodyne_pointcloud",
                executable="velodyne_transform_node",
                parameters=[{"model": "VLP16", "calibration": calibration}],
                output="screen",
            ),
            Node(
                package="lidar_mapper",
                executable="lidar_mapper",
                output="screen",
            ),
        ],
    )
