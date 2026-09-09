"""
Localization stack: robot_state_publisher + dual EKF + navsat_transform.

  ekf_local  : IMU only        -> odometry/filtered/local  + TF odom -> base_link
  ekf_global : IMU + GPS odom  -> odometry/filtered/global + TF map  -> odom
  navsat     : GPS + IMU + global odom <-> odometry/gps

There is no wheel odometry on a boat, so the local EKF is dead reckoning off the
IMU alone and only the global one is worth steering by.

This launch starts no sensors. It works against the boat or against Gazebo,
whichever is publishing /imu (sensor_msgs/Imu) and /gps_raw (NavSatFix).

    ros2 launch prop_localization localization.launch.py
    ros2 topic echo /odometry/filtered/global
    ros2 run tf2_ros tf2_echo map odom
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def generate_launch_description():
    config = pkg_share("prop_localization", "config", "ekf.yaml")

    with open(pkg_share("prop_localization", "urdf", "prop.urdf")) as urdf:
        robot_description = urdf.read()

    return LaunchDescription(
        [
            # base_link -> imu and base_link -> lidar_link, out of prop.urdf.
            # The EKFs need the first one to rotate the IMU out of its 180
            # degree mounting.
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_local",
                parameters=[config],
                remappings=[("odometry/filtered", "/odometry/filtered/local")],
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_global",
                parameters=[config],
                remappings=[("odometry/filtered", "/odometry/filtered/global")],
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                parameters=[config],
                remappings=[
                    ("gps/fix", "/gps_raw"),
                    ("imu", "/imu"),
                    ("odometry/filtered", "/odometry/filtered/global"),
                    ("odometry/gps", "/odometry/gps"),
                ],
                output="screen",
            ),
        ],
    )
