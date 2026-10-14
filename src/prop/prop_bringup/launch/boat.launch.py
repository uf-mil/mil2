"""
The boat: sensors, localization, perception and control.

    ros2 launch prop_bringup boat.launch.py
    ros2 launch prop_bringup boat.launch.py sensors:=false

Localization and the controller come up disabled, so the map origin is not
fixed and nothing drives until you say so:

    prop-start-localization  once the boat sits where the origin should be
    start-controller         once the estimate has settled; holds position
    stop-controller          zero wrench, controller out of the loop
    reset-controller         zero wrench, reload gains, hold where it is now
    start-planning           clear the obstacle map and start routing

Started here:

    prop3              the lidar and the Infix, unless sensors:=false
    prop_localization  rf2o, map -> odom -> base_link, odometry/filtered/global
    pcd                filter, cluster and track the lidar cloud
    prop_planner       remember what was seen, and route around it
    pid_controller     subjugator_controller_again, goal/trajectory -> cmd_wrench

The thruster driver is not: it runs on the Pi, against the Navigator board.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def include(pkg, launch_file, condition=None, **launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_share(pkg, "launch", launch_file)),
        launch_arguments=launch_arguments.items(),
        condition=condition,
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensors",
                default_value="true",
                description="Run the lidar and Infix drivers here.",
            ),
            DeclareLaunchArgument(
                "lidar_topic",
                default_value="/velodyne_points",
                description="Raw PointCloud2 from the lidar driver.",
            ),
            include(
                "prop_bringup",
                "prop3.launch.py",
                IfCondition(LaunchConfiguration("sensors")),
            ),
            # include("prop_thrusters", "thrusters.launch.py"),
            include("prop_localization", "localization.launch.py"),
            # Both default to the boat already; the topic is passed through so
            # a driver on another name is a launch argument away.
            include(
                "pcd",
                "filter.launch.py",
                input_topic=LaunchConfiguration("lidar_topic"),
            ),
            include(
                "prop_planner",
                "planner.launch.py",
                frame="odom",
                odometry="/odometry/filtered/local",
            ),
            include("subjugator_controller_again", "pid_controller.launch.py"),
        ],
    )
