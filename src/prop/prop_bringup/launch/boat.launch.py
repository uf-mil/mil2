"""
The boat: drivers, localization, perception, planning and control.

    ros2 launch prop_bringup boat.launch.py
    ros2 launch prop_bringup boat.launch.py mission:=square
    ros2 launch prop_bringup boat.launch.py control:=false

Everything here runs on the hull's own clock against its own sensors. The
simulator's counterpart is prop_gazebo/launch/prop_sim.launch.py, which starts
these same nodes against Gazebo; all that differs between the two is where
/imu, /gps_raw and the lidar cloud come from, and which clock they carry.

Started here:

    prop_thrusters     the driver, listening on /thrusters/left and /right
    prop_localization  map -> odom -> base_link, and odometry/filtered/global
    pcd                filter, cluster and track the lidar cloud
    prop_planner       remember what was seen, and route around it
    prop_controller    guidance and the thruster manager, after a delay

Not started here, because their drivers live outside this tree: whatever
publishes /imu and /gps_raw, and the Velodyne publishing /velodyne_points.

The planner is the normal source of a plan on the boat, so control comes up
with no mission and waits for one. It publishes nothing until it is given a
goal on goal_pose, and the thruster manager drops thrust a second after the
last command, so bringing control up idle moves nothing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def include(pkg, launch_file, **launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_share(pkg, "launch", launch_file)),
        launch_arguments=launch_arguments.items(),
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_topic",
                default_value="/velodyne_points",
                description="Raw PointCloud2 from the lidar driver.",
            ),
            DeclareLaunchArgument(
                "control",
                default_value="true",
                description="Run guidance and the thruster manager.",
            ),
            DeclareLaunchArgument(
                "mission",
                default_value="none",
                description="Mission file for the controller to start on, or "
                "none to follow the planner instead.",
            ),
            DeclareLaunchArgument(
                "control_delay",
                default_value="15.0",
                description="Seconds to wait for localization before the "
                "controller starts.",
            ),
            include("prop_thrusters", "thrusters.launch.py"),
            include("prop_localization", "localization.launch.py"),
            # Both default to the boat already; the topic is passed through so
            # a driver on another name is a launch argument away.
            include(
                "pcd",
                "filter.launch.py",
                input_topic=LaunchConfiguration("lidar_topic"),
            ),
            include("prop_planner", "planner.launch.py"),
            # navsat_transform takes its datum from the first fix, so the map
            # frame ends up wherever the boat was when localization came up.
            # Hold the controller until then, or the boat drives off first and
            # everything is offset by however far it got.
            TimerAction(
                period=LaunchConfiguration("control_delay"),
                actions=[
                    include(
                        "prop_controller",
                        "controller.launch.py",
                        mission=LaunchConfiguration("mission"),
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("control")),
            ),
        ],
    )
