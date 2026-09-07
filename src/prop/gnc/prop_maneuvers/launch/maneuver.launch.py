"""Everything needed to watch one maneuver run in the simulator.

    ros2 launch prop_maneuvers maneuver.launch.py maneuver:=face_object
    ros2 launch prop_maneuvers maneuver.launch.py maneuver:=circle_object
    ros2 launch prop_maneuvers maneuver.launch.py maneuver:=approach_object

Starts the boat simulation, the perception chain, and the chosen maneuver
program. guidance and thruster_manager are started directly here rather than
through prop_controller's controller.launch.py, because that also starts
Carlos's mission node, which immediately drives the square plan -- by the time
a maneuver locks on and calls release(), the boat may have driven far enough
along that square to lose the buoy. Skipping the mission node keeps the boat
parked at the origin until a maneuver takes over. The maneuver is delayed so
the position estimate has settled and guidance is up before anything commands
the motors.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    maneuvers_share = get_package_share_directory("prop_maneuvers")
    gazebo_share = get_package_share_directory("prop_gazebo")
    controller_share = get_package_share_directory("prop_controller")
    settings = os.path.join(maneuvers_share, "config", "maneuvers.yaml")
    controller_config = os.path.join(controller_share, "config", "controller.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("maneuver", default_value="face_object"),
            DeclareLaunchArgument(
                "control_delay",
                default_value="15.0",
                description="Seconds to wait for localization before guidance "
                "and thruster_manager start, matching prop_sim.launch.py.",
            ),
            DeclareLaunchArgument(
                "start_delay",
                default_value="25.0",
                description="Seconds before the maneuver starts, letting the EKFs settle.",
            ),
            DeclareLaunchArgument("use_front", default_value="true"),
            DeclareLaunchArgument("target_x", default_value="20.0"),
            DeclareLaunchArgument("target_y", default_value="-3.0"),
            DeclareLaunchArgument(
                "blind_spots",
                default_value="false",
                description="Fake the real boat's blind spots by masking the scan.",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value=PythonExpression(
                    [
                        '"/lidar/scan_masked" if "',
                        LaunchConfiguration("blind_spots"),
                        '" == "true" else "/lidar/scan"',
                    ],
                ),
                description="Topic the perception chain reads. Defaults to the masked "
                "scan when blind_spots is true, so blind_spots:=true alone is enough.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, "launch", "prop_sim.launch.py"),
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(maneuvers_share, "launch", "perception.launch.py"),
                ),
                launch_arguments={
                    "blind_spots": LaunchConfiguration("blind_spots"),
                    "scan_topic": LaunchConfiguration("scan_topic"),
                }.items(),
            ),
            TimerAction(
                period=LaunchConfiguration("control_delay"),
                actions=[
                    Node(
                        package="prop_controller",
                        executable="guidance",
                        name="guidance",
                        parameters=[controller_config],
                        output="screen",
                    ),
                    Node(
                        package="prop_controller",
                        executable="thruster_manager",
                        name="thruster_manager",
                        parameters=[controller_config],
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("start_delay"),
                actions=[
                    Node(
                        package="prop_maneuvers",
                        executable=LaunchConfiguration("maneuver"),
                        name="maneuver",
                        output="screen",
                        parameters=[
                            settings,
                            {"use_sim_time": True},
                            {"use_front": LaunchConfiguration("use_front")},
                            {"target_x": LaunchConfiguration("target_x")},
                            {"target_y": LaunchConfiguration("target_y")},
                        ],
                    ),
                ],
            ),
        ],
    )
