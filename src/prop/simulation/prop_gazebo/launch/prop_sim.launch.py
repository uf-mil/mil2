"""
The simulated boat, its sensors and localization, optionally under control.

    ros2 launch prop_gazebo prop_sim.launch.py
    ros2 launch prop_gazebo prop_sim.launch.py control:=true mission:=square
    ros2 launch prop_gazebo prop_sim.launch.py rviz:=false \
        gz_args:="-s --headless-rendering"

Gazebo supplies the hull and the sensors and nothing else: localization and
control are the same nodes the boat runs, and all that differs is what is
publishing /imu, /gps_raw and /lidar/scan, and what is listening on
/thrusters/left and /thrusters/right.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def generate_launch_description():
    world = PathJoinSubstitution(
        [pkg_share("prop_gazebo", "worlds"), LaunchConfiguration("world")],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_share("ros_gz_sim", "launch", "gz_sim.launch.py"),
        ),
        launch_arguments={
            "gz_args": [world, " -r ", LaunchConfiguration("gz_args")],
        }.items(),
    )

    spawn_prop = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "prop",
            "-file",
            pkg_share("prop_gazebo", "models", "prop", "model.sdf"),
            "-x",
            "0.0",
            "-y",
            "0.0",
            # About where the hull settles, so it does not drop in.
            "-z",
            "0.06",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"config_file": pkg_share("prop_gazebo", "config", "prop_bridge.yaml")},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="prop_lake.world"),
            DeclareLaunchArgument(
                "gz_args",
                default_value="--render-engine ogre2",
                description="Extra gz sim flags. Pass '-s --headless-rendering' "
                "to run without the Gazebo window.",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "control",
                default_value="false",
                description="Run the mission, guidance and thruster manager.",
            ),
            DeclareLaunchArgument("mission", default_value="square"),
            DeclareLaunchArgument(
                "control_delay",
                default_value="15.0",
                description="Seconds to wait for localization before the "
                "controller starts.",
            ),
            SetParameter("use_sim_time", True),
            gz_sim,
            spawn_prop,
            bridge,
            # Stands in for the thruster driver, and for the covariances the
            # bridge cannot carry.
            Node(
                package="prop_gazebo",
                executable="sim_thrusters.py",
                name="sim_thrusters",
                output="screen",
            ),
            Node(
                package="prop_gazebo",
                executable="sim_sensors.py",
                name="sim_sensors",
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    pkg_share("prop_localization", "launch", "localization.launch.py"),
                ),
            ),
            # navsat_transform takes its datum from the first fix, so the map
            # frame ends up wherever the boat was when localization came up.
            # Hold the controller until then, or the boat drives off first and
            # the whole mission is offset by however far it got.
            TimerAction(
                period=LaunchConfiguration("control_delay"),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            pkg_share(
                                "prop_controller",
                                "launch",
                                "controller.launch.py",
                            ),
                        ),
                        launch_arguments={
                            "mission": LaunchConfiguration("mission"),
                        }.items(),
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("control")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", pkg_share("prop_gazebo", "config", "prop_sim.rviz")],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ],
    )
