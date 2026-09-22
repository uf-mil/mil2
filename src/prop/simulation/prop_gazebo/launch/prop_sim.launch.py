"""
The simulated boat, its sensors and localization, optionally under control.

    ros2 launch prop_gazebo prop_sim.launch.py
    ros2 launch prop_gazebo prop_sim.launch.py control:=true
    ros2 launch prop_gazebo prop_sim.launch.py control:=true mission:=square
    ros2 launch prop_gazebo prop_sim.launch.py \
        world_pkg:=navigator_gazebo world:=robotx_2024.world
    ros2 launch prop_gazebo prop_sim.launch.py rviz:=false \
        gz_args:="-s --headless-rendering"

Gazebo supplies the hull and the sensors and nothing else: localization,
perception, planning and control are the same nodes the boat runs, and all
that differs is what is publishing /imu, /gps_raw and /lidar/points, and what
is listening on /thrusters/left and /thrusters/right.

Perception and the planner come up with the rest, and with control:=true
guidance follows whatever the planner publishes. Send it somewhere to go with
RViz's "2D Goal Pose". A mission file drives a fixed route instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def generate_launch_description():
    # Any package that installs worlds/ will do, so the boat can be dropped
    # into the RobotX course as easily as into its own lake.
    world = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("world_pkg")),
            "worlds",
            LaunchConfiguration("world"),
        ],
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
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            # About where the hull settles, so it does not drop in.
            "-z",
            "0.06",
            "-Y",
            LaunchConfiguration("yaw"),
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
                "world_pkg",
                default_value="prop_gazebo",
                description="Package the world lives in, under its worlds/ "
                "directory. navigator_gazebo for robotx_2024.world.",
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument(
                "yaw",
                default_value="0.0",
                description="Spawn heading in radians, ENU.",
            ),
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
            DeclareLaunchArgument(
                "mission",
                default_value="none",
                description="Mission file for the controller to start on. "
                "The default follows the planner instead; a plan can also be "
                "sent by hand with 'ros2 run prop_controller plan.py'.",
            ),
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
            # Both are written for the boat by default; here the cloud comes off
            # the bridge and everything is on the simulator's clock.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    pkg_share("pcd", "launch", "filter.launch.py"),
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "input_topic": "/lidar/points",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    pkg_share("prop_planner", "launch", "planner.launch.py"),
                ),
                launch_arguments={"use_sim_time": "true"}.items(),
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
