import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="robotx_2024.world",
        description="World file to load in Gazebo.",
    )

    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="lidar_platform",
        description="Name of the spawned lidar platform model.",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_share("ros_gz_sim", "launch", "gz_sim.launch.py"),
        ),
        launch_arguments={
            "gz_args": [
                PathJoinSubstitution(
                    [
                        pkg_share("navigator_gazebo"),
                        "worlds",
                        LaunchConfiguration("world"),
                    ],
                ),
                " --render-engine",
                " ogre2",
            ],
        }.items(),
    )

    spawn_lidar_platform = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            LaunchConfiguration("model_name"),
            "-file",
            pkg_share("prop_gazebo", "models", "lidar_platform", "model.sdf"),
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.1",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": pkg_share("prop_gazebo", "config", "lidar_bridge.yaml"),
            },
        ],
        output="screen",
    )

    base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0.15",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "lidar_link",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            world_arg,
            model_name_arg,
            SetParameter("use_sim_time", True),
            gz_sim,
            spawn_lidar_platform,
            bridge,
            base_tf,
        ],
    )
