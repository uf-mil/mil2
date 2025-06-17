import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory("subjugator_bringup")
    pkg_project_gazebo = get_package_share_directory("subjugator_gazebo")
    # pkg_project_description = get_package_share_directory("subjugator_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Setup to launch the simulator and Gazebo world
    gz_sim_world = DeclareLaunchArgument("world", default_value="robosub_2024.world")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py"),
        ),
        launch_arguments={
            "gz_args": [
                PathJoinSubstitution(
                    [pkg_project_gazebo, "worlds", LaunchConfiguration("world")],
                ),
                " --render-engine",
                " ogre",
            ],
        }.items(),
    )

    # Include the Subjugator_Setup Launch file
    subjugator_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, "launch", "subjugator_setup.launch.py"),
        ),
        launch_arguments={}.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project_bringup,
                    "config",
                    "subjugator_bridge.yaml",
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            },
        ],
        output="screen",
    )

<<<<<<< online_bagger
    bagger = Node(
        package="online_bagger",
        executable="online_bagger_server_node",
        parameters=[
            {
                # add topics to bag here
                "topics": ["/imu/data"],
            },
=======
    return LaunchDescription(
        [
            gz_sim_world,
            gz_sim,
            subjugator_setup,
            bridge,
>>>>>>> main
        ],
        output="screen",
    )

    return LaunchDescription(
        [gz_sim, subjugator_setup, bridge, bagger],
    )
