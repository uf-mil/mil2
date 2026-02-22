from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("subjugator_bringup"),
                            "launch",
                            "gazebo.launch.py",
                        ],
                    ),
                ),
            ),
            Node(
                package="mil_robogym",
                executable="mil_robogym",
                name="mil_robogym",
                output="screen",
            ),
            Node(
                package="mil_robogym",
                executable="gz_pose_srv",
                name="gz_pose_srv",
                output="screen",
            ),
        ],
    )
