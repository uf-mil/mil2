from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launches yolo package and example shark detection node.
    """

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("yolo_bringup"),
                            "launch",
                            "yolov11_sim.launch.py",
                        ],
                    ),
                ),
            ),
            Node(
                package="mil_robogym",
                executable="shark_detect",
                name="shark_detect",
                output="screen",
            ),
        ],
    )
