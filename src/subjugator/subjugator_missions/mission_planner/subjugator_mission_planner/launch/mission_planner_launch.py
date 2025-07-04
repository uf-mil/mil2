from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="subjugator_mission_planner",
                executable="mission_planner",
                name="mission_planner",
                parameters=[
                    {
                        "mission_file": "subjugator_mission_planner/missions/prequal.yaml",
                    },
                ],
            ),
        ],
    )
