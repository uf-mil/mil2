from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ① declare a launch-argument (default just the file-name)
    declare_mission = DeclareLaunchArgument(
        "mission_file", default_value="prequal.yaml",
        description="YAML file located in subjugator_mission_planner/missions")

    # ② make its value available
    mission = LaunchConfiguration("mission_file")

    # ③ feed that value into the Node’s parameter dict
    planner = Node(
        package="subjugator_mission_planner",
        executable="mission_planner",
        name="mission_planner",
        parameters=[{ "mission_file": mission }],
    )

    return LaunchDescription([declare_mission, planner])
