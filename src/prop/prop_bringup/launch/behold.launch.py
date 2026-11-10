"""
The whole boat for a run: boat.launch.py, the cameras, fusion and the missions.

    ros2 launch prop_bringup behold.launch.py
    ros2 launch prop_bringup behold.launch.py model:=<weights.pt>
    ros2 launch prop_bringup behold.launch.py perception:=false missions:=false

Started here:

    boat.launch.py         sensors, localization, planner, controller, heartbeat
    cameras.launch.py      camera_node, vision_node and beacon_node, with
                           publish_image on, since docking needs image_rect
    fusion.launch.py       classified_markers and labelled_objects
    safe_passage           waits for RunStart from the OCS
    docking_mission        waits for the OCS's START
    boat_timeout           Task 4: goes where an AssistanceRequest says, and
                           the missions stand aside until it is back
    survey                 only with survey:=true; waits for /survey/start

Nothing moves until it is told to. Afterwards, as with boat.launch.py:

    prop-start-localization, start-controller, prop-auto

Ctrl-C stops all of it, localization too, so the map origin is lost. To restart
the cameras alone, launch with perception:=false and run cameras.launch.py and
fusion.launch.py in their own terminals.

Any argument of the included files can be passed here too, e.g.
front_device:=..., planner:=visibility or debug_image:=true.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(pkg, launch_file, switch=None):
    return GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(pkg),
                        "launch",
                        launch_file,
                    ),
                ),
            ),
        ],
        condition=IfCondition(LaunchConfiguration(switch)) if switch else None,
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "perception",
                default_value="true",
                description="Start the cameras and fusion.",
            ),
            DeclareLaunchArgument(
                "missions",
                default_value="true",
                description="Start safe_passage, docking_mission and boat_timeout.",
            ),
            DeclareLaunchArgument(
                "survey",
                default_value="false",
                description="Start survey too.",
            ),
            DeclareLaunchArgument(
                "model",
                default_value="~/buoy_buoy10-13.pt",
                description="YOLO weights for the cameras.",
            ),
            DeclareLaunchArgument(
                "publish_image",
                default_value="true",
                description="image_rect from the cameras. docking_mission needs it.",
            ),
            include("prop_bringup", "boat.launch.py"),
            include("prop_vision", "cameras.launch.py", "perception"),
            include("prop_fusion", "fusion.launch.py", "perception"),
            include("prop_missions", "safe_passage.launch.py", "missions"),
            include("prop_missions", "docking.launch.py", "missions"),
            Node(
                package="boat_timeout",
                executable="boat_timeout",
                output="screen",
                condition=IfCondition(LaunchConfiguration("missions")),
            ),
            include("prop_missions", "survey.launch.py", "survey"),
        ],
    )
