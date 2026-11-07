"""
Replays a bag through perception, to watch offline what the boat would have seen.

    ros2 launch prop_bringup bag_replay.launch.py bag:=<bag folder> model:=<weights.pt>

Only raw sensor topics are played. Everything downstream of them is produced
fresh by the code in this workspace, on the bag's clock:

    ekf_local      /imu/data and /lidar/odom -> odom -> base_link
    pcd            /velodyne_points -> tracked_markers
    occupancy_grid /velodyne_points -> occupancy_grid
    vision_node    /front_cam/image_raw -> /front_camera/detections
    fusion         tracks and detections -> classified_markers
    object_map     classified_markers and the grid -> labelled_objects

Pass rviz:=false to run without a window.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

SENSOR_TOPICS = (
    "/tf_static",
    "/imu/data",
    "/lidar/odom",
    "/velodyne_points",
)
CAMERA = "front_camera"
STARTUP_TIME = 8.0  # s, for the nodes to come up before the bag plays


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def include(pkg, launch_file, **launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_share(pkg, "launch", launch_file)),
        launch_arguments=launch_arguments.items(),
    )


def replay_config():
    return pkg_share("prop_bringup", "config", "bag_replay.yaml")


def localization():
    return Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        parameters=[
            pkg_share("prop_localization", "config", "killme.yaml"),
            replay_config(),
        ],
        remappings=[("odometry/filtered", "/odometry/filtered/local")],
        output="log",
    )


def occupancy_grid():
    return Node(
        package="prop_planner",
        executable="occupancy_grid",
        name="occupancy_grid",
        parameters=[
            pkg_share("prop_planner", "config", "grid_planner.yaml"),
            replay_config(),
        ],
        remappings=[("cloud", "/velodyne_points")],
        output="screen",
    )


def viewer():
    return Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", pkg_share("prop_bringup", "rviz", "bag_replay.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="log",
    )


def vision(model, image_topic, deck_line):
    return Node(
        package="prop_vision",
        executable="vision_node.py",
        name="vision",
        namespace=CAMERA,
        parameters=[
            {
                "image_topic": image_topic,
                "model": model,
                "frame_id": f"{CAMERA}_optical_frame",
                "publish_image": True,
                "deck_line": deck_line,
            },
        ],
        additional_env={"PYTHONUNBUFFERED": "1"},
        output="screen",
    )


def replay(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    image_topic = argument("image_topic")
    sensors_file = argument("sensors_file")

    play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            os.path.expanduser(argument("bag")),
            "--clock",
            "100",
            "--rate",
            argument("rate"),
            "--start-offset",
            argument("start"),
            "--topics",
            *SENSOR_TOPICS,
            image_topic,
        ],
        output="screen",
    )
    return [
        include("prop_fusion", "sensors.launch.py", sensors_file=sensors_file),
        localization(),
        include("pcd", "filter.launch.py", lidar_tf="false"),
        occupancy_grid(),
        vision(
            os.path.expanduser(argument("model")),
            image_topic,
            float(argument("deck_line")),
        ),
        include(
            "prop_fusion",
            "fusion.launch.py",
            sensors_file=sensors_file,
            debug_image="true",
        ),
        viewer(),
        TimerAction(period=STARTUP_TIME, actions=[play]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", description="Bag folder to play."),
            DeclareLaunchArgument("model", description="YOLO weights."),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/front_cam/image_raw",
                description="Camera topic in the bag.",
            ),
            DeclareLaunchArgument(
                "sensors_file",
                default_value=pkg_share(
                    "prop_bringup",
                    "config",
                    "sensors_2026_10_18.yaml",
                ),
                description="Where the sensors were on the day of the bag.",
            ),
            DeclareLaunchArgument(
                "deck_line",
                default_value="0.78",
                description="Height in the image, as a fraction, where the boat's "
                "own deck starts for the camera in the bag.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Open RViz on the replay.",
            ),
            DeclareLaunchArgument(
                "rate",
                default_value="1.0",
                description="Playback speed.",
            ),
            DeclareLaunchArgument(
                "start",
                default_value="0.0",
                description="Seconds into the bag to start from.",
            ),
            SetParameter("use_sim_time", True),
            OpaqueFunction(function=replay),
        ],
    )
