"""Randomized dock fixture. Only the independent evaluator receives its layout."""

import math
import os
import random
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_trial(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory("prop_gazebo")
    controller = os.path.join(
        get_package_share_directory("prop_controller"),
        "config",
        "controller.yaml",
    )
    rng = random.Random(int(value("seed")))
    dx, dy, yaw = rng.uniform(0, 3), rng.uniform(-1.5, 1.5), rng.uniform(-0.15, 0.15)
    colors = ["red", "red", "green"]
    rng.shuffle(colors)
    tree = ET.parse(os.path.join(share, "worlds", "prop_docking.world"))
    world = tree.getroot().find("world")
    world.find("physics/real_time_factor").text = value("real_time_factor")
    color_rgb = {"red": "1 0 0 1", "green": "0 1 0 1"}
    for model in world.findall("model"):
        if model.get("name") == "water":
            continue
        original = model.find("pose").text.split()
        x, y, z = map(float, original[:3])
        if model.get("name").startswith("indicator_"):
            index = {3.0: 0, 0.0: 1, -3.0: 2}[y]
            color = colors[index]
            model.set("name", "beacon_" + str(index))
            z = 0.65
            for size in model.findall(".//box/size"):
                size.text = "0.2 0.5 0.7"
            for material in model.findall(".//material"):
                for tag in ("ambient", "diffuse"):
                    material.find(tag).text = (
                        "0.4 0.4 0.4 1"
                        if color == value("hide_color")
                        else color_rgb[color]
                    )
        wx = dx + math.cos(yaw) * x - math.sin(yaw) * y
        wy = dy + math.sin(yaw) * x + math.cos(yaw) * y
        model.find("pose").text = f"{wx} {wy} {z} 0 0 {yaw}"
    # Unique world per launch; geometry metadata never goes to perception.
    fd, world_path = tempfile.mkstemp(prefix="mil2_dock_", suffix=".world")
    with os.fdopen(fd, "w") as output:
        tree.write(output, encoding="unicode")
    evaluator = Node(
        package="prop_gazebo",
        executable="docking_evaluator.py",
        parameters=[
            {
                "use_sim_time": True,
                "dock_x": dx,
                "dock_y": dy,
                "dock_yaw": yaw,
                "colors": colors,
                "timeout": float(value("timeout")) + float(value("ocs_start_delay")) + 25.0,
                "seed": int(value("seed")),
            },
        ],
        output="screen",
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", "prop_sim.launch.py"),
            ),
            launch_arguments={
                "world_path": world_path,
                "control": "false",
                "rviz": value("rviz"),
                "gz_args": value("gz_args"),
            }.items(),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=evaluator,
                on_exit=[
                    EmitEvent(event=Shutdown(reason="Docking evaluation finished")),
                ],
            ),
            condition=IfCondition(LaunchConfiguration("exit_on_result")),
        ),
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package="prop_controller",
                    executable="guidance",
                    name="guidance",
                    parameters=[
                        controller,
                        {
                            "use_sim_time": True,
                            "speed": 0.3,
                            "lookahead": 0.7,
                            "accept_radius": 0.2,
                            "approach_gain": 0.25,
                        },
                    ],
                    output="screen",
                ),
                Node(
                    package="prop_controller",
                    executable="thruster_manager",
                    name="thruster_manager",
                    parameters=[controller, {"use_sim_time": True}],
                    output="screen",
                ),
                Node(
                    package="prop_missions",
                    executable="docking_mission",
                    parameters=[
                        {
                            "use_sim_time": True,
                            "timeout": float(value("timeout")),
                            "sensor_timeout": 1.5,
                        },
                    ],
                    output="screen",
                ),
                Node(
                    package="prop_missions",
                    executable="mock_ocs",
                    condition=IfCondition(LaunchConfiguration("mock_ocs")),
                    parameters=[{"use_sim_time": True,
                                 "start_delay": float(value("ocs_start_delay")),
                                 "drop_receipts": int(value("ocs_drop_receipts"))}],
                    output="screen",
                ),
                evaluator,
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "seed",
                default_value="1",
                description="Repeatable random dock translation, yaw and color order",
            ),
            DeclareLaunchArgument(
                "hide_color",
                default_value="none",
                choices=["none", "red", "green"],
            ),
            DeclareLaunchArgument("mock_ocs", default_value="true"),
            DeclareLaunchArgument("ocs_start_delay", default_value="3.0"),
            DeclareLaunchArgument("ocs_drop_receipts", default_value="0"),
            DeclareLaunchArgument("timeout", default_value="240.0"),
            DeclareLaunchArgument("real_time_factor", default_value="0.5"),
            DeclareLaunchArgument("exit_on_result", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("gz_args", default_value="--render-engine ogre2"),
            OpaqueFunction(function=launch_trial),
        ],
    )
