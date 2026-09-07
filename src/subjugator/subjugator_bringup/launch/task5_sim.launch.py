"""Task 5 sim testing bring-up — a superset of ``gazebo.launch.py``.

One command that gets you from nothing to a Task 5 down-cam stack in Gazebo:

    ros2 launch subjugator_bringup task5_sim.launch.py

That does everything ``gazebo.launch.py`` does (sim + world + Sub9 + bridge +
pinger heading) AND starts the down-cam YOLO node the missions read from
(``octagon_sim.pt`` on ``/down_cam/image_raw`` under namespace ``yolo_down``).
After it settles you drive stages from another terminal with ``pooltest.sh``,
exactly as in ``pool_tests/SIM_TESTING.md``.

Optionally it also kicks off a mission autonomously once the sim is up, so a
single command brings up sim + YOLO and starts a stage:

    ros2 launch subjugator_bringup task5_sim.launch.py stage:=combined role:=survey_repair
    ros2 launch subjugator_bringup task5_sim.launch.py stage:=full role:=survey_repair score_level:=3

``stage:=`` accepts the friendly pooltest names below (or a raw mission name):

    calib     -> CenterCameraTest      hone   -> HoneOverTableOnly
    select    -> SelectOnly            combined -> HoneOverTableSelect   (S2+S3)
    grasp     -> OctagonGraspMission   place  -> OctagonPlaceMission
    loop      -> OctagonLoopMission    full   -> OctagonMission          (S1..S7)
    full_s2   -> OctagonTableMission

Why not ``preflight``/``deadreckon`` here? Those are interactive/probe-only
stages that live in ``pooltest.sh`` (they need a typed ``GO`` / manual teleop,
which cannot run inside ``ros2 launch``). Use ``pooltest.sh`` for those and for
any run where you want the GO-gate, bag recording, and a results form.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def pkg_share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


# Friendly pooltest stage name -> mission the mission_planner_node should run.
# Only autonomous missions are listed (no GO-gate); preflight/deadreckon stay in
# pooltest.sh. Raw mission names are also accepted and passed through unchanged.
STAGE_TO_MISSION = {
    "calib": "CenterCameraTest",
    "hone": "HoneOverTableOnly",
    "select": "SelectOnly",
    "combined": "HoneOverTableSelect",
    "grasp": "OctagonGraspMission",
    "place": "OctagonPlaceMission",
    "loop": "OctagonLoopMission",
    "full_s2": "OctagonTableMission",
    "full": "OctagonMission",
}

# Stages that are inherently interactive / probe-only — not launchable here.
POOLTEST_ONLY = {"preflight", "deadreckon"}


def _maybe_mission(context, *args, **kwargs):
    """Build the optional autonomous mission node from the resolved ``stage``."""
    stage = LaunchConfiguration("stage").perform(context).strip()
    if not stage:
        return []  # bring-up only

    if stage in POOLTEST_ONLY:
        return [
            LogInfo(
                msg=(
                    f"[task5_sim] stage '{stage}' is interactive/probe-only — run it "
                    f"with pool_tests/pooltest.sh {stage} --sim (not via this launch)."
                ),
            ),
        ]

    mission = STAGE_TO_MISSION.get(stage, stage)  # accept raw mission names too
    role = LaunchConfiguration("role").perform(context).strip()
    down_topic = LaunchConfiguration("down_image_topic").perform(context)
    delay = float(LaunchConfiguration("mission_delay").perform(context))

    params = {
        "mission": mission,
        "role": role,
        "down_image_topic": down_topic,
        "use_sim_time": True,
    }

    # Capstone dials only apply to the full S1..S7 mission.
    if mission == "OctagonMission":
        score_level = LaunchConfiguration("score_level").perform(context).strip()
        if score_level:
            params["score_level"] = int(score_level)
        no_pinger = LaunchConfiguration("no_pinger").perform(context).strip().lower()
        params["do_pinger"] = 0 if no_pinger in ("1", "true", "yes") else 1

    mission_node = Node(
        package="mission_planner",
        executable="mission_planner_node",
        name="mission_planner_node",
        output="screen",
        parameters=[params],
    )

    return [
        LogInfo(
            msg=(
                f"[task5_sim] sim + yolo_down up; starting mission '{mission}' "
                f"(role={role}) in {delay:.0f}s — Ctrl-C tears everything down."
            ),
        ),
        # Give gz spawn + the bridge + YOLO a moment before the mission subscribes.
        # (The mission also waits internally for detections/odom, so this is just
        # a head-start, not a hard readiness gate.)
        TimerAction(period=delay, actions=[mission_node]),
    ]


def generate_launch_description():
    # ---- args ---------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="robosub_2025.world",
        description="Gazebo world file (forwarded to gazebo.launch.py).",
    )
    device_arg = DeclareLaunchArgument(
        "device",
        default_value="cpu",
        description="YOLO device: 'cpu' (default) or '0' for a CUDA GPU.",
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=pkg_share("subjugator_vision", "models", "octagon_sim.pt"),
        description="Down-cam YOLO model (the Task 5 octagon model).",
    )
    down_topic_arg = DeclareLaunchArgument(
        "down_image_topic",
        default_value="/down_cam/image_raw",
        description="Image topic the down-cam YOLO node consumes.",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="yolo_down",
        description="Namespace for the down-cam YOLO node (-> /<ns>/detections).",
    )
    stage_arg = DeclareLaunchArgument(
        "stage",
        default_value="",
        description=(
            "Optional: pooltest stage name (calib/hone/select/combined/grasp/"
            "place/loop/full_s2/full) or a raw mission name to run autonomously "
            "after bring-up. Empty (default) = bring-up only."
        ),
    )
    role_arg = DeclareLaunchArgument(
        "role",
        default_value="survey_repair",
        description="survey_repair (nut_cylinder,electric_box) | search_rescue "
        "(pill_cylinder,bandaid_box). Used only when 'stage' is set.",
    )
    score_level_arg = DeclareLaunchArgument(
        "score_level",
        default_value="",
        description="Only for stage:=full — how far up the ladder to run "
        "(0..6; empty = mission default 6).",
    )
    no_pinger_arg = DeclareLaunchArgument(
        "no_pinger",
        default_value="true",
        description="Only for stage:=full — skip S1 acoustic homing (sim usually "
        "has no pinger; sub is hand-placed over the octagon).",
    )
    mission_delay_arg = DeclareLaunchArgument(
        "mission_delay",
        default_value="12.0",
        description="Seconds to wait after bring-up before starting the optional "
        "mission (head-start for gz spawn + bridge + YOLO).",
    )

    # ---- bring-up: everything gazebo.launch.py does, plus down-cam YOLO ------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_share("subjugator_bringup", "launch", "gazebo.launch.py"),
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    yolo_down = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_share("yolo_bringup", "launch", "yolov11_sim.launch.py"),
        ),
        launch_arguments={
            "model": LaunchConfiguration("model"),
            "input_image_topic": LaunchConfiguration("down_image_topic"),
            "namespace": LaunchConfiguration("namespace"),
            "device": LaunchConfiguration("device"),
        }.items(),
    )

    return LaunchDescription(
        [
            world_arg,
            device_arg,
            model_arg,
            down_topic_arg,
            namespace_arg,
            stage_arg,
            role_arg,
            score_level_arg,
            no_pinger_arg,
            mission_delay_arg,
            LogInfo(
                msg="[task5_sim] bringing up Gazebo + down-cam YOLO "
                "(octagon_sim.pt -> /yolo_down/detections)...",
            ),
            gazebo,
            yolo_down,
            OpaqueFunction(function=_maybe_mission),
        ],
    )
