import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("yolo_bringup"),
                        "launch",
                        "yolo.launch.py",
                    )
                ),
                launch_arguments={
                    "model": LaunchConfiguration(
                        "model",
                        default="/home/carlos/mil2/src/subjugator/gnc/subjugator_vision/models/start_gate_sim.pt",
                    ),
                    "input_image_topic": LaunchConfiguration(
                        "input_image_topic", default="/front_cam/image_raw"
                    ),
                    "image_reliability": LaunchConfiguration(
                        "image_reliability", default="1"
                    ),
                    "namespace": LaunchConfiguration("namespace", default="yolo"),
                    "device": LaunchConfiguration("device", default="cpu"),
                    "half": LaunchConfiguration("half", default="False"),
                    "threshold": LaunchConfiguration("threshold", default="0.2"),
                    "tracker": LaunchConfiguration("tracker", default="bytetrack.yaml"),
                    "enable": LaunchConfiguration("enable", default="True"),
                    "use_tracking": LaunchConfiguration(
                        "use_tracking", default="False"
                    ),
                    "use_debug": LaunchConfiguration("use_debug", default="True"),
                    # leave use_3d at default False
                }.items(),
            ),
        ]
    )
