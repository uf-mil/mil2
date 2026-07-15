# Copyright (C) 2024 Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Absolute path to the installed weights. Relative paths break because the
    # yolo_node resolves them against its runtime cwd, not this launch file.
    # Requires `colcon build --packages-select subjugator_vision` so the .pt is
    # copied into the package share/ directory.
    model_path = os.path.join(
        get_package_share_directory("subjugator_vision"),
        "models",
        "torp_model.pt",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("yolo_bringup"),
                        "launch",
                        "yolo.launch.py",
                    ),
                ),
                launch_arguments={
                    # model_type "YOLO" lets Ultralytics auto-detect the YOLO26
                    # architecture from the weights file. Override "model" with a
                    # path to other yolo26 weights, e.g. model:=/abs/path/to.pt.
                    "model_type": LaunchConfiguration("model_type", default="YOLO"),
                    "model": LaunchConfiguration(
                        "model",
                        default=model_path,
                    ),
                    "tracker": LaunchConfiguration("tracker", default="bytetrack.yaml"),
                    "device": LaunchConfiguration("device", default="cuda:0"),
                    "enable": LaunchConfiguration("enable", default="True"),
                    "half": LaunchConfiguration("half", default="True"),
                    "threshold": LaunchConfiguration("threshold", default="0.5"),
                    "input_image_topic": LaunchConfiguration(
                        "input_image_topic",
                        default="/front_cam/image_raw",
                    ),
                    "image_reliability": LaunchConfiguration(
                        "image_reliability",
                        default="1",
                    ),
                    "namespace": LaunchConfiguration("namespace", default="yolo"),
                }.items(),
            ),
        ],
    )
