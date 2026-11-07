"""
The boat's cameras, one looking ahead and one astern. Each runs in its own
namespace: camera_node reads the device, vision_node detects on its frames.

    /front_camera/image_raw  /front_camera/detections  /front_camera/camera_info
    /back_camera/image_raw   /back_camera/detections   /back_camera/camera_info

    ros2 launch prop_vision cameras.launch.py model:=<weights.pt>
    ros2 launch prop_vision cameras.launch.py detect:=false
    ros2 launch prop_vision cameras.launch.py model:=<weights.pt> \
        back_device:=/dev/v4l/by-path/<port>-video-index0

detect:=false runs the cameras alone, to look at them or record them.

They are the same model, so /dev/v4l/by-id cannot tell them apart and
/dev/videoN reorders between boots; by-path names the USB port, which is fixed
by how they are plugged in. If a camera moves to another port, pass its path
here. v4l2-ctl --list-devices shows the ports, and video-index0 is the node with
frames. The back camera stays off until it is given a device.

Each camera has a deck line, because the model sees buoys in the boat's own
fittings. It depends on how the camera is mounted, so measure it again when one
moves.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAMERAS = ("front_camera", "back_camera")
FRONT_DEVICE = "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4.1:1.0-video-index0"


def camera(name, device, width, height):
    return Node(
        package="prop_vision",
        executable="camera_node.py",
        name="camera",
        namespace=name,
        parameters=[
            {
                "device": device,
                "width": width,
                "height": height,
                "frame_id": f"{name}_optical_frame",
            },
        ],
        # A camera that drops off the bus comes back when it is reopened.
        respawn=True,
        respawn_delay=2.0,
        output="screen",
    )


def vision(name, model, deck_line, publish_image):
    parameters = {
        "frame_id": f"{name}_optical_frame",
        "publish_image": publish_image,
        "deck_line": deck_line,
    }
    if model:
        parameters["model"] = os.path.expanduser(model)
    return Node(
        package="prop_vision",
        executable="vision_node.py",
        name="vision",
        namespace=name,
        parameters=[parameters],
        # Undistorter reports the calibration it scaled with print.
        additional_env={"PYTHONUNBUFFERED": "1"},
        output="screen",
    )


def cameras(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    nodes = []
    for name in CAMERAS:
        side = name.split("_")[0]
        device = argument(f"{side}_device")
        if not device:
            continue
        nodes.append(
            camera(name, device, int(argument("width")), int(argument("height"))),
        )
        if argument("detect") == "true":
            nodes.append(
                vision(
                    name,
                    argument("model"),
                    float(argument(f"{side}_deck_line")),
                    argument("publish_image") == "true",
                ),
            )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "front_device",
                default_value=FRONT_DEVICE,
                description="The camera looking ahead: /dev/v4l/by-path/...",
            ),
            DeclareLaunchArgument(
                "back_device",
                default_value="",
                description="The camera looking astern. Empty leaves it off.",
            ),
            DeclareLaunchArgument(
                "width",
                default_value="640",
                description="Width to ask the cameras for.",
            ),
            DeclareLaunchArgument(
                "height",
                default_value="360",
                description="Height to ask the cameras for.",
            ),
            DeclareLaunchArgument(
                "detect",
                default_value="true",
                description="Run the detector. false leaves only the cameras.",
            ),
            DeclareLaunchArgument(
                "front_deck_line",
                default_value="0.78",
                description="Boxes reaching past this fraction of the image's "
                "height are dropped as the boat's own deck. Measured on the "
                "2026-10-18 mount.",
            ),
            DeclareLaunchArgument(
                "back_deck_line",
                default_value="1.0",
                description="The same for the camera looking astern. 1.0 drops "
                "nothing, until it is measured.",
            ),
            DeclareLaunchArgument(
                "model",
                default_value="",
                description="YOLO weights for both. Empty is models/best.pt "
                "beside vision_node.py.",
            ),
            DeclareLaunchArgument(
                "publish_image",
                default_value="false",
                description="Also publish image_rect, for RViz or "
                "prop_fusion's debug_image.",
            ),
            OpaqueFunction(function=cameras),
        ],
    )
