"""Sensor placement out of sensors.yaml, as the static transforms that fix it."""

import os
from dataclasses import dataclass

import yaml
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

# Body axes (x forward, y left, z up) to image axes (z forward, x right, y down).
OPTICAL = Rotation.from_euler("xyz", [-90.0, 0.0, -90.0], degrees=True)


@dataclass(frozen=True)
class Mount:
    parent: str
    child: str
    xyz: tuple
    quat: tuple  # x, y, z, w


@dataclass(frozen=True)
class Sensors:
    base_frame: str
    cameras: tuple
    mounts: tuple

    @classmethod
    def load(cls, path):
        with open(path) as f:
            spec = yaml.safe_load(f)
        base = spec.get("base_frame", "base_link")
        cameras = spec.get("cameras") or {}

        mounts = []
        if spec.get("lidar"):
            mounts.append(_mount(base, spec["lidar"]["frame"], spec["lidar"]))
        for name, pose in cameras.items():
            mounts.append(_mount(base, link_frame(name), pose))
            mounts.append(
                Mount(
                    link_frame(name),
                    optical_frame(name),
                    (0.0, 0.0, 0.0),
                    _quat(OPTICAL),
                ),
            )
        return cls(base, tuple(cameras), tuple(mounts))


def default_sensors_file():
    return os.path.join(
        get_package_share_directory("prop_fusion"),
        "config",
        "sensors.yaml",
    )


def link_frame(camera):
    return camera + "_link"


def optical_frame(camera):
    return camera + "_optical_frame"


def _mount(parent, child, pose):
    rotation = Rotation.from_euler(
        "xyz",
        pose.get("rpy", [0.0, 0.0, 0.0]),
        degrees=True,
    )
    return Mount(parent, child, tuple(float(v) for v in pose["xyz"]), _quat(rotation))


def _quat(rotation):
    return tuple(float(v) for v in rotation.as_quat())
