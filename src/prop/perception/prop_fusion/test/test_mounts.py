import numpy as np
import pytest
from prop_fusion.mounts import Sensors, default_sensors_file
from scipy.spatial.transform import Rotation

SPEC = """
base_frame: base_link
lidar: {frame: velodyne, xyz: [0, 0, 0.67]}
cameras:
  front_camera: {xyz: [1.0, 0.0, 0.5], rpy: [0, 10, 0]}
  back_camera: {xyz: [-1.0, 0.0, 0.5], rpy: [0, 0, 180]}
"""


@pytest.fixture
def sensors(tmp_path):
    path = tmp_path / "sensors.yaml"
    path.write_text(SPEC)
    return Sensors.load(path)


def mount(sensors, child):
    return next(m for m in sensors.mounts if m.child == child)


def axes(m):
    return Rotation.from_quat(m.quat)


def test_every_sensor_gets_a_frame(sensors):
    children = {m.child for m in sensors.mounts}
    assert children == {
        "velodyne",
        "front_camera_link",
        "front_camera_optical_frame",
        "back_camera_link",
        "back_camera_optical_frame",
    }
    assert sensors.cameras == ("front_camera", "back_camera")


def test_optical_frame_looks_along_the_camera(sensors):
    optical = axes(mount(sensors, "front_camera_optical_frame"))
    assert optical.apply([0, 0, 1]) == pytest.approx([1, 0, 0], abs=1e-9)
    assert optical.apply([1, 0, 0]) == pytest.approx([0, -1, 0], abs=1e-9)
    assert optical.apply([0, 1, 0]) == pytest.approx([0, 0, -1], abs=1e-9)


def test_positive_pitch_tilts_the_lens_down(sensors):
    forward = axes(mount(sensors, "front_camera_link")).apply([1, 0, 0])
    assert forward[2] < 0


def test_back_camera_looks_astern(sensors):
    forward = axes(mount(sensors, "back_camera_link")).apply([1, 0, 0])
    assert forward == pytest.approx([-1, 0, 0], abs=1e-9)


def test_shipped_config_loads():
    sensors = Sensors.load(default_sensors_file())
    assert sensors.cameras
    for m in sensors.mounts:
        assert np.linalg.norm(m.quat) == pytest.approx(1.0)
