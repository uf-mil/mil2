"""Round-trip tests for torpedo board 3D pose estimation.

We place the board at a known pose, project its corners into the image with a known
camera matrix, then check that ``estimate_board_pose`` recovers the original pose.
No ROS / torch / gazebo required -- run directly with ``python3`` or ``pytest``.
"""

import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "admission"))

from torpedo_pose import (
    BOARD_HEIGHT_M,
    BOARD_WIDTH_M,
    board_object_points,
    default_camera_matrix,
    estimate_board_pose,
    rotation_vector_to_quaternion,
)

WIDTH, HEIGHT = 640, 360
K = default_camera_matrix(WIDTH, HEIGHT, hfov_rad=np.deg2rad(80.0))


def project(rvec, tvec, width_m=BOARD_WIDTH_M, height_m=BOARD_HEIGHT_M):
    obj = board_object_points(width_m, height_m)
    pts, _ = cv2.projectPoints(obj, rvec, tvec, K, np.zeros(5))
    return pts.reshape(-1, 2)


def test_recovers_frontal_pose():
    # Board 3 m straight ahead, no rotation.
    rvec_true = np.zeros(3)
    tvec_true = np.array([0.0, 0.0, 3.0])
    corners = project(rvec_true, tvec_true)

    ok, rvec, tvec = estimate_board_pose(corners, K)
    assert ok
    assert np.linalg.norm(tvec.ravel() - tvec_true) < 1e-3, tvec.ravel()
    assert np.linalg.norm(rvec.ravel() - rvec_true) < 1e-3, rvec.ravel()


def test_recovers_angled_pose():
    # Board 2.5 m ahead, offset to the side, yawed ~25 deg and pitched ~10 deg.
    rvec_true = np.array([np.deg2rad(10.0), np.deg2rad(25.0), 0.0])
    tvec_true = np.array([0.4, -0.2, 2.5])
    corners = project(rvec_true, tvec_true)

    ok, rvec, tvec = estimate_board_pose(corners, K)
    assert ok
    # Position within 1 cm, rotation within ~0.5 deg.
    assert np.linalg.norm(tvec.ravel() - tvec_true) < 1e-2, tvec.ravel()
    assert np.linalg.norm(rvec.ravel() - rvec_true) < 1e-2, rvec.ravel()


def test_range_scales_with_board_size():
    # If we assume a board twice as large, the recovered range doubles.
    rvec_true = np.zeros(3)
    tvec_true = np.array([0.0, 0.0, 3.0])
    corners = project(rvec_true, tvec_true, width_m=0.6, height_m=0.6)

    _, _, tvec = estimate_board_pose(corners, K, width_m=1.2, height_m=1.2)
    assert abs(tvec.ravel()[2] - 6.0) < 1e-2, tvec.ravel()


def test_quaternion_identity():
    q = rotation_vector_to_quaternion(np.zeros(3))
    assert np.allclose(q, [0.0, 0.0, 0.0, 1.0]), q


def test_quaternion_matches_opencv():
    rvec = np.array([0.3, -0.2, 0.7])
    q = rotation_vector_to_quaternion(rvec)
    # Reconstruct rotation matrix from quaternion and compare to Rodrigues.
    x, y, z, w = q
    R_q = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
    )
    R_cv, _ = cv2.Rodrigues(rvec)
    assert np.allclose(R_q, R_cv, atol=1e-9)


if __name__ == "__main__":
    passed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS {name}")
            passed += 1
    print(f"\n{passed} tests passed")
