"""3D pose estimation for the torpedo target board.

This module is intentionally free of ROS / torch / gazebo imports so it can be
unit-tested on its own (see ``test/test_torpedo_pose.py``).

The torpedo perception pipeline (``torpedo.py``) matches XFeat keypoints between a
stored reference image of the board and the live front-camera frame and computes a
homography ``H``.  Warping the reference-image corners through ``H`` gives the four
board corners in the live image.  Those 2D corners, together with the camera
intrinsics ``K`` and the board's known physical size, are enough to recover the
board's full 3D pose relative to the camera via ``cv2.solvePnP`` (issue #521).
"""

import cv2
import numpy as np

# Physical size of the flat torpedo-target face, in meters.  These set the absolute
# scale (range) of the returned pose -- bearing and orientation are correct
# regardless, but distance is only as accurate as these numbers.
#
# Source: the ``Board_V2`` object in the simulation mesh
# subjugator_description/models/torpedoMapping_2026_v2/torpedoMapping_2026_v2.obj,
# which is a zero-thickness plane measuring 0.6096 x 0.6096 m -- i.e. exactly
# 24 x 24 inches.  Landing on a round imperial value strongly suggests it was
# transcribed from the RoboSub spec rather than eyeballed.
# TODO(#521): confirm against the competition handbook / a tape measure on the
# real board before trusting absolute range in the water.
BOARD_WIDTH_M = 0.6096
BOARD_HEIGHT_M = 0.6096

# Corner ordering used throughout the pipeline: top-left, top-right, bottom-right,
# bottom-left.  This matches the order the reference corners are warped in
# ``torpedo.py`` so 2D image corners line up with the 3D object points below.
CORNER_ORDER = ("top_left", "top_right", "bottom_right", "bottom_left")


def default_camera_matrix(width, height, hfov_rad):
    """Pinhole camera matrix ``K`` from image size and horizontal field of view.

    Used as a fallback when no ``CameraInfo`` is published (e.g. the current
    front-cam driver).  In simulation the exact FOV is known from the Gazebo
    camera sensor; on real hardware prefer a calibrated ``K``.
    """
    fx = fy = (width / 2.0) / np.tan(hfov_rad / 2.0)
    cx = width / 2.0
    cy = height / 2.0
    return np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def board_object_points(width_m=BOARD_WIDTH_M, height_m=BOARD_HEIGHT_M):
    """3D board corners in the board's own frame (planar, ``z = 0``).

    Origin at the board center, ``x`` right, ``y`` up.  Ordered per ``CORNER_ORDER``.
    """
    w = width_m / 2.0
    h = height_m / 2.0
    return np.array(
        [
            [-w, h, 0.0],  # top-left
            [w, h, 0.0],  # top-right
            [w, -h, 0.0],  # bottom-right
            [-w, -h, 0.0],  # bottom-left
        ],
        dtype=np.float64,
    )


def estimate_board_pose(
    corners_px,
    camera_matrix,
    dist_coeffs=None,
    width_m=BOARD_WIDTH_M,
    height_m=BOARD_HEIGHT_M,
):
    """Recover the board's 3D pose from its four image corners.

    Args:
        corners_px: 4x2 array-like of image corners in ``CORNER_ORDER``.
        camera_matrix: 3x3 intrinsics ``K``.
        dist_coeffs: lens distortion, or ``None`` for a distortion-free pinhole.
        width_m, height_m: physical board size (sets absolute range).

    Returns:
        ``(ok, rvec, tvec)`` where ``rvec`` is a Rodrigues rotation vector and
        ``tvec`` is the board-center position, both in the camera optical frame
        (x right, y down, z forward).  ``ok`` is ``False`` if the solve failed.
    """
    obj = board_object_points(width_m, height_m)
    img = np.asarray(corners_px, dtype=np.float64).reshape(-1, 2)
    if img.shape[0] != 4:
        return False, None, None
    if dist_coeffs is None:
        dist_coeffs = np.zeros(5, dtype=np.float64)

    # IPPE_SQUARE is purpose-built for a planar 4-point target and is far more
    # stable here than the default iterative solver.
    ok, rvec, tvec = cv2.solvePnP(
        obj,
        img,
        np.asarray(camera_matrix, dtype=np.float64),
        np.asarray(dist_coeffs, dtype=np.float64),
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    return bool(ok), rvec, tvec


def rotation_vector_to_quaternion(rvec):
    """Rodrigues rotation vector -> quaternion ``(x, y, z, w)``.

    Standalone so we don't depend on ``transforms3d`` (which is currently broken
    under numpy>=2.0 in this environment).
    """
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)
