"""Torpedo target-board 3D pose + hole localization (issue #521, ring-PnP).

Pipeline: front-cam frame -> detect the four red target rings -> match them to the
board's known ring layout and solve ``cv2.solvePnP`` -> publish the board's 3D pose,
every hole's 3D aim point, RViz markers, and an annotated ``/debug_img``.

This replaces the earlier XFeat-homography approach, which was unreliable on this
board (four identical rings + repeated icons -> ambiguous matches -> unstable
homography -> bad range). Ring-PnP exploits the board's known geometry instead and
runs on the full-board view (no YOLO detection needed). See ``board_targets.py``.
"""

import contextlib
import math
import os

import adm
import cv2
import numpy as np
from board_targets import (
    RING_LAYOUT,
    detect_red_rings,
    hole_positions_camera,
    solve_board_pose,
)
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import CameraInfo, Image
from torpedo_pose import rotation_vector_to_quaternion
from visualization_msgs.msg import Marker

# Horizontal FOV of the front camera (rad), used only as a fallback when no
# CameraInfo has arrived. In sim the bridge publishes /front_cam/camera_info with
# the real intrinsics (fx ~ 336 for the 960x600 wide cam), which we prefer -- the
# absolute range from PnP is only as good as fx.
FRONT_CAM_HFOV_RAD = 1.919862177

_camera_matrix = None
_dist_coeffs = None


def _camera_info_cb(msg):
    global _camera_matrix, _dist_coeffs
    _camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
    _dist_coeffs = np.array(msg.d, dtype=np.float64) if msg.d else None


adm.node.create_subscription(CameraInfo, "/front_cam/camera_info", _camera_info_cb, 10)


def _current_camera_matrix(width, height):
    """Calibrated K from CameraInfo if we've received one, else the FOV fallback."""
    if _camera_matrix is not None:
        return _camera_matrix, _dist_coeffs
    fx = fy = (width / 2.0) / math.tan(FRONT_CAM_HFOV_RAD / 2.0)
    K = np.array([[fx, 0.0, width / 2.0], [0.0, fy, height / 2.0], [0.0, 0.0, 1.0]])
    return K, None


# --- Ground-truth range (sim only) for validating the estimate on the HUD -------
# Computed camera->board so it is directly comparable to the PnP "detected" range.
_gt_enabled = None
_gt_sub = None  # (x, y, z, qx, qy, qz, qw) latest sub9 world pose
_gz_node = None
_BOARD_XYZ = np.array([-4.0, 20.0, -0.8])
_CAM_OFFSET = np.array([0.78, -0.014, -0.05])  # front_cam_link in sub9 body frame


def _init_ground_truth():
    global _gz_node, _gt_enabled, _BOARD_XYZ
    if _gt_enabled is not None:
        return
    try:
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode
    except Exception:
        _gt_enabled = False
        return
    env = os.environ.get("TORPEDO_BOARD_XYZ")
    if env:
        with contextlib.suppress(Exception):
            _BOARD_XYZ = np.array([float(v) for v in env.split(",")])
    world = os.environ.get("TORPEDO_GT_WORLD", "task4_2026_v2")

    def _pose_cb(msg):
        global _gt_sub
        for p in msg.pose:
            if p.name == "sub9":
                o = p.orientation
                _gt_sub = (p.position.x, p.position.y, p.position.z, o.x, o.y, o.z, o.w)
                return

    _gz_node = GzNode()
    if _gz_node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", _pose_cb):
        _gt_enabled = True
    else:
        _gz_node, _gt_enabled = None, False


def _actual_range():
    """True camera->board distance (m), or None -- comparable to the PnP range."""
    if not _gt_enabled or _gt_sub is None:
        return None
    x, y, z, qx, qy, qz, qw = _gt_sub
    R = np.array(
        [
            [
                1 - 2 * (qy * qy + qz * qz),
                2 * (qx * qy - qz * qw),
                2 * (qx * qz + qy * qw),
            ],
            [
                2 * (qx * qy + qz * qw),
                1 - 2 * (qx * qx + qz * qz),
                2 * (qy * qz - qx * qw),
            ],
            [
                2 * (qx * qz - qy * qw),
                2 * (qy * qz + qx * qw),
                1 - 2 * (qx * qx + qy * qy),
            ],
        ],
    )
    cam_world = np.array([x, y, z]) + R @ _CAM_OFFSET
    return float(np.linalg.norm(cam_world - _BOARD_XYZ))


# --- publishing ----------------------------------------------------------------
def _board_pose_msg(rvec, tvec):
    x, y, z, w = rotation_vector_to_quaternion(rvec)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in tvec.ravel())
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        float(x),
        float(y),
        float(z),
        float(w),
    )
    return pose


def _publish_markers(rvec, tvec, holes, frame_id):
    """Board-center pose marker + a sphere at each hole's 3D aim point."""
    board = Marker()
    board.header.frame_id = frame_id
    board.ns, board.id, board.type, board.action = (
        "torpedo_board",
        0,
        Marker.SPHERE,
        Marker.ADD,
    )
    board.pose = _board_pose_msg(rvec, tvec)
    board.scale.x = board.scale.y = board.scale.z = 0.05
    board.color.r, board.color.g, board.color.b, board.color.a = 0.2, 0.6, 1.0, 0.9
    adm.marker_pub.publish(board)

    for i, (name, (px, py, pz)) in enumerate(holes.items(), start=1):
        m = Marker()
        m.header.frame_id = frame_id
        m.ns, m.id, m.type, m.action = "torpedo_holes", i, Marker.SPHERE, Marker.ADD
        m.pose.position = Point(x=px, y=py, z=pz)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.10
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 0.9
        adm.marker_pub.publish(m)


def _hud_text(im, text, y, scale, color):
    cv2.putText(
        im,
        text,
        (10, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        4,
        cv2.LINE_AA,
    )
    cv2.putText(
        im,
        text,
        (10, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        2,
        cv2.LINE_AA,
    )


async def torpedo():
    _init_ground_truth()
    adm.node.get_logger().info(
        "[torpedo] ready -- ring-PnP board pose; waiting for /front_cam/image_raw",
    )
    async for packet in adm.Join(adm.frontcam_sub):
        img = packet[0]
        if img is None:
            continue

        # Copy out of the message buffer (a view would be mutated by drawing).
        im = np.ndarray((img.height, img.width, 3), np.uint8, buffer=img.data).copy()
        bgr = cv2.cvtColor(im, cv2.COLOR_RGB2BGR) if img.encoding == "rgb8" else im

        rings = detect_red_rings(bgr)
        K, dist = _current_camera_matrix(img.width, img.height)
        ok, rvec, tvec, corr, reproj = solve_board_pose(rings, K, dist)

        # faint outline of every red-ring candidate
        for x, y, r in rings:
            cv2.circle(im, (int(x), int(y)), int(r), (0, 170, 0), 1)

        rng = None
        if ok:
            rng = float(np.linalg.norm(tvec))
            frame_id = img.header.frame_id or "front_cam_link"
            holes = hole_positions_camera(rvec, tvec)
            _publish_markers(rvec, tvec, holes, frame_id)
            adm.node.get_logger().info(
                f"board @ {rng:.2f} m  reproj={reproj:.2f}px  rings={len(rings)}",
            )
            # labeled, solved rings (green) connected in layout order
            order = list(RING_LAYOUT)
            poly = np.array([corr[n] for n in order], dtype=np.int32)
            cv2.polylines(im, [poly], True, (0, 255, 0), 1, cv2.LINE_AA)
            for name, (x, y) in corr.items():
                cv2.circle(im, (int(x), int(y)), 7, (0, 255, 0), 2)
                cv2.putText(
                    im,
                    name,
                    (int(x) + 8, int(y) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

        # HUD: detected (PnP) vs actual (gt) range + fit quality
        actual = _actual_range()
        if rng is not None:
            det = (
                f"detected: {rng:.2f} m   "
                f"x={tvec.ravel()[0]:.2f} y={tvec.ravel()[1]:.2f} z={tvec.ravel()[2]:.2f}"
            )
            det_color = (0, 255, 0)
        else:
            det = f"detected: ---  ({len(rings)}/4 rings)"
            det_color = (255, 255, 255)
        if actual is not None and rng is not None:
            act = f"actual: {actual:.2f} m   (err {rng - actual:+.2f} m)"
        elif actual is not None:
            act = f"actual: {actual:.2f} m"
        else:
            act = "actual: ---"
        reproj_s = (
            f"rings: {len(rings)}   reproj: {reproj:.2f} px"
            if ok
            else f"rings: {len(rings)}   reproj: --"
        )
        _hud_text(im, det, 30, 0.7, det_color)
        _hud_text(im, act, 58, 0.7, (0, 255, 255))
        _hud_text(im, reproj_s, 84, 0.6, (255, 255, 255))

        dbg = Image()
        dbg.header = img.header
        dbg.height, dbg.width = img.height, img.width
        dbg.encoding = img.encoding
        dbg.step = img.step
        dbg.data = im.tobytes()
        adm.debug_pub.publish(dbg)


if __name__ == "__main__":
    adm.run(torpedo())
