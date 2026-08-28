"""Ring-based 3D pose estimation for the torpedo target board (issue #521, v2).

This replaces the XFeat-homography approach, which was fragile on this board: its
four identical red rings and repeated icons produce ambiguous feature matches, so
the homography was unstable and the recovered range was unreliable.

Instead we exploit the board's *known structure*. The four red target rings sit at
known positions on the board face, so we:

  1. detect the red ring outlines in the image (HSV red + circularity),
  2. match the detections to the known ring layout (combinatorial search minimising
     reprojection error -- this also rejects red clutter like the painted symbols),
  3. solve ``cv2.solvePnP`` for the board's full 6-DoF pose, and
  4. report every ring's 3D position in the camera frame for torpedo aiming.

The module is intentionally free of ROS / torch / gazebo imports so it unit-tests
on its own. Ring geometry was read from the board texture and validated by PnP
against Gazebo ground truth (range within a few percent).
"""

import itertools

import cv2
import numpy as np

# Ring centres in the board frame: origin at the board face centre, x right, y up,
# z = 0 (planar). Metres. Two hole sizes (the big/small pairs) break the board's
# rotational symmetry. Read from Task4_ver2.png and validated against gz ground
# truth. The keys double as target labels; a symbol classifier (future work) maps
# each ring to its emblem (blood-drop / ambulance / fire-truck / flame) for aiming.
RING_LAYOUT = {
    "top_center": (-0.003, 0.192),
    "left": (-0.211, 0.064),
    "bottom_center": (-0.006, -0.200),
    "bottom_right": (0.214, -0.216),
}
RING_NAMES = list(RING_LAYOUT)
_OBJ = np.array(
    [[RING_LAYOUT[n][0], RING_LAYOUT[n][1], 0.0] for n in RING_NAMES],
    dtype=np.float64,
)


def detect_red_rings(bgr, roi=None, min_r=5, max_r=120, min_circ=0.62, min_area=45):
    """Detect the red target rings. Returns ``[(cx, cy, r), ...]`` in image pixels.

    Detection is by the ring's *hole*: each ring encloses a light circular centre,
    so we find HOLES in the red mask (inner contours) that are circular. This is
    far more robust at range than keying on the red band itself -- at a distance
    the band thins and merges with a neighbouring painted symbol (truck/flame),
    which destroys the outer blob's circularity but leaves the enclosed hole
    intact. ``r`` is the hole radius (used only for size-consistency filtering).
    ``roi`` optionally restricts the search; results are in full-image coordinates.
    """
    x0, y0 = 0, 0
    img = bgr
    if roi is not None:
        x0, y0, x1, y1 = (int(v) for v in roi)
        x0, y0 = max(0, x0), max(0, y0)
        img = bgr[y0:y1, x0:x1]
        if img.size == 0:
            return []
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Looser than before (S>=70, V>=50): red is attenuated by water at range.
    mask = cv2.inRange(hsv, (0, 70, 50), (14, 255, 255)) | cv2.inRange(
        hsv,
        (166, 70, 50),
        (180, 255, 255),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    cnts, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    rings = []
    if hier is not None:
        for i, c in enumerate(cnts):
            if hier[0][i][3] == -1:  # outer contour -> skip; we want holes
                continue
            area = cv2.contourArea(c)
            per = cv2.arcLength(c, True)
            if area < min_area or per == 0:
                continue
            (cx, cy), r = cv2.minEnclosingCircle(c)
            if not (min_r <= r <= max_r):
                continue
            if 4.0 * np.pi * area / (per * per) < min_circ:
                continue
            rings.append((cx + x0, cy + y0, r))
    return rings


def solve_board_pose(
    rings_px,
    camera_matrix,
    dist_coeffs=None,
    max_reproj_px=3.0,
    max_candidates=8,
    max_radius_ratio=1.6,
):
    """Match detected rings to the known layout and solve the board's 3D pose.

    Args:
        rings_px: list of ``(cx, cy, r)`` detected rings (>= 4 needed).
        camera_matrix: 3x3 intrinsics ``K``.
        dist_coeffs: distortion, or ``None`` for a pinhole.
        max_reproj_px: reject the solve if the best mean reprojection error exceeds
            this (guards against clutter that happens to include 4 red blobs).
        max_candidates: cap on how many detected rings to consider (by radius,
            largest first) to bound the combinatorial search.

    Returns ``(ok, rvec, tvec, correspondence, reproj_px)`` where ``correspondence``
    maps each ``RING_NAMES`` label to its ``(cx, cy)`` image point, and ``rvec``/
    ``tvec`` place the board frame in the camera optical frame (x right, y down,
    z forward). ``ok`` is ``False`` if no consistent 4-ring assignment was found.
    """
    K = np.asarray(camera_matrix, dtype=np.float64)
    dist = np.zeros(5) if dist_coeffs is None else np.asarray(dist_coeffs, np.float64)
    if len(rings_px) < 4:
        return False, None, None, None, None

    cand = sorted(rings_px, key=lambda t: -t[2])[:max_candidates]
    centers = [np.array([c[0], c[1]], dtype=np.float64) for c in cand]
    radii = [float(c[2]) for c in cand]

    best = None  # (err, rvec, tvec, perm_indices)
    for four in itertools.combinations(range(len(centers)), 4):
        # The four real rings are similar in size (the big/small pair ratio is
        # ~1.2); a false hole (a symbol's window) is markedly smaller, so a set
        # whose radii span too wide a ratio can't be the four rings. This rejects
        # false candidates that would otherwise fit a plane with low reprojection.
        rs = [radii[i] for i in four]
        if max(rs) / max(min(rs), 1e-6) > max_radius_ratio:
            continue
        img4 = np.array([centers[i] for i in four])
        for perm in itertools.permutations(range(4)):
            ordered = img4[list(perm)]
            ok, rvec, tvec = cv2.solvePnP(
                _OBJ,
                ordered,
                K,
                dist,
                flags=cv2.SOLVEPNP_IPPE,
            )
            if not ok or tvec[2] <= 0:
                continue
            proj, _ = cv2.projectPoints(_OBJ, rvec, tvec, K, dist)
            err = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - ordered, axis=1)))
            if best is None or err < best[0]:
                best = (err, rvec, tvec, [four[p] for p in perm])

    if best is None or best[0] > max_reproj_px:
        return False, None, None, None, (best[0] if best else None)

    err, rvec, tvec, idx = best
    corr = {
        RING_NAMES[i]: (float(centers[idx[i]][0]), float(centers[idx[i]][1]))
        for i in range(4)
    }
    return True, rvec, tvec, corr, err


def hole_positions_camera(rvec, tvec):
    """3D position of every ring in the camera frame. ``{label: (x, y, z)}`` metres.

    These are the points to aim torpedoes at.
    """
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    t = np.asarray(tvec, dtype=np.float64).reshape(3)
    out = {}
    for name, (x, y) in RING_LAYOUT.items():
        p = R @ np.array([x, y, 0.0]) + t
        out[name] = (float(p[0]), float(p[1]), float(p[2]))
    return out
