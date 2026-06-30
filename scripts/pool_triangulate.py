#!/usr/bin/env python3
"""
Localize underwater competition elements from a tripod camera outside the pool.

Single-view mode  (1 image):
  Shoot a ray from camera → refract at water surface → intersect depth plane.
  Requires --depth assumption (default = pool floor).

Two-view mode  (2 images, recommended):
  Shoot refracted rays from both camera positions; find closest-point between
  the pair. No depth assumption needed — depth is solved by geometry.

Pool world frame (meters):
  Origin = Top-Left corner at water surface
  +X     = long axis  (50 m)
  +Y     = short axis (22.86 m)
  +Z     = up  (water surface z=0, pool floor z=-2.1336)

Corner click order: Top-Left → Top-Right → Bottom-Right → Bottom-Left
  ("Top" = far end relative to your camera vantage point)

Usage:
  # two-view (best):
  python pool_triangulate.py pos1.jpg pos2.jpg [--calib cal.npz]

  # single-view fallback:
  python pool_triangulate.py pos1.jpg [--calib cal.npz] [--depth 2.1336]

  --calib  .npz with keys 'camera_matrix' and 'dist_coeffs'
  --fov    horizontal FOV in degrees if no --calib (default 70)
  --depth  prop depth below surface in metres for single-view mode
"""

from __future__ import annotations

import argparse
import sys
from typing import Optional

import cv2
import numpy as np

# ── Pool constants ────────────────────────────────────────────────────────────
POOL_LENGTH = 50.0
POOL_WIDTH = 22.86
POOL_DEPTH = 2.1336

N_AIR = 1.0
N_WATER = 1.333

CORNER_LABELS = ["TL", "TR", "BR", "BL"]
POOL_CORNERS_3D = np.array(
    [
        [0.0, 0.0, 0.0],
        [POOL_LENGTH, 0.0, 0.0],
        [POOL_LENGTH, POOL_WIDTH, 0.0],
        [0.0, POOL_WIDTH, 0.0],
    ],
    dtype=np.float64,
)


# ── Geometry ──────────────────────────────────────────────────────────────────
def snell_refract(
    d: np.ndarray, normal: np.ndarray, n1: float, n2: float
) -> Optional[np.ndarray]:
    """Snell's law in vector form. Returns refracted unit vector or None for TIR."""
    r = n1 / n2
    cos_i = float(-np.dot(d, normal))
    sin2_t = r**2 * (1.0 - cos_i**2)
    if sin2_t > 1.0:
        return None
    cos_t = float(np.sqrt(max(0.0, 1.0 - sin2_t)))
    out = r * d + (r * cos_i - cos_t) * normal
    return out / np.linalg.norm(out)


def ray_z_intersect(
    origin: np.ndarray, direction: np.ndarray, z: float
) -> Optional[tuple[np.ndarray, float]]:
    """Intersect ray with horizontal plane z=const. Returns (point, t) or None."""
    dz = direction[2]
    if abs(dz) < 1e-10:
        return None
    t = (z - origin[2]) / dz
    if t < 0:
        return None
    return origin + t * direction, t


def closest_point_two_rays(
    o1: np.ndarray, d1: np.ndarray,
    o2: np.ndarray, d2: np.ndarray,
) -> tuple[np.ndarray, float]:
    """
    Midpoint of the shortest segment between two 3-D lines.
    Returns (midpoint, gap) where gap is the distance between closest points
    (ideally ~0 for perfect triangulation; non-zero = reprojection error).
    """
    w = o1 - o2
    b = float(np.dot(d1, d2))
    denom = 1.0 - b * b
    if abs(denom) < 1e-10:
        return o1, float(np.linalg.norm(np.cross(w, d1)))
    d = float(np.dot(d1, w))
    e = float(np.dot(d2, w))
    t1 = (b * e - d) / denom
    t2 = (e - b * d) / denom
    p1 = o1 + t1 * d1
    p2 = o2 + t2 * d2
    gap = float(np.linalg.norm(p1 - p2))
    return (p1 + p2) / 2.0, gap


# ── Per-image view ────────────────────────────────────────────────────────────
class CameraView:
    """
    Handles one image: PnP pose from pool corners, then collects refracted
    rays for each clicked prop.
    """

    def __init__(
        self,
        image_path: str,
        K: np.ndarray,
        D: np.ndarray,
        label: str = "",
    ):
        img = cv2.imread(image_path)
        if img is None:
            sys.exit(f"ERROR: Cannot read image: {image_path}")
        self.image = img
        self.display = img.copy()
        self.K = K
        self.D = D
        self.label = label

        self.corners_2d: list[list[float]] = []
        self.R: Optional[np.ndarray] = None
        self.t: Optional[np.ndarray] = None
        self.mode = "corners"

        # Each entry: (surface_point, refracted_unit_direction) in world frame
        self.prop_rays: list[tuple[np.ndarray, np.ndarray]] = []

    # ── Pose ──────────────────────────────────────────────────────────────────
    def cam_pos(self) -> np.ndarray:
        return -(self.R.T @ self.t)

    def _solve_pose(self):
        pts2d = np.array(self.corners_2d, dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            POOL_CORNERS_3D, pts2d, self.K, self.D, flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            print("ERROR: solvePnP failed — verify corner click order TL→TR→BR→BL.")
            return
        self.R, _ = cv2.Rodrigues(rvec)
        self.t = tvec.flatten()
        cp = self.cam_pos()
        print(f"  [{self.label}] Camera world pos: X={cp[0]:.3f}  Y={cp[1]:.3f}  Z={cp[2]:.3f} m")
        self.mode = "props"
        print(f"  [{self.label}] Pose solved. Click props in order, then press Enter or 'n'.\n")

    # ── Ray building ──────────────────────────────────────────────────────────
    def _pixel_ray(self, x: int, y: int) -> np.ndarray:
        """Pixel → undistorted unit ray in world frame."""
        pt = cv2.undistortPoints(
            np.array([[[float(x), float(y)]]], dtype=np.float64), self.K, self.D
        )[0][0]
        ray_cam = np.array([pt[0], pt[1], 1.0])
        ray_cam /= np.linalg.norm(ray_cam)
        ray_world = self.R.T @ ray_cam
        return ray_world / np.linalg.norm(ray_world)

    def compute_refracted_ray(
        self, x: int, y: int
    ) -> Optional[tuple[np.ndarray, np.ndarray]]:
        """
        Returns (surface_point, refracted_direction) for a clicked pixel, or
        None if the ray misses the surface or hits TIR (shouldn't happen air→water).
        """
        cam = self.cam_pos()
        ray = self._pixel_ray(x, y)
        result = ray_z_intersect(cam, ray, 0.0)
        if result is None:
            print("  WARNING: ray does not intersect water surface.")
            return None
        surf_pt, _ = result
        refracted = snell_refract(ray, np.array([0.0, 0.0, 1.0]), N_AIR, N_WATER)
        if refracted is None:
            print("  WARNING: TIR (unexpected for air→water).")
            return None
        return surf_pt, refracted

    # ── Single-view localisation (depth assumption) ───────────────────────────
    def localize_single(self, x: int, y: int, depth: float) -> Optional[np.ndarray]:
        result = self.compute_refracted_ray(x, y)
        if result is None:
            return None
        surf_pt, refracted = result
        hit = ray_z_intersect(surf_pt, refracted, -depth)
        if hit is None:
            print("  WARNING: refracted ray does not reach target depth.")
            return None
        return hit[0]

    # ── UI ────────────────────────────────────────────────────────────────────
    def _on_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.mode == "corners":
            idx = len(self.corners_2d)
            self.corners_2d.append([float(x), float(y)])
            lbl = CORNER_LABELS[idx]
            cv2.circle(self.display, (x, y), 6, (0, 220, 0), -1)
            cv2.putText(
                self.display, lbl, (x + 9, y + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 0), 2,
            )
            print(f"  [{self.label}] Corner {lbl}: ({x}, {y})")
            if len(self.corners_2d) == 4:
                self._solve_pose()

        elif self.mode == "props":
            ray = self.compute_refracted_ray(x, y)
            if ray is None:
                return
            self.prop_rays.append(ray)
            n = len(self.prop_rays)
            cv2.circle(self.display, (x, y), 6, (0, 80, 255), -1)
            cv2.putText(
                self.display, f"P{n}", (x + 9, y + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 80, 255), 2,
            )
            print(f"  [{self.label}] Prop {n} ray recorded at pixel ({x}, {y})")

    def run_interactive(self) -> bool:
        """Open interactive window. Returns True if pose was solved."""
        win = f"View: {self.label}"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(win, self._on_click)

        h, w = self.image.shape[:2]
        print(f"\n[{self.label}] Image: {w}×{h}")
        print(f"[{self.label}] Click corners TL→TR→BR→BL, then props. Press 'n' when done.\n")

        while True:
            disp = self.display.copy()
            if self.mode == "corners":
                remaining = CORNER_LABELS[len(self.corners_2d):]
                msg = f"[{self.label}] Next corner: {remaining[0] if remaining else '—'}"
            else:
                msg = f"[{self.label}] Props: {len(self.prop_rays)}  |  'n' = next view / finish"
            cv2.putText(disp, msg, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 230, 30), 2)
            cv2.imshow(win, disp)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord("n"), ord("q"), 13):  # n / q / Enter
                break

        cv2.destroyWindow(win)
        return self.R is not None


# ── Results ───────────────────────────────────────────────────────────────────
def print_results(positions: list[tuple[np.ndarray, Optional[float]]]):
    print("\n=== Triangulated prop positions ===")
    if not positions:
        print("  (none)")
        return
    for i, (pos, gap) in enumerate(positions, 1):
        gap_str = f"  ray-gap={gap*100:.1f} cm" if gap is not None else ""
        print(f"  Prop {i:2d}: X={pos[0]:.3f} m  Y={pos[1]:.3f} m  Z={pos[2]:.3f} m{gap_str}")


# ── Entry point ───────────────────────────────────────────────────────────────
def approximate_K(image_shape, fov_deg: float) -> np.ndarray:
    h, w = image_shape[:2]
    f = w / (2.0 * np.tan(np.radians(fov_deg / 2.0)))
    return np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1.0]], dtype=np.float64)


def main():
    ap = argparse.ArgumentParser(
        description="Localize pool props via PnP + Snell refraction. "
                    "Pass 2 images for depth-free two-view mode.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("images", nargs="+", metavar="IMAGE", help="1 or 2 pool images")
    ap.add_argument("--calib", help=".npz with keys 'camera_matrix' and 'dist_coeffs'")
    ap.add_argument("--fov", type=float, default=70.0, help="Horizontal FOV (deg) if no --calib")
    ap.add_argument(
        "--depth", type=float, default=POOL_DEPTH,
        help=f"Prop depth (m) for single-view mode (default {POOL_DEPTH:.4f} = floor)",
    )
    args = ap.parse_args()

    if len(args.images) > 2:
        ap.error("Pass at most 2 images.")

    # Camera intrinsics
    if args.calib:
        data = np.load(args.calib)
        K, D = data["camera_matrix"], data["dist_coeffs"]
    else:
        probe = cv2.imread(args.images[0])
        if probe is None:
            sys.exit(f"ERROR: Cannot read {args.images[0]}")
        K = approximate_K(probe.shape, args.fov)
        D = np.zeros(5, dtype=np.float64)
        print(
            f"WARNING: No calibration file — using approximate pinhole (FOV={args.fov}°).\n"
            "         Accuracy will suffer; run cv2.calibrateCamera() for better results.\n"
        )

    # ── Single-view mode ──────────────────────────────────────────────────────
    if len(args.images) == 1:
        print(f"Mode: single-view  (depth assumption = {args.depth:.4f} m)\n")
        view = CameraView(args.images[0], K, D, label="View1")

        # Wrap single-view props through the UI but resolve immediately
        win = "View: View1"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

        results: list[tuple[np.ndarray, Optional[float]]] = []

        def on_click_single(event, x, y, flags, param):
            if event != cv2.EVENT_LBUTTONDOWN:
                return
            if view.mode == "corners":
                idx = len(view.corners_2d)
                view.corners_2d.append([float(x), float(y)])
                lbl = CORNER_LABELS[idx]
                cv2.circle(view.display, (x, y), 6, (0, 220, 0), -1)
                cv2.putText(view.display, lbl, (x + 9, y + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 0), 2)
                print(f"  Corner {lbl}: ({x}, {y})")
                if len(view.corners_2d) == 4:
                    view._solve_pose()
            elif view.mode == "props":
                pos = view.localize_single(x, y, args.depth)
                if pos is None:
                    return
                results.append((pos, None))
                n = len(results)
                lbl = f"P{n} ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})m"
                cv2.circle(view.display, (x, y), 6, (0, 80, 255), -1)
                cv2.putText(view.display, lbl, (x + 9, y + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 80, 255), 2)
                print(f"  Prop {n}: X={pos[0]:.3f}  Y={pos[1]:.3f}  Z={pos[2]:.3f} m")

        cv2.setMouseCallback(win, on_click_single)
        print("Click corners TL→TR→BR→BL, then props. Press 'q' to quit.\n")

        while True:
            disp = view.display.copy()
            if view.mode == "corners":
                remaining = CORNER_LABELS[len(view.corners_2d):]
                msg = f"Next corner: {remaining[0] if remaining else '—'}"
            else:
                msg = f"Props: {len(results)}  |  'q' to finish"
            cv2.putText(disp, msg, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 230, 30), 2)
            cv2.imshow(win, disp)
            if cv2.waitKey(20) & 0xFF == ord("q"):
                break

        cv2.destroyAllWindows()
        print_results(results)
        return

    # ── Two-view mode ─────────────────────────────────────────────────────────
    print("Mode: two-view triangulation  (no depth assumption)\n")
    views: list[CameraView] = []
    for i, path in enumerate(args.images, 1):
        v = CameraView(path, K, D, label=f"View{i}")
        if not v.run_interactive():
            sys.exit(f"ERROR: Pose not solved for {path}. Aborting.")
        views.append(v)

    n1, n2 = len(views[0].prop_rays), len(views[1].prop_rays)
    if n1 == 0 or n2 == 0:
        sys.exit("ERROR: Need at least 1 prop clicked in each view.")
    if n1 != n2:
        print(
            f"WARNING: View1 has {n1} props, View2 has {n2}. "
            f"Triangulating first {min(n1, n2)} pairs."
        )

    n = min(n1, n2)
    results = []
    for i in range(n):
        s1, r1 = views[0].prop_rays[i]
        s2, r2 = views[1].prop_rays[i]
        pos, gap = closest_point_two_rays(s1, r1, s2, r2)
        results.append((pos, gap))

    print_results(results)


if __name__ == "__main__":
    main()