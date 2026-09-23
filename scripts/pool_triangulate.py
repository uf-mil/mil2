#!/usr/bin/env python3
"""
Localize underwater competition elements from a tripod camera outside the pool.

Single-view mode  (1 image):
  Shoot a ray from camera -> refract at water surface -> intersect depth plane.
  Requires --depth assumption (default = pool floor).

Two-view mode  (2 images, recommended):
  Shoot refracted rays from both camera positions; find closest-point between
  the pair. No depth assumption needed -- depth is solved by geometry.

Pool world frame (meters):
  Origin = Top-Left corner at water surface
  +X     = long axis  (50 m)
  +Y     = short axis (22.86 m)
  +Z     = up  (water surface z=0, pool floor z=-2.1336)

Corner click order: Top-Left -> Top-Right -> Bottom-Right -> Bottom-Left
  ("Top" = far end relative to your camera vantage point)

Usage:
  # two-view (best):
  python pool_triangulate.py pos1.jpg pos2.jpg [--calib cal.npz]

  # single-view fallback:
  python pool_triangulate.py pos1.jpg [--calib cal.npz] [--depth 2.1336]

  --calib  .npz with keys 'camera_matrix' and 'dist_coeffs'
  --fov    horizontal FOV in degrees if no --calib (default 70)
  --depth  prop depth below surface in metres for single-view mode

Click sequence per image:
  1. Corners: TL -> TR -> BR -> BL  (auto-advances after 4th click)
  2. Props:   click each underwater prop in a consistent order
  3. Press 's' to switch to start mode
  4. Click the sub's start gate / entry point in the image
  5. Press 'n' (next image) or 'q' (quit / finish)

Output:
  Pool frame  -- absolute position in pool coordinates (origin = TL corner)
  Odom frame  -- displacement from sub start; use directly as move_rel waypoints
"""

from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np

# -- Pool constants ------------------------------------------------------------
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


# -- Geometry -----------------------------------------------------------------
def snell_refract(
    d: np.ndarray,
    normal: np.ndarray,
    n1: float,
    n2: float,
) -> np.ndarray | None:
    """Snell's law vector form. Returns refracted unit vector or None for TIR."""
    r = n1 / n2
    cos_i = float(-np.dot(d, normal))
    sin2_t = r**2 * (1.0 - cos_i**2)
    if sin2_t > 1.0:
        return None
    cos_t = float(np.sqrt(max(0.0, 1.0 - sin2_t)))
    out = r * d + (r * cos_i - cos_t) * normal
    return out / np.linalg.norm(out)


def ray_z_intersect(
    origin: np.ndarray,
    direction: np.ndarray,
    z: float,
) -> tuple[np.ndarray, float] | None:
    """Intersect ray with horizontal plane z=const. Returns (point, t) or None."""
    dz = direction[2]
    if abs(dz) < 1e-10:
        return None
    t = (z - origin[2]) / dz
    if t < 0:
        return None
    return origin + t * direction, t


def closest_point_two_rays(
    o1: np.ndarray,
    d1: np.ndarray,
    o2: np.ndarray,
    d2: np.ndarray,
) -> tuple[np.ndarray, float]:
    """
    Midpoint of shortest segment between two 3-D lines.
    Returns (midpoint, gap) -- gap > 0 means reprojection error.
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
    return (p1 + p2) / 2.0, float(np.linalg.norm(p1 - p2))


# -- Per-image view -----------------------------------------------------------
class CameraView:
    """
    Handles one image: PnP pose from pool corners, prop rays, sub start position.

    Modes (in order): "corners" -> "props" -> "start"

    start_surface_pt: pool-frame XY of the sub's start position (z=0).
                      Computed by intersecting the camera ray with z=0 directly
                      (no refraction) since the start gate is a surface feature.
    """

    def __init__(self, image_path: str, K: np.ndarray, D: np.ndarray, label: str = ""):
        img = cv2.imread(image_path)
        if img is None:
            sys.exit(f"ERROR: Cannot read image: {image_path}")
        self.image = img
        self.display = img.copy()
        self.K = K
        self.D = D
        self.label = label

        self.corners_2d: list[list[float]] = []
        self.R: np.ndarray | None = None
        self.t: np.ndarray | None = None
        self.mode = "corners"

        self.prop_rays: list[tuple[np.ndarray, np.ndarray]] = []
        self.start_surface_pt: np.ndarray | None = None

    # -- Pose -----------------------------------------------------------------
    def cam_pos(self) -> np.ndarray:
        return -(self.R.T @ self.t)

    def _solve_pose(self) -> None:
        pts2d = np.array(self.corners_2d, dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            POOL_CORNERS_3D,
            pts2d,
            self.K,
            self.D,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            print("ERROR: solvePnP failed -- verify corner click order TL->TR->BR->BL.")
            self.corners_2d = []  # reset so user can re-click all 4 corners
            return
        self.R, _ = cv2.Rodrigues(rvec)
        self.t = tvec.flatten()
        cp = self.cam_pos()
        print(f"  [{self.label}] Camera: X={cp[0]:.3f}  Y={cp[1]:.3f}  Z={cp[2]:.3f} m")
        self.mode = "props"
        print(
            f"  [{self.label}] Pose solved. Click props, then press 's' to mark start.\n",
        )

    # -- Ray helpers ----------------------------------------------------------
    def _pixel_ray(self, x: int, y: int) -> np.ndarray:
        pt = cv2.undistortPoints(
            np.array([[[float(x), float(y)]]], dtype=np.float64),
            self.K,
            self.D,
        )[0][0]
        ray_cam = np.array([pt[0], pt[1], 1.0])
        ray_cam /= np.linalg.norm(ray_cam)
        ray_world = self.R.T @ ray_cam
        return ray_world / np.linalg.norm(ray_world)

    def compute_refracted_ray(
        self,
        x: int,
        y: int,
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """Returns (surface_point, refracted_direction) or None."""
        cam = self.cam_pos()
        ray = self._pixel_ray(x, y)
        result = ray_z_intersect(cam, ray, 0.0)
        if result is None:
            print("  WARNING: ray does not intersect water surface.")
            return None
        surf_pt, _ = result
        refracted = snell_refract(ray, np.array([0.0, 0.0, 1.0]), N_AIR, N_WATER)
        if refracted is None:
            print("  WARNING: TIR (unexpected air->water).")
            return None
        return surf_pt, refracted

    def compute_surface_point(self, x: int, y: int) -> np.ndarray | None:
        """Camera ray -> water surface (z=0), no refraction. For surface features."""
        cam = self.cam_pos()
        ray = self._pixel_ray(x, y)
        result = ray_z_intersect(cam, ray, 0.0)
        if result is None:
            print("  WARNING: ray does not hit water surface.")
            return None
        return result[0]

    def localize_single(self, x: int, y: int, depth: float) -> np.ndarray | None:
        result = self.compute_refracted_ray(x, y)
        if result is None:
            return None
        surf_pt, refracted = result
        hit = ray_z_intersect(surf_pt, refracted, -depth)
        if hit is None:
            print("  WARNING: refracted ray does not reach target depth.")
            return None
        return hit[0]

    # -- UI -------------------------------------------------------------------
    def _on_click(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.mode == "corners":
            idx = len(self.corners_2d)
            self.corners_2d.append([float(x), float(y)])
            lbl = CORNER_LABELS[idx]
            cv2.circle(self.display, (x, y), 6, (0, 220, 0), -1)
            cv2.putText(
                self.display,
                lbl,
                (x + 9, y + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 220, 0),
                2,
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
                self.display,
                f"P{n}",
                (x + 9, y + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 80, 255),
                2,
            )
            print(f"  [{self.label}] Prop {n} at pixel ({x}, {y})")

        elif self.mode == "start":
            pt = self.compute_surface_point(x, y)
            if pt is None:
                return
            self.start_surface_pt = pt
            cv2.circle(self.display, (x, y), 8, (0, 220, 220), -1)
            cv2.putText(
                self.display,
                "START",
                (x + 9, y + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 220, 220),
                2,
            )
            print(
                f"  [{self.label}] Start: X={pt[0]:.3f}  Y={pt[1]:.3f} m (pool frame)",
            )

    def run_interactive(self) -> bool:
        """Open interactive window. Returns True if pose was solved."""
        win = f"View: {self.label}"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(win, self._on_click)

        h, w = self.image.shape[:2]
        print(f"\n[{self.label}] Image: {w}x{h}")
        print(f"[{self.label}] 1. Click corners TL->TR->BR->BL")
        print(f"[{self.label}] 2. Click props in order")
        print(f"[{self.label}] 3. Press 's' -> click sub start gate")
        print(f"[{self.label}] 4. Press 'n' to finish this view\n")

        status = {
            "corners": lambda: f"Next corner: {CORNER_LABELS[len(self.corners_2d)]}",
            "props": lambda: f"Props: {len(self.prop_rays)}  |  's'=mark start  'n'=done",
            "start": lambda: "Click sub start position (start gate / entry point)",
        }

        while True:
            disp = self.display.copy()
            msg = f"[{self.label}] {status.get(self.mode, lambda: '')()}"
            cv2.putText(
                disp,
                msg,
                (12, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (255, 230, 30),
                2,
            )
            cv2.imshow(win, disp)
            key = cv2.waitKey(20) & 0xFF
            if key == ord("s") and self.mode == "props":
                self.mode = "start"
                print(f"  [{self.label}] Start mode: click the sub's entry point.")
            elif key in (ord("n"), ord("q"), 13):
                break

        cv2.destroyWindow(win)
        return self.R is not None


# -- Output -------------------------------------------------------------------
def print_results(
    prop_pool_positions: list[tuple[np.ndarray, float | None]],
    start_pool_pos: np.ndarray | None,
) -> None:
    print("\n-- Pool frame (origin = TL corner) " + "-" * 30)
    if start_pool_pos is not None:
        print(
            f"  Start ref: X={start_pool_pos[0]:.3f} m"
            f"  Y={start_pool_pos[1]:.3f} m  Z=0.000 m",
        )
    for i, (pos, gap) in enumerate(prop_pool_positions, 1):
        gap_str = f"  ray-gap={gap * 100:.1f} cm" if gap is not None else ""
        print(
            f"  Prop {i:2d}:   X={pos[0]:.3f} m  Y={pos[1]:.3f} m"
            f"  Z={pos[2]:.3f} m{gap_str}",
        )

    if start_pool_pos is None:
        print("\n  (no start position marked -- skipping odom frame output)")
        return

    print(
        "\n-- Odom frame waypoints (relative to sub start, use with move_rel) "
        + "-" * 10,
    )
    for i, (pos, _) in enumerate(prop_pool_positions, 1):
        dx = pos[0] - start_pool_pos[0]
        dy = pos[1] - start_pool_pos[1]
        dz = pos[2]  # start is at surface z=0; sub starts at odom z=0
        print(f"  Prop {i:2d}:   dx={dx:+.3f} m  dy={dy:+.3f} m  dz={dz:+.3f} m")
    print()


# -- Entry point --------------------------------------------------------------
def approximate_K(image_shape: tuple[int, ...], fov_deg: float) -> np.ndarray:
    h, w = image_shape[:2]
    f = w / (2.0 * np.tan(np.radians(fov_deg / 2.0)))
    return np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1.0]], dtype=np.float64)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Localize pool props via PnP + Snell refraction. "
        "Pass 2 images for depth-free two-view mode.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("images", nargs="+", metavar="IMAGE", help="1 or 2 pool images")
    ap.add_argument("--calib", help=".npz with keys 'camera_matrix' and 'dist_coeffs'")
    ap.add_argument(
        "--fov",
        type=float,
        default=70.0,
        help="Horizontal FOV (deg) if no --calib",
    )
    ap.add_argument(
        "--depth",
        type=float,
        default=POOL_DEPTH,
        help=f"Prop depth (m) for single-view mode (default {POOL_DEPTH:.4f} = floor)",
    )
    args = ap.parse_args()

    if len(args.images) > 2:
        ap.error("Pass at most 2 images.")

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
            f"WARNING: No calibration file -- using approximate pinhole (FOV={args.fov} deg).\n"
            "         Calibrate with cv2.calibrateCamera() for better accuracy.\n",
        )

    # -- Single-view mode -----------------------------------------------------
    if len(args.images) == 1:
        print(f"Mode: single-view  (prop depth = {args.depth:.4f} m)\n")
        view = CameraView(args.images[0], K, D, label="View1")

        win = "View: View1"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

        prop_results: list[tuple[np.ndarray, float | None]] = []

        def on_click_single(
            event: int,
            x: int,
            y: int,
            flags: int,
            param: object,
        ) -> None:
            if event != cv2.EVENT_LBUTTONDOWN:
                return
            if view.mode == "corners":
                idx = len(view.corners_2d)
                view.corners_2d.append([float(x), float(y)])
                lbl = CORNER_LABELS[idx]
                cv2.circle(view.display, (x, y), 6, (0, 220, 0), -1)
                cv2.putText(
                    view.display,
                    lbl,
                    (x + 9, y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 220, 0),
                    2,
                )
                print(f"  Corner {lbl}: ({x}, {y})")
                if len(view.corners_2d) == 4:
                    view._solve_pose()
            elif view.mode == "props":
                pos = view.localize_single(x, y, args.depth)
                if pos is None:
                    return
                prop_results.append((pos, None))
                n = len(prop_results)
                lbl = f"P{n} ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})m"
                cv2.circle(view.display, (x, y), 6, (0, 80, 255), -1)
                cv2.putText(
                    view.display,
                    lbl,
                    (x + 9, y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 80, 255),
                    2,
                )
                print(f"  Prop {n}: X={pos[0]:.3f}  Y={pos[1]:.3f}  Z={pos[2]:.3f} m")
            elif view.mode == "start":
                pt = view.compute_surface_point(x, y)
                if pt is None:
                    return
                view.start_surface_pt = pt
                cv2.circle(view.display, (x, y), 8, (0, 220, 220), -1)
                cv2.putText(
                    view.display,
                    "START",
                    (x + 9, y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 220, 220),
                    2,
                )
                print(f"  Start: X={pt[0]:.3f}  Y={pt[1]:.3f} m (pool frame)")

        cv2.setMouseCallback(win, on_click_single)
        print("1. Click corners TL->TR->BR->BL")
        print("2. Click props")
        print("3. Press 's' -> click sub start gate")
        print("4. Press 'q' to finish\n")

        status = {
            "corners": lambda: f"Next corner: {CORNER_LABELS[len(view.corners_2d)]}",
            "props": lambda: f"Props: {len(prop_results)}  |  's'=mark start  'q'=done",
            "start": lambda: "Click sub start position (start gate / entry point)",
        }

        while True:
            disp = view.display.copy()
            msg = status.get(view.mode, lambda: "")()
            cv2.putText(
                disp,
                msg,
                (12, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 230, 30),
                2,
            )
            cv2.imshow(win, disp)
            key = cv2.waitKey(20) & 0xFF
            if key == ord("s") and view.mode == "props":
                view.mode = "start"
                print("  Start mode: click the sub's entry point.")
            elif key == ord("q"):
                break

        cv2.destroyAllWindows()
        print_results(prop_results, view.start_surface_pt)
        return

    # -- Two-view mode --------------------------------------------------------
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
            f"WARNING: View1 has {n1} props, View2 has {n2}."
            f" Using first {min(n1, n2)} pairs.",
        )

    n = min(n1, n2)
    prop_results = []
    for i in range(n):
        s1, r1 = views[0].prop_rays[i]
        s2, r2 = views[1].prop_rays[i]
        pos, gap = closest_point_two_rays(s1, r1, s2, r2)
        prop_results.append((pos, gap))

    # Average start positions from both views
    start_pos: np.ndarray | None = None
    sp0, sp1 = views[0].start_surface_pt, views[1].start_surface_pt
    if sp0 is not None and sp1 is not None:
        start_pos = (sp0 + sp1) / 2.0
        diff = float(np.linalg.norm(sp0[:2] - sp1[:2]))
        if diff > 0.5:
            print(f"WARNING: Start position disagreement between views: {diff:.2f} m.")
    elif sp0 is not None:
        start_pos = sp0
    elif sp1 is not None:
        start_pos = sp1

    print_results(prop_results, start_pos)


if __name__ == "__main__":
    main()
