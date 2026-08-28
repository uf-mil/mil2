"""How badly does torpedo board pose degrade with range and corner noise?

Answers a practical question for issue #521: the XFeat+homography approach was
observed to work "at medium-close range" -- this quantifies where the wall is.

We project the board at a known pose, jitter the corner pixels by a realistic
amount, recover the pose, and report mean error over many trials.

Run:  python3 src/subjugator/admission/test/noise_sweep.py
"""

import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "admission"))

from torpedo_pose import (
    board_object_points,
    default_camera_matrix,
    estimate_board_pose,
)

WIDTH, HEIGHT = 640, 360
HFOV_DEG = 80.0
TRIALS = 300

# Board tilted a realistic amount so rotation error is meaningful.
RVEC_TRUE = np.array([np.deg2rad(5.0), np.deg2rad(20.0), 0.0])

RANGES_M = (2.0, 3.0, 4.0, 6.0, 8.0)
NOISE_PX = (0.0, 0.5, 1.0, 2.0, 3.0)


def sweep(seed=0):
    rng = np.random.default_rng(seed)
    K = default_camera_matrix(WIDTH, HEIGHT, np.deg2rad(HFOV_DEG))
    obj = board_object_points()

    print(
        f"board {0.6}x{0.6} m | {WIDTH}x{HEIGHT} | hfov {HFOV_DEG} deg | {TRIALS} trials",
    )
    print(f"\n{'range':>7} {'noise':>7} {'range err':>11} {'rot err':>10} {'fail':>6}")
    print("-" * 46)

    for z in RANGES_M:
        for npx in NOISE_PX:
            tvec_true = np.array([0.0, 0.0, z])
            clean, _ = cv2.projectPoints(obj, RVEC_TRUE, tvec_true, K, np.zeros(5))
            clean = clean.reshape(-1, 2)

            range_errs, rot_errs, fails = [], [], 0
            for _ in range(TRIALS):
                corners = clean + rng.normal(0.0, npx, clean.shape)
                ok, rvec, tvec = estimate_board_pose(corners, K)
                if not ok:
                    fails += 1
                    continue
                range_errs.append(abs(tvec.ravel()[2] - z))
                rot_errs.append(
                    float(np.linalg.norm(np.rad2deg(rvec.ravel() - RVEC_TRUE))),
                )

            print(
                f"{z:6.1f}m {npx:6.1f}px {np.mean(range_errs) * 100:9.1f}cm "
                f"{np.mean(rot_errs):8.1f}deg {fails:6d}",
            )
        print()


if __name__ == "__main__":
    sweep()
