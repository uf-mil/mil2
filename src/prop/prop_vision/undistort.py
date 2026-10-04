import json

import cv2
import numpy as np


class Undistorter:
    # Loads camera calibration  from camera_params.json  and applies
    # undistorts each frame.

    def __init__(self, params_path="camera_params.json", crop_to_roi=True):
        with open(params_path) as f:
            params = json.load(f)

        self.mtx = np.array(params["camera_matrix"])
        self.dist = np.array(params["dist_coeffs"])
        self.calibrated_width = params.get("calibrated_width")
        self.calibrated_height = params.get("calibrated_height")
        self.crop_to_roi = crop_to_roi

        self.mapx = None
        self.mapy = None
        self.roi = None

    def _build_maps(self, w, h):
        if (self.calibrated_width, self.calibrated_height) != (w, h):
            print(
                f" calibration was done at "
                f"{self.calibrated_width}x{self.calibrated_height}, "
                f"but frames are {w}x{h}.",
            )

        newcammtx, roi = cv2.getOptimalNewCameraMatrix(
            self.mtx,
            self.dist,
            (w, h),
            alpha=1,
            newImgSize=(w, h),
        )
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(
            self.mtx,
            self.dist,
            None,
            newcammtx,
            (w, h),
            cv2.CV_32FC1,
        )
        self.roi = roi

    def apply(self, frame):
        # returns the undistorted frame.
        h, w = frame.shape[:2]

        if self.mapx is None:
            self._build_maps(w, h)

        undistorted = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)

        if self.crop_to_roi:
            x, y, rw, rh = self.roi
            undistorted = undistorted[y : y + rh, x : x + rw]

        return undistorted
