import cv2
import numpy as np

from utils import CAM_CONFIG_PATH, load_config


class Undistorter:
    # Loads camera calibration from config/cam.yaml and
    # undistorts each frame.

    def __init__(self, params_path=CAM_CONFIG_PATH, crop_to_roi=True):
        params = load_config(params_path)

        self.mtx = np.array(params["camera_matrix"])
        self.dist = np.array(params["dist_coeffs"])
        self.calibrated_width = params.get("calibrated_width")
        self.calibrated_height = params.get("calibrated_height")
        self.crop_to_roi = crop_to_roi

        self.mapx = None
        self.mapy = None
        self.roi = None

        self.intrinsics = None

    def _build_maps(self, w, h):
        mtx = self.mtx.copy()
        if (self.calibrated_width, self.calibrated_height) != (w, h):
            sx = w / self.calibrated_width
            sy = h / self.calibrated_height
            if abs(sx - sy) > 1e-3:
                print(
                    f"WARNING: aspect ratio differs from calibration "
                    f"({self.calibrated_width}x{self.calibrated_height} vs {w}x{h}).",
                )
            mtx[0] *= sx
            mtx[1] *= sy
            print(
                f"scaled calibration from {self.calibrated_width}x{self.calibrated_height} to {w}x{h}.",
            )

        newcammtx, roi = cv2.getOptimalNewCameraMatrix(
            mtx,
            self.dist,
            (w, h),
            alpha=1,
            newImgSize=(w, h),
        )
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(
            mtx,
            self.dist,
            None,
            newcammtx,
            (w, h),
            cv2.CV_32FC1,
        )
        self.roi = roi

        x, y, rw, rh = roi
        if self.crop_to_roi and (rw == 0 or rh == 0):
            print("valid-pixel ROI is empty. not cropping.")
            self.crop_to_roi = False

        cx = float(newcammtx[0, 2])
        cy = float(newcammtx[1, 2])
        out_w, out_h = w, h
        if self.crop_to_roi:
            cx -= x
            cy -= y
            out_w, out_h = rw, rh

        self.intrinsics = {
            "fx": float(newcammtx[0, 0]),
            "fy": float(newcammtx[1, 1]),
            "cx": cx,
            "cy": cy,
            "width": int(out_w),
            "height": int(out_h),
        }

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
