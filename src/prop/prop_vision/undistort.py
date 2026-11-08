import json

import cv2
from prop_vision.calibration import load


class Undistorter:
    # Loads camera calibration  from camera_params.json  and applies
    # undistorts each frame.

    def __init__(self, params_path="camera_params.json", crop_to_roi=True, rotate=0):
        with open(params_path) as f:
            params = json.load(f)
        if rotate not in (0, 180):
            raise ValueError(f"rotate must be 0 or 180, not {rotate}")

        self.params_path = params_path
        self.rotate = rotate
        self.calibrated_width = params.get("calibrated_width")
        self.calibrated_height = params.get("calibrated_height")
        self.crop_to_roi = crop_to_roi

        self.mapx = None
        self.mapy = None
        self.roi = None

        self.intrinsics = None

    def _build_maps(self, w, h):
        if (self.calibrated_width, self.calibrated_height) != (w, h):
            sx = w / self.calibrated_width
            sy = h / self.calibrated_height
            if abs(sx - sy) > 1e-3:
                print(
                    f"WARNING: aspect ratio differs from calibration "
                    f"({self.calibrated_width}x{self.calibrated_height} vs {w}x{h}).",
                )
            print(
                f"scaled calibration from {self.calibrated_width}x{self.calibrated_height} to {w}x{h}.",
            )
        mtx, dist = load(self.params_path, w, h, self.rotate)

        newcammtx, roi = cv2.getOptimalNewCameraMatrix(
            mtx,
            dist,
            (w, h),
            alpha=1,
            newImgSize=(w, h),
        )
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(
            mtx,
            dist,
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
