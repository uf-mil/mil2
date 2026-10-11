import math
import statistics
from collections import OrderedDict, deque

import config
from config import EDGE_MARGIN_PX, OBJECT_HEIGHTS_M, RANGE_SCALE

SMOOTH_WINDOW = getattr(config, "SMOOTH_WINDOW", 15)
MIN_BOX_HEIGHT_FRAC = getattr(config, "MIN_BOX_HEIGHT_FRAC", 0.016)
MAX_TRACKS = 50


def _block_reason(det, intrinsics):
    height_m = OBJECT_HEIGHTS_M.get(det["label"])
    if height_m is None:
        return f'no real height configured for class "{det["label"]}"'
    _, y1, _, y2 = det["box"]
    box_h_px = y2 - y1
    if box_h_px <= 0:
        return "empty box"
    if y1 <= EDGE_MARGIN_PX or y2 >= intrinsics["height"] - EDGE_MARGIN_PX:
        return "box touches frame edge (probably truncated)"
    if box_h_px < MIN_BOX_HEIGHT_FRAC * intrinsics["height"]:
        return f"box too small ({box_h_px}px) for a reliable range"
    return None


def locate(det, intrinsics, box_h_px=None):
    # returns bearing and estimated range (Z = fy * H / box_h_px) for one detection.
    fx, fy = intrinsics["fx"], intrinsics["fy"]
    cx, cy = intrinsics["cx"], intrinsics["cy"]

    x1, y1, x2, y2 = det["box"]
    u = (x1 + x2) / 2.0
    v = (y1 + y2) / 2.0
    if box_h_px is None:
        box_h_px = y2 - y1

    result = {
        "bearing_deg": math.degrees(math.atan2(u - cx, fx)),
        "z_m": None,
        "x_m": None,
        "y_m": None,
        "range_m": None,
        "range_note": _block_reason(det, intrinsics),
    }
    if result["range_note"]:
        return result

    z = RANGE_SCALE * fy * OBJECT_HEIGHTS_M[det["label"]] / box_h_px
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    result["z_m"] = z
    result["x_m"] = x
    result["y_m"] = y
    result["range_m"] = math.sqrt(x * x + y * y + z * z)
    return result


class Localizer:
    def __init__(self, window=SMOOTH_WINDOW):
        self.window = max(int(window), 1)
        self.history = OrderedDict()

    def locate(self, det, intrinsics):
        track_id = det.get("track_id")
        box_h_px = None
        if track_id is not None and _block_reason(det, intrinsics) is None:
            hist = self.history.setdefault(track_id, deque(maxlen=self.window))
            self.history.move_to_end(track_id)
            hist.append(det["box"][3] - det["box"][1])
            box_h_px = statistics.median(hist)
            while len(self.history) > MAX_TRACKS:
                self.history.popitem(last=False)
        return locate(det, intrinsics, box_h_px)
