"""Where a lidar box lands in an image. Pure numpy, no ROS."""

from dataclasses import dataclass

import numpy as np

# Nearer than this a box is beside the lens rather than in front of it.
MIN_DEPTH = 0.5  # m

_SIGNS = np.array(
    [[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)],
)


@dataclass(frozen=True)
class Lens:
    """A rectified image: its 3x4 projection matrix and size in pixels."""

    P: np.ndarray
    width: int
    height: int

    @property
    def fx(self):
        return float(self.P[0, 0])

    def box(self, points):
        """
        The pixel box (left, top, right, bottom) that camera-frame points
        cover, clipped to the image, or None if it misses the image.

        Corners behind the lens are pulled up to MIN_DEPTH, so a box the
        camera is partway past still spreads across the side it is on.
        """
        if points[:, 2].mean() < MIN_DEPTH:
            return None
        depth = np.maximum(points[:, 2], MIN_DEPTH)
        uvw = np.c_[points[:, :2], depth, np.ones(len(points))] @ self.P.T
        uv = uvw[:, :2] / uvw[:, 2:3]
        left, top = np.maximum(uv.min(axis=0), 0.0)
        right, bottom = np.minimum(uv.max(axis=0), (self.width, self.height))
        if left >= right or top >= bottom:
            return None
        return float(left), float(top), float(right), float(bottom)


def corners(centre, size, rotation=None):
    """The eight corners of a box, one per row."""
    offsets = _SIGNS * np.asarray(size, dtype=float)
    if rotation is not None:
        offsets = offsets @ np.asarray(rotation).T
    return np.asarray(centre, dtype=float) + offsets


def unoccluded(spans, depths, max_hidden=0.5):
    """
    Indices of the horizontal spans that nearer ones hide no more than
    max_hidden of. A buoy hidden behind another would otherwise take the
    near one's detection.
    """
    kept, nearer = [], []
    for i in np.argsort(depths, kind="stable"):
        low, high = spans[i]
        if _covered(spans[i], nearer) <= max_hidden * (high - low):
            kept.append(int(i))
        nearer.append(spans[i])
    return kept


def _covered(span, others):
    low, high = span
    clipped = sorted(
        (max(a, low), min(b, high)) for a, b in others if a < high and b > low
    )
    total, reach = 0.0, low
    for a, b in clipped:
        a = max(a, reach)
        if b > a:
            total += b - a
            reach = b
    return total
