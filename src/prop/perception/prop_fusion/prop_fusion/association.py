"""
Where a lidar track lands in the image, which detection it is, and what it has
been called. No ROS here, so it tests on its own.
"""

import numpy as np
from scipy.optimize import linear_sum_assignment

# Pairs outside the gate cost this, so the solver takes one only when it must,
# and those are thrown out afterwards.
_GATED = 1e9

# Nearer than this a box is beside the lens rather than in front of it.
MIN_DEPTH = 0.5  # m

_SIGNS = np.array(
    [[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)],
)


def corners(centre, size, rotation):
    """The eight corners of a box, one per row."""
    return np.asarray(centre) + (_SIGNS * np.asarray(size)) @ np.asarray(rotation).T


def project(points, P, width, height):
    """
    The pixel box (left, top, right, bottom) that camera-frame points cover,
    clipped to the image, or None if it misses the image.

    Corners behind the lens are pulled up to MIN_DEPTH, so a box the camera is
    partway past still spreads across the side of the frame it is on.
    """
    if points[:, 2].mean() < MIN_DEPTH:
        return None
    clamped = np.c_[points[:, :2], np.maximum(points[:, 2], MIN_DEPTH)]
    uvw = np.c_[clamped, np.ones(len(clamped))] @ P.T
    uv = uvw[:, :2] / uvw[:, 2:3]
    left, top = np.maximum(uv.min(axis=0), 0.0)
    right, bottom = np.minimum(uv.max(axis=0), (width, height))
    if left >= right or top >= bottom:
        return None
    return float(left), float(top), float(right), float(bottom)


def _covered(span, others):
    """How much of span the union of others covers."""
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


def unoccluded(spans, depths, max_hidden=0.5):
    """
    Indices of the horizontal spans that nearer ones hide no more than
    max_hidden of.

    A buoy directly behind another is close enough in bearing to take the
    near one's detection, so it sits this frame out instead.
    """
    kept, nearer = [], []
    for i in np.argsort(depths, kind="stable"):
        low, high = spans[i]
        if _covered(spans[i], nearer) <= max_hidden * (high - low):
            kept.append(int(i))
        nearer.append(spans[i])
    return kept


def associate(track_spans, detection_spans, tolerance):
    """
    Pair tracks with detections one to one, by horizontal position alone.

    Bearing is what a hand-measured mount gets closest to right; height is at
    the mercy of its pitch and of where the waterline cuts the buoy. A pair is
    allowed when the gap between the spans is within tolerance pixels, and
    costs the distance between their centres.
    """
    if not track_spans or not detection_spans:
        return []
    cost = np.full((len(track_spans), len(detection_spans)), _GATED)
    for i, (t_low, t_high) in enumerate(track_spans):
        for j, (d_low, d_high) in enumerate(detection_spans):
            if max(d_low - t_high, t_low - d_high, 0.0) <= tolerance:
                cost[i, j] = abs((t_low + t_high) - (d_low + d_high)) / 2.0
    rows, cols = linear_sum_assignment(cost)
    return [(int(i), int(j)) for i, j in zip(rows, cols) if cost[i, j] < _GATED]


class Votes:
    """Per lidar track, the confidence each class has been detected with."""

    def __init__(self, margin):
        self.margin = margin
        self._tallies = {}

    def add(self, track_id, label, score):
        tally = self._tallies.setdefault(track_id, {})
        tally[label] = tally.get(label, 0.0) + score

    def label(self, track_id):
        """The leading class once it is margin ahead of the next, else ''."""
        tally = self._tallies.get(track_id)
        if not tally:
            return ""
        leader, *rest = sorted(tally, key=tally.get, reverse=True)
        runner_up = tally[rest[0]] if rest else 0.0
        return leader if tally[leader] - runner_up >= self.margin else ""

    def keep(self, track_ids):
        """Forget tracks pcl_tracker has dropped. It never reuses an ID."""
        self._tallies = {i: t for i, t in self._tallies.items() if i in track_ids}
