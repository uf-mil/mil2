"""Which detection is which track, and what each track has been called."""

import numpy as np
from scipy.optimize import linear_sum_assignment

_GATED = 1e9


def associate(track_spans, detection_spans, tolerance):
    """
    Pair tracks with detections one to one by horizontal position alone:
    bearing is what a hand-measured mount gets closest to right.

    Spans are (left, right) in pixels. A pair is allowed when the gap between
    them is within tolerance pixels, and costs the distance between centres.
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
    """
    Per track, the confidence each class has been detected with
    ex:
    track_id 4: {"red_buoy": 2.7, "green_buoy": 0.8}
    """

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
        """Forget tracks the tracker has dropped. It never reuses an ID."""
        self._tallies = {i: t for i, t in self._tallies.items() if i in track_ids}
