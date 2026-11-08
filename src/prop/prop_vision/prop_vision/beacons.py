"""
Which Task 1 light a buoy shows, in RoboCommand's names: OFF, FLASHING_RED,
FLASHING_GREEN, FLASHING_BLUE or STEADY_BLUE. The lights flash 1 s on, 1 s off.

The classifier is Noah's, from noah-light-detection. A frame's colour is the LED
colour with the most bright, saturated pixels in the window where the beacon
should be, and the last few seconds of those colours say whether the light
flashes (turns on and off), is steady (always lit) or is off (never lit).

Here the history belongs to an object_map id rather than a camera track, so a
buoy keeps what it showed when the camera loses it. Each time the pattern is
decided it is one vote for that buoy's state.
"""

from collections import Counter, deque

import cv2
import numpy as np

# HSV ranges, lowest (H, S, V) to highest. OpenCV hue runs 0-179.
COLOR_RANGES = {
    "red": [((0, 180, 225), (6, 255, 255)), ((174, 180, 225), (179, 255, 255))],
    "green": [((45, 150, 225), (80, 255, 255))],
    "blue": [((100, 150, 225), (128, 255, 255))],
}

OFF = "OFF"
STATES = {
    ("red", "flashing"): "FLASHING_RED",
    ("green", "flashing"): "FLASHING_GREEN",
    ("blue", "flashing"): "FLASHING_BLUE",
    ("blue", "steady"): "STEADY_BLUE",
}


def lit_color(crop, min_pixels=4, max_fraction=0.2):
    """
    The colour of the light in this crop, or None if it is dark. A colour that
    covers more than max_fraction of the crop is a wall or a panel, not an LED.

    Unlike Noah's, the crop is not blurred first: a beacon ten metres off is a
    few pixels wide, and blurring it into its background drops it below the
    brightness floor. min_pixels and the on-off pattern take care of noise.
    """
    if crop.size == 0:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    area = hsv.shape[0] * hsv.shape[1]
    counts = {}
    for color, ranges in COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for low, high in ranges:
            mask |= cv2.inRange(hsv, np.array(low), np.array(high))
        count = int(cv2.countNonZero(mask))
        counts[color] = 0 if count > max_fraction * area else count
    best = max(counts, key=counts.get)
    return best if counts[best] >= min_pixels else None


def pattern(history, min_watch=2.5, min_run=0.25, steady=0.9, off=0.05):
    """
    The state a history of (time, colour or None) shows, or None if it does not
    show one yet. Runs shorter than min_run in the middle are glitches.
    """
    total = history[-1][0] - history[0][0]
    if total < min_watch:
        return None

    runs = []  # [lit, start, end]
    for t, color in history:
        lit = color is not None
        if runs and runs[-1][0] == lit:
            runs[-1][2] = t
        else:
            if runs:
                runs[-1][2] = t
            runs.append([lit, t, t])
    lit_fraction = sum(end - start for lit, start, end in runs if lit) / total
    if lit_fraction < off:
        return OFF

    kept = [
        run
        for i, run in enumerate(runs)
        if i in (0, len(runs) - 1) or run[2] - run[1] >= min_run
    ]
    merged = []
    for run in kept:
        if merged and merged[-1][0] == run[0]:
            merged[-1][2] = run[2]
        else:
            merged.append(list(run))

    color = Counter(c for _, c in history if c is not None).most_common(1)[0][0]
    if len(merged) - 1 >= 2:
        return STATES.get((color, "flashing"))
    if lit_fraction >= steady:
        return STATES.get((color, "steady"))
    return None


class Beacons:
    """Each object's recent colours, and its votes for a state."""

    def __init__(
        self,
        window=4.0,
        min_watch=2.5,
        max_gap=0.5,
        min_run=0.25,
        steady=0.9,
        off=0.05,
        min_votes=15,
    ):
        self.window = window
        self.max_gap = max_gap
        self.min_votes = min_votes
        self.rules = {
            "min_watch": min_watch,
            "min_run": min_run,
            "steady": steady,
            "off": off,
        }
        self.history = {}
        self.votes = {}

    def add(self, object_id, color, t, close=True):
        """
        One frame's colour for an object. A gap longer than max_gap starts its
        history again. Not seeing a light only counts when close: from further
        away a lit beacon may be too small to show.
        """
        history = self.history.setdefault(object_id, deque())
        if history and t - history[-1][0] > self.max_gap:
            history.clear()
        history.append((t, color))
        while t - history[0][0] > self.window:
            history.popleft()
        state = pattern(history, **self.rules)
        if state is None or (state == OFF and not close):
            return None
        self.votes.setdefault(object_id, Counter())[state] += 1
        return state

    def state(self, object_id):
        """The state with the most votes once it has min_votes, else None."""
        votes = self.votes.get(object_id)
        if not votes:
            return None
        state, count = votes.most_common(1)[0]
        return state if count >= self.min_votes else None

    def confidence(self, object_id):
        votes = self.votes.get(object_id)
        if not votes:
            return 0.0
        return votes.most_common(1)[0][1] / sum(votes.values())

    def forget(self, keep):
        """Drop objects object_map no longer has."""
        for table in (self.history, self.votes):
            for object_id in [i for i in table if i not in keep]:
                del table[object_id]
