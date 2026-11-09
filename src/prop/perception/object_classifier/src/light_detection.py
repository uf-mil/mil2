from collections import Counter, deque
from dataclasses import asdict, dataclass
from typing import Optional

import cv2
import numpy as np

# ---------- settings ----------
# HSV ranges: (lowest H, S, V) to (highest H, S, V). OpenCV hue is 0-179.
COLOR_RANGES = {
    "red": [((0, 180, 225), (6, 255, 255)), ((174, 180, 225), (179, 255, 255))],
    "green": [((45, 150, 225), (80, 255, 255))],
    "blue": [((100, 150, 225), (128, 255, 255))],
}

ROLE = {
    ("red", "flashing"): "Flashing Red",
    ("green", "flashing"): "Flashing Green",
    ("blue", "flashing"): "Flashing Blue",
    ("blue", "steady"): "Steady Blue",
}


@dataclass
class Result:
    label: str  # human-readable answer
    pattern: str  # "analyzing", "flashing", "steady", "off", or "unsure"
    color: Optional[str]  # "red", "green", "blue", or None


class BuoyClassifier:
    def __init__(
        self,
        window=4.0,  # seconds of history to judge from
        min_watch=2.5,  # must watch this long before deciding (a flash cycle is 2 s)
        min_pixels=30,  # colored pixels needed in the box to call it "lit"
        max_fraction=0.2,  # a color covering more than this much of the box is background, not a light
        min_run=0.25,  # on/off periods shorter than this (sec) are treated as glitches
        steady_fraction=0.9,  # lit this much of the window with no flashing = steady
        off_fraction=0.05,
    ):  # lit less than this = never lit = off
        self.window = window
        self.min_watch = min_watch
        self.min_pixels = min_pixels
        self.max_fraction = max_fraction
        self.min_run = min_run
        self.steady_fraction = steady_fraction
        self.off_fraction = off_fraction
        self.history = deque()  # (timestamp, color or None) for each frame
        self.last_color = None  # what the latest frame looked like (handy for tuning)

    def reset(self):
        self.history.clear()
        self.last_color = None

    # ----- look at one frame -----
    def _lit_color(self, crop):
        """Return the color of the light in this crop, or None if it's dark."""
        blurred = cv2.GaussianBlur(crop, (3, 3), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        area = hsv.shape[0] * hsv.shape[1]
        counts = {}
        for color, ranges in COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for low, high in ranges:
                mask |= cv2.inRange(hsv, np.array(low), np.array(high))
            n = int(cv2.countNonZero(mask))
            if (
                n > self.max_fraction * area
            ):  # too much of the box: a face or wall, not an LED
                n = 0
            counts[color] = n
        best = max(counts, key=counts.get)  # the color with the most pixels wins
        return best if counts[best] >= self.min_pixels else None

    # ----- add a frame, get an answer -----
    def update(self, crop, t):
        color = self._lit_color(crop)
        self.last_color = color
        self.history.append((t, color))
        while (
            t - self.history[0][0] > self.window
        ):  # forget frames older than the window
            self.history.popleft()
        return self._decide()

    # ----- decide from the history -----
    def _decide(self):
        first_t = self.history[0][0]
        last_t = self.history[-1][0]
        total = last_t - first_t
        if total < self.min_watch:
            return Result("analyzing...", "analyzing", None)

        # break the history into runs of "lit" / "dark": [is_lit, start_time, end_time]
        runs = []
        for t, color in self.history:
            lit = color is not None
            if runs and runs[-1][0] == lit:
                runs[-1][2] = t
            else:
                if runs:
                    runs[-1][2] = t  # previous run ends where this one starts
                runs.append([lit, t, t])

        lit_time = sum(r[2] - r[1] for r in runs if r[0])
        lit_fraction = lit_time / total

        # count on<->off switches, ignoring glitchy short runs in the middle
        kept = [
            r
            for i, r in enumerate(runs)
            if i == 0 or i == len(runs) - 1 or (r[2] - r[1]) >= self.min_run
        ]
        merged = []
        for r in kept:
            if merged and merged[-1][0] == r[0]:
                merged[-1][2] = r[2]
            else:
                merged.append(list(r))
        transitions = len(merged) - 1

        # the light's color = the most common color among lit frames
        votes = Counter(c for _, c in self.history if c is not None)
        color = votes.most_common(1)[0][0] if votes else None

        if lit_fraction < self.off_fraction:
            return Result("BLACK (obstacle)", "off", None)
        if transitions >= 2:
            pattern = "flashing"
        elif lit_fraction >= self.steady_fraction:
            pattern = "steady"
        else:
            return Result("unsure", "unsure", color)

        label = ROLE.get((color, pattern), f"{color} {pattern}")
        return Result(label, pattern, color)


class Light_Detector:
    # Actual class that will be used in the pipeline
    def __init__(self, padding_above=0.25, max_gap=0.5):
        self.padding_above = padding_above  # how far above buoy to look for light
        self.max_gap = max_gap  # seconds
        self.classifiers = {}

    def detect_light(self, image, detections, t):
        recent_classifiers = {}

        for track_id, saved_data in self.classifiers.items():
            classifier, last_seen = saved_data
            elapsed = t - last_seen

            if elapsed > 0 and elapsed <= self.max_gap:
                recent_classifiers[track_id] = (classifier, last_seen)

        self.classifiers = recent_classifiers

        height, width = image.shape[:2]
        lights = []
        for det in detections:
            x1, y1, x2, y2 = det["box"]
            if x2 <= x1 or y2 <= y1:
                continue
            y1 -= round((y2 - y1) * self.padding_above)
            x1, x2 = max(0, int(x1)), min(width, int(x2))
            y1, y2 = max(0, int(y1)), min(height, int(y2))
            if x2 <= x1 or y2 <= y1:
                continue

            track_id = det["track_id"]
            if track_id not in self.classifiers:
                self.classifiers[track_id] = (BuoyClassifier(), t)
            classifier, _ = self.classifiers[track_id]
            result = classifier.update(image[y1:y2, x1:x2], t)
            self.classifiers[track_id] = (classifier, t)
            lights.append(
                {
                    "track_id": track_id,
                    "box": (x1, y1, x2, y2),
                    **asdict(result),
                },
            )
        return lights
