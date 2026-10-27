"""Sensor-only planar dock geometry and RGB beacon association. No world access."""

import math
from dataclasses import dataclass

import cv2
import numpy as np

# Avoid oversubscribing rendering and ROS executors on development laptops.
cv2.setNumThreads(1)


@dataclass
class Bay:
    stage: np.ndarray
    goal: np.ndarray
    inward: np.ndarray
    beacon: np.ndarray
    width: float
    entrance: float
    number: int

    @property
    def yaw(self):
        return math.atan2(self.inward[1], self.inward[0])


def color_centers(rgb):
    """Return a list of horizontal centroids per indicator color, including repeats."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = cv2.split(hsv)
    masks = {
        "red": (h < 10) | (h > 170),
        "green": (h > 40) & (h < 85),
    }
    centers = {}
    for color, hue in masks.items():
        mask = (hue & (s > 160) & (v > 70)).astype(np.uint8)
        count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
        blobs = [
            i
            for i in range(1, count)
            if stats[i, cv2.CC_STAT_AREA] >= 8 and stats[i, cv2.CC_STAT_HEIGHT] >= 3
        ]
        if blobs:
            centers[color] = sorted(float(centroids[i, 0]) for i in blobs)
    return centers


def lines(points):
    """Extract supported line segments using deterministic RANSAC and SVD."""
    remaining = points.copy()
    rng = np.random.default_rng(7)
    result = []
    for _ in range(14):
        if len(remaining) < 8:
            break
        best = np.zeros(len(remaining), dtype=bool)
        for _ in range(160):
            a, b = remaining[rng.choice(len(remaining), 2, replace=False)]
            d = b - a
            if np.linalg.norm(d) < 0.5:
                continue
            n = np.array([-d[1], d[0]]) / np.linalg.norm(d)
            mask = np.abs((remaining - a) @ n) < 0.055
            if mask.sum() > best.sum():
                best = mask
        if best.sum() < 8:
            break
        support = remaining[best]
        center = support.mean(axis=0)
        _, _, vt = np.linalg.svd(support - center, full_matrices=False)
        direction = vt[0]
        if np.ptp(support @ direction) > 0.7:
            result.append((center, direction, support))
        remaining = remaining[~best]
    return result


def ray(pixel, fx, cx, yaw):
    local = np.array([1.0, (cx - pixel) / fx])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]]) @ local


def detect_bay(points, pixels, fx, cx, camera, yaw):
    """Fit back wall + dividing fingers; intersect RGB rays with LiDAR wall.

    Requires one green and two red indicators at acquisition. No assumed world
    pose, bay order, wall range, or bay spacing. Dimensions only gate plausibility.
    """
    if (
        len(pixels.get("green", [])) != 1
        or len(pixels.get("red", [])) != 2
        or len(points) < 40
    ):
        return None
    _, unique = np.unique(np.round(points / 0.06), axis=0, return_index=True)
    points = points[unique]
    segments = lines(points)
    candidates = []
    for center, tangent, support in segments:
        inward = np.array([-tangent[1], tangent[0]])
        if (center - camera) @ inward < 0:
            inward = -inward
        lateral = np.array([-inward[1], inward[0]])
        wall = center @ inward
        if wall - camera @ inward < 2 or np.ptp(support @ lateral) < 3:
            continue
        hits = []
        # Green is first; red indicators remain distinct observations.
        for pixel in pixels["green"] + pixels["red"]:
            direction = ray(pixel, fx, cx, yaw)
            denom = direction @ inward
            if denom < 0.3:
                break
            hit = camera + direction * ((wall - camera @ inward) / denom)
            # Every beacon ray must meet actual returns on this wall, not a
            # mathematically extended line across an opening at the dock front.
            if np.min(np.linalg.norm(points - hit, axis=1)) > 0.4:
                break
            hits.append(hit)
        if len(hits) != 3:
            continue
        finger_segments = [
            (c, d, pts)
            for c, d, pts in segments
            if abs(d @ inward) > 0.98 and np.ptp(pts @ inward) > 1.0
        ]
        beacon_s = hits[0] @ lateral
        left = [
            (c @ lateral, pts)
            for c, _, pts in finger_segments
            if c @ lateral > beacon_s + 0.4
        ]
        right = [
            (c @ lateral, pts)
            for c, _, pts in finger_segments
            if c @ lateral < beacon_s - 0.4
        ]
        if not left or not right:
            continue
        hi, hp = min(left, key=lambda pair: pair[0])
        lo, lp = max(right, key=lambda pair: pair[0])
        width = hi - lo
        if not 1.5 < width < 5:
            continue
        entrance = min(np.min(hp @ inward), np.min(lp @ inward))
        if not 2 < wall - entrance < 8:
            continue
        # A side face can be hidden behind a finger. Its visible opposite
        # face would bias the bay center by half the finger thickness. Measure
        # the two front caps instead; their inner edges bound the open water.
        tip_s = np.sort((points @ lateral)[np.abs(points @ inward - entrance) < 0.15])
        groups = np.split(tip_s, np.flatnonzero(np.diff(tip_s) > 0.7) + 1)
        caps = [
            (float(g.min()), float(g.max()))
            for g in groups
            if len(g) >= 3 and 0.12 < np.ptp(g) < 0.8
        ]
        upper = [a for a, b in caps if a > beacon_s + 0.4]
        lower = [b for a, b in caps if b < beacon_s - 0.4]
        if not upper or not lower:
            continue
        hi, lo = min(upper), max(lower)
        width = hi - lo
        if not 1.5 < width < 5:
            continue
        middle = (hi + lo) / 2
        if abs(beacon_s - middle) > width * 0.3:
            continue
        # Facing inward, positive lateral is left. All three observed indicators
        # must be separated by plausible bay widths before assigning an ordinal.
        positions = sorted(float(hit @ lateral) for hit in hits)
        if any(not 1.5 < gap < 5 for gap in np.diff(positions)):
            continue
        number = 1 + sum(float(hit @ lateral) > float(beacon_s) for hit in hits[1:])
        stage = inward * (entrance - 2.0) + lateral * middle
        goal = inward * (wall - 1.25) + lateral * middle
        candidates.append(Bay(stage, goal, inward, hits[0], width, entrance, number))
    if not candidates:
        return None
    # The back wall is the deepest geometrically supported candidate.
    return max(candidates, key=lambda bay: np.linalg.norm(bay.beacon - camera))
