"""Our velocity from the small objects around us, one vote per object.

rf2o weighs every beam the same, so one big moving thing (a kayak, a boat)
can outvote every buoy in view. Here a buoy and a kayak each get one vote,
and the vote only counts when most objects agree.
"""

import math
from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
from statistics import median


def rotate(x: float, y: float, angle: float) -> tuple[float, float]:
    c, s = math.cos(angle), math.sin(angle)
    return c * x - s * y, s * x + c * y


class GyroHeading:
    """Heading [rad] integrated from the gyro's yaw rate, for any time it has seen."""

    def __init__(self, kept_s: float) -> None:
        self._kept_s = kept_s
        self._times: deque[float] = deque()
        self._headings: deque[float] = deque()
        self._last_rate = 0.0

    def add(self, t: float, yaw_rate: float) -> None:
        if self._times and t <= self._times[-1]:
            return
        heading = 0.0
        if self._times:
            dt = t - self._times[-1]
            heading = self._headings[-1] + 0.5 * (self._last_rate + yaw_rate) * dt
        self._times.append(t)
        self._headings.append(heading)
        self._last_rate = yaw_rate
        while self._times[-1] - self._times[0] > self._kept_s:
            self._times.popleft()
            self._headings.popleft()

    def at(self, t: float) -> float | None:
        if not self._times or not self._times[0] <= t <= self._times[-1]:
            return None
        i = bisect_left(self._times, t)
        if self._times[i] == t:
            return self._headings[i]
        t0, t1 = self._times[i - 1], self._times[i]
        h0, h1 = self._headings[i - 1], self._headings[i]
        return h0 + (h1 - h0) * (t - t0) / (t1 - t0)


@dataclass
class Vote:
    vx: float
    vy: float
    start: float
    end: float
    agreeing: int


def object_velocity(t0, p0, t1, p1, turned) -> tuple[float, float]:
    """Our velocity [m/s] in base_link, from a static object seen at p0 then p1 (base_link, m).

    turned is our heading change from t0 to t1. Measured across the mid-step heading.
    """
    back_x, back_y = rotate(*p1, turned)
    dx, dy = rotate(p0[0] - back_x, p0[1] - back_y, -turned / 2)
    dt = t1 - t0
    return dx / dt, dy / dt


class ObjectVote:
    """Follows objects from scan to scan and votes on our velocity once per scan."""

    def __init__(
        self,
        heading: GyroHeading,
        window: float,
        match_gate: float,
        min_agreeing: int,
        agree_speed: float,
    ) -> None:
        self._heading = heading
        self._window = window
        self._match_gate = match_gate
        self._min_agreeing = min_agreeing
        self._agree_speed = agree_speed
        self._tracks: dict[int, deque[tuple[float, tuple[float, float]]]] = {}
        self._next_id = 0

    def update(self, t: float, points: list[tuple[float, float]]) -> Vote | None:
        """points are this scan's object centres in base_link."""
        heading = self._heading.at(t)
        if heading is None:
            return None
        for track_id in [
            k for k, track in self._tracks.items() if t - track[-1][0] > self._window
        ]:
            del self._tracks[track_id]
        self._match(t, points, heading)

        votes = []
        for track in self._tracks.values():
            while t - track[0][0] > self._window:
                track.popleft()
            t0, p0 = track[0]
            t1, p1 = track[-1]
            start_heading = self._heading.at(t0)
            if t1 != t or t1 - t0 < self._window / 2 or start_heading is None:
                continue
            votes.append((t0, object_velocity(t0, p0, t1, p1, heading - start_heading)))
        return self._tally(t, votes)

    def _match(self, t, points, heading) -> None:
        # Buoys drop out of single scans as the laser rings cross them, so a
        # track continues from its last sighting, not only from the last scan.
        previous = {}
        for track_id, track in self._tracks.items():
            seen_heading = self._heading.at(track[-1][0])
            if seen_heading is not None:
                previous[track_id] = rotate(*track[-1][1], seen_heading - heading)

        # A static object barely moves between sightings once our turn is undone,
        # so a mutual nearest neighbour inside the gate is the same object.
        def nearest(p, candidates):
            best = min(
                candidates,
                key=lambda k: math.dist(p, candidates[k]),
                default=None,
            )
            if best is None or math.dist(p, candidates[best]) > self._match_gate:
                return None
            return best

        current = dict(enumerate(points))
        for i, p in current.items():
            track_id = nearest(p, previous)
            if track_id is not None and nearest(previous[track_id], current) == i:
                self._tracks[track_id].append((t, p))
            else:
                self._tracks[self._next_id] = deque([(t, p)])
                self._next_id += 1

    def _tally(self, t, votes) -> Vote | None:
        if len(votes) < self._min_agreeing:
            return None
        mid = (median(v[0] for _, v in votes), median(v[1] for _, v in votes))
        agreeing = [
            (t0, v) for t0, v in votes if math.dist(v, mid) <= self._agree_speed
        ]
        if len(agreeing) < self._min_agreeing or 2 * len(agreeing) <= len(votes):
            return None
        return Vote(
            vx=sum(v[0] for _, v in agreeing) / len(agreeing),
            vy=sum(v[1] for _, v in agreeing) / len(agreeing),
            start=min(t0 for t0, _ in agreeing),
            end=t,
            agreeing=len(agreeing),
        )
