"""Pure aggregation over collected samples.

Deliberately free of rclpy: the live node in runner.py appends tuples, these
functions turn tuples into the report. That keeps every number in the report
unit-testable without a sim running.

Conventions:
  clock samples : list of (wall_s, sim_s)
  pose samples  : list of (sim_s, x, y, z)
"""

from __future__ import annotations

import math


def rtf(clock: list) -> float | None:
    """Real-time factor actually achieved: sim seconds per wall second."""
    if len(clock) < 2:
        return None
    wall_span = clock[-1][0] - clock[0][0]
    if wall_span <= 0:
        return None
    return (clock[-1][1] - clock[0][1]) / wall_span


def wall_to_sim(clock: list, wall_s: float) -> float | None:
    """Map a wall timestamp (e.g. from a BT log line) onto sim time."""
    if not clock:
        return None
    if wall_s <= clock[0][0]:
        return clock[0][1]
    if wall_s >= clock[-1][0]:
        return clock[-1][1]
    for (w0, s0), (w1, s1) in zip(clock, clock[1:]):
        if w0 <= wall_s <= w1:
            if w1 == w0:
                return s1
            return s0 + (s1 - s0) * (wall_s - w0) / (w1 - w0)
    return clock[-1][1]


def path_length(poses: list) -> float:
    total = 0.0
    for (_, x0, y0, z0), (_, x1, y1, z1) in zip(poses, poses[1:]):
        total += math.dist((x0, y0, z0), (x1, y1, z1))
    return total


def depth_range(poses: list) -> tuple:
    """(shallowest, deepest) z. Negative is below the surface."""
    if not poses:
        return (0.0, 0.0)
    zs = [p[3] for p in poses]
    return (max(zs), min(zs))


def _nearest(samples: list, sim_s: float):
    return min(samples, key=lambda s: abs(s[0] - sim_s))


def drift(odom: list, truth: list) -> dict:
    """Horizontal odom-vs-ground-truth error, matched by nearest sim time."""
    if not odom or not truth:
        return {"max": None, "end": None}
    errors = []
    for sim_s, x, y, _z in odom:
        _, tx, ty, _tz = _nearest(truth, sim_s)
        errors.append(math.hypot(x - tx, y - ty))
    return {"max": max(errors), "end": errors[-1]}


def goal_error(last_goal: tuple | None, poses: list) -> float | None:
    """Distance from the final pose to the last commanded goal."""
    if last_goal is None or not poses:
        return None
    _, x, y, z = poses[-1]
    return math.dist((x, y, z), last_goal)


def stalled(clock: list, now_wall: float, tol_s: float) -> bool:
    """True when /clock has not advanced for tol_s wall seconds.

    A wedged or paused gz is the failure this catches, and it catches it in a
    minute regardless of RTF -- which an absolute wall-clock cap cannot do.

    No samples means the sim has not started publishing yet, which is not a
    stall; readiness is lifecycle.py's job, not this watchdog's.
    """
    if not clock:
        return False
    return (now_wall - clock[-1][0]) > tol_s


def consumed_sim(clock: list, start_sim: float) -> float:
    if not clock:
        return 0.0
    return clock[-1][1] - start_sim


def budget_exhausted(clock: list, start_sim: float, budget_s: float) -> bool:
    return consumed_sim(clock, start_sim) >= budget_s
