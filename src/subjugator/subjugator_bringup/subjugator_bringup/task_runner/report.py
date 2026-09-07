"""Terminal summary, stats.json, and exit-code mapping.

For the people this harness exists for, the report *is* the product: everything
upstream only fills in a dict. Three choices here are load-bearing rather than
cosmetic.

* **Never print a projected or expected wall time.** RTF on this box has been
  measured to vary ~20x between runs of the same mission (0.05 and 0.9 across
  two calib runs), so any "this should take N minutes" claim is misinformation.
  The report states the RTF that was actually measured instead.
* **The stop annotation names `innermost_node`, not `last_node`.** A behaviour
  tree result unwinds *upward*, so the last transition in the log is the
  outermost node of the cascade -- typically a bare `Sequence` -- while the
  first transition of that cascade is the deepest node on the failing path, the
  one that actually explains the run. See btlog.py. The annotation renders even
  when the stage table is empty, because that is the `calib` shape: a
  `RosTimeout` around a single leaf produces no subtree records at all, and
  `innermost_node` is then the only thing the report can say.
* **A ground-truth disagreement is the loudest line in the report.** The scorer
  sets `disagrees_with_bt` when the tree claimed SUCCESS but nothing was
  physically placed. A green verdict that is lying must not read as one more
  table row.

Everything optional degrades to a dash rather than crashing or leaking the
string "None": a killed run has no outcome, no RTF and no stages, and that run
still has to produce a readable report.
"""

from __future__ import annotations

import json

HR = "=" * 60


def _fmt(value, spec="", dash="--"):
    if value is None:
        return dash
    return format(value, spec)


def _cell(value, spec, width):
    """A right-aligned numeric column that degrades to a dash."""
    return f"{_fmt(value, spec):>{width}}"


def _pct(value, width=9):
    """A fraction rendered as a percentage -- presence is 0.98, not 98."""
    if value is None:
        return f"{'--':>{width}} "
    return f"{value * 100:>{width}.0f}%"


def _hms(seconds: float | None) -> str:
    if seconds is None:
        return "--"
    seconds = int(seconds)
    hours, rem = divmod(seconds, 3600)
    minutes, secs = divmod(rem, 60)
    if hours:
        return f"{hours}h{minutes:02d}m"
    if minutes:
        return f"{minutes}m{secs:02d}s"
    return f"{secs}s"


def _stopped_in(node: str, stage_name: str | None = None) -> str:
    """The "where it stopped" annotation, without repeating a name twice."""
    if node == stage_name:
        return "   <- stopped here"
    return f"   <- stopped here, in {node}"


def _header(run: dict) -> list:
    return [
        HR,
        f"run_task {run['task']} -- {run['task_name']}",
        "",
        f"stage: {run['stage']} ({run['mission']})   role: {run.get('role') or '--'}",
        (
            f"outcome:  {run.get('outcome') or 'NO VERDICT'}      "
            f"stop: {run.get('stop_reason', '--')}      exit {exit_code(run)}"
        ),
        (
            f"duration: {_fmt(run.get('sim_duration'), '.0f')} sim-s / "
            f"{_hms(run.get('wall_duration'))} wall    "
            f"RTF {_fmt(run.get('rtf'), '.3f')} measured    "
            f"(budget {_fmt(run.get('sim_budget'), '.0f')} sim-s)"
        ),
    ]


def _behavior_tree(run: dict) -> list:
    stages = run.get("stages") or []
    innermost = run.get("innermost_node")
    if not stages and not innermost:
        return []

    lines = ["", "behavior tree"]
    for index, stage in enumerate(stages):
        label = stage["name"]
        if stage.get("occurrence", 1) > 1:
            label = f"{label}#{stage['occurrence']}"
        row = (
            f"  {label:22s} {stage['result']:8s} "
            f"{_cell(stage.get('sim_duration'), '.0f', 6)} sim-s"
        )
        if innermost and index == len(stages) - 1:
            row += _stopped_in(innermost, stage["name"])
        lines.append(row)

    if not stages:
        lines.append(f"  no subtrees ran{_stopped_in(innermost)}")
    return lines


def _vehicle(run: dict) -> list:
    vehicle = run.get("vehicle") or {}
    if not vehicle:
        return []
    shallow, deep = vehicle.get("depth_range") or (None, None)
    settle = vehicle.get("settle") or {}
    state = "converged" if settle.get("converged") else "capped"
    drift = vehicle.get("drift") or {}
    return [
        "",
        "vehicle",
        (
            f"  path {_fmt(vehicle.get('path_length'), '.1f')} m    "
            f"depth range {_fmt(shallow, '.2f')} .. {_fmt(deep, '.2f')} m    "
            f"final goal error {_fmt(vehicle.get('goal_error'), '.2f')} m"
        ),
        (
            f"  anchor offset {_fmt(vehicle.get('anchor_offset'), '.3f')} m "
            f"(settle {state}, {_fmt(settle.get('sim_s'), '.0f')} sim-s)"
        ),
        (
            f"  odom-vs-truth drift: max {_fmt(drift.get('max'), '.3f')} m, "
            f"end {_fmt(drift.get('end'), '.3f')} m"
        ),
    ]


def _perception(run: dict) -> list:
    perception = run.get("perception") or {}
    if not perception:
        return []

    lines = [
        "",
        (
            f"perception  {perception.get('topic', '--')}   "
            f"{_fmt(perception.get('rate_hz'), '.1f')} Hz   "
            f"{perception.get('frames', 0)} frames   "
            f"longest dropout {_fmt(perception.get('longest_dropout_s'), '.1f')} s"
        ),
    ]

    classes = perception.get("classes") or {}
    if not classes:
        # A topic that produced nothing is a finding, not an empty section:
        # dropping it silently would hide the most common perception failure.
        lines.append("  no detections on this topic")
        return lines

    lines.append(
        f"  {'class':16s}{'detections':>12s}{'present':>10s}"
        f"{'mean conf':>12s}{'min conf':>11s}",
    )
    for name, stats in classes.items():
        lines.append(
            f"  {name:16s}{_cell(stats.get('detections'), 'd', 12)}"
            f"{_pct(stats.get('presence'))}"
            f"{_cell(stats.get('mean_conf'), '.2f', 12)}"
            f"{_cell(stats.get('min_conf'), '.2f', 11)}",
        )
    return lines


def _ground_truth(run: dict) -> list:
    truth = run.get("ground_truth") or {}
    if not truth:
        return []

    lines = ["", "ground truth"]
    for name, obj in (truth.get("objects") or {}).items():
        lines.append(
            f"  {name:16s} moved {_fmt(obj.get('displacement'), '.2f')} m   "
            f"rose {_fmt(obj.get('max_z_rise'), '.2f')} m   {obj.get('verdict', '--')}",
        )

    center = truth.get("basket_center")
    where = f" ({center[0]:.2f}, {center[1]:.2f})" if center else ""
    lines.append(
        f"  verdict: {truth.get('placed', 0)}/{truth.get('targets', 0)} placed "
        f"in the {truth.get('basket', '--')} basket{where}",
    )
    if truth.get("disagrees_with_bt"):
        lines.append(
            "  WARNING: the tree reported SUCCESS but nothing was physically placed.",
        )
    return lines


def _footer(run: dict) -> list:
    health = run.get("health") or {}
    return [
        "",
        (
            f"health  {health.get('crashes', 0)} crashes, "
            f"{health.get('fatal', 0)} FATAL, {health.get('error', 0)} ERROR"
        ),
        f"artifacts  {run.get('run_dir', '--')}",
        HR,
    ]


def render(run: dict) -> str:
    """The terminal summary for one collected run."""
    lines = _header(run)
    lines += _behavior_tree(run)
    lines += _vehicle(run)
    lines += _perception(run)
    lines += _ground_truth(run)
    lines += _footer(run)
    return "\n".join(lines)


def write_json(run: dict, path: str) -> None:
    with open(path, "w") as handle:
        json.dump(run, handle, indent=2, sort_keys=True, default=str)


def exit_code(run: dict) -> int:
    """0 mission SUCCESS, 1 mission ran and lost, 2 the harness itself broke.

    Constants come from exitcodes.py so the contract has exactly one definition.
    A harness error outranks the mission's own verdict: a rig that broke is
    still a rig that broke, whatever the tree managed to report first.
    """
    from .exitcodes import EXIT_HARNESS, EXIT_MISSION_FAILED, EXIT_OK

    if run.get("harness_error"):
        return EXIT_HARNESS
    return EXIT_OK if run.get("outcome") == "SUCCESS" else EXIT_MISSION_FAILED
