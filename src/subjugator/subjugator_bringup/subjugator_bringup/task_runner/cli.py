"""run_task command line: argument parsing, --list, and input resolution.

Exit codes come from `exitcodes` so the CLI, the report renderer, and the shell
wrapper cannot drift apart.
"""

from __future__ import annotations

import argparse
import sys

from . import tasks
from .exitcodes import EXIT_HARNESS, EXIT_OK


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="run_task",
        description="Bring up a task in sim, run it, tear it down, report on it.",
    )
    p.add_argument("task", nargs="?", type=int, help="task number, e.g. 5")
    p.add_argument("--list", action="store_true", help="list wired tasks and exit")
    p.add_argument("--stage", help="stage to run (default: the task's default)")
    p.add_argument("--start", help="start preset override")
    p.add_argument(
        "--role",
        default=None,
        help="mission role parameter (default: the task's own role)",
    )
    p.add_argument(
        "--score-level",
        type=int,
        dest="score_level",
        help="how far into the mission to run, for trees that support it",
    )
    # No --gui: gazebo.launch.py declares only `world` and hard-codes gui:true, so
    # there is nothing to toggle. Headless needs a launch-file change, out of scope.
    p.add_argument("--device", default="cpu", help="YOLO device, e.g. cuda:0")
    p.add_argument(
        "--sim-budget",
        type=float,
        dest="sim_budget",
        help="override the stage's sim-time budget, in sim seconds",
    )
    p.add_argument(
        "--stall-timeout",
        type=float,
        default=60.0,
        dest="stall_timeout",
        help="abort if /clock stops advancing for this many wall seconds "
        "(default: %(default)s)",
    )
    p.add_argument(
        "--keep-alive",
        action="store_true",
        dest="keep_alive",
        help="leave the sim running after the mission ends, for inspection",
    )
    p.add_argument(
        "--force-clean",
        action="store_true",
        dest="force_clean",
        help="kill leftover sim processes in preflight instead of refusing to start",
    )
    p.add_argument("--bag", action="store_true", help="record a rosbag of the run")
    p.add_argument(
        "--dry-run",
        action="store_true",
        dest="dry_run",
        help="print the commands that would run, then exit without launching them",
    )
    return p


def format_listing() -> str:
    lines = []
    for num, spec in sorted(tasks.TASKS.items()):
        lines.append(f"task {num}: {spec.name}")
        for name, stage in spec.stages.items():
            label = f"{name} (default)" if name == spec.default_stage else name
            note = f"  [{stage.budget_note}]" if stage.budget_note else ""
            lines.append(
                f"  {label:19s} {stage.mission:22s} "
                f"budget {stage.sim_budget:6.0f} sim-s  start {stage.start}{note}",
            )
    return "\n".join(lines)


def resolve(args) -> tuple:
    """Return (spec, stage_name, stage, start_name); also fills in args.role.

    Raises SystemExit(EXIT_HARNESS) on bad input.
    """
    spec = tasks.TASKS.get(args.task)
    if spec is None:
        print(f"run_task: task {args.task} is not wired.\n", file=sys.stderr)
        print(format_listing(), file=sys.stderr)
        raise SystemExit(EXIT_HARNESS)

    stage_name = args.stage or spec.default_stage
    stage = spec.stages.get(stage_name)
    if stage is None:
        known = ", ".join(spec.stages)
        print(
            f"run_task: unknown stage '{stage_name}' (have: {known})",
            file=sys.stderr,
        )
        raise SystemExit(EXIT_HARNESS)

    start_name = args.start or stage.start
    if start_name not in spec.starts:
        known = ", ".join(spec.starts)
        print(
            f"run_task: unknown start '{start_name}' (have: {known})",
            file=sys.stderr,
        )
        raise SystemExit(EXIT_HARNESS)

    # Empty means the task takes no role, and the run then omits `role:=` entirely.
    args.role = args.role or spec.role

    return spec, stage_name, stage, start_name


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.list:
        print(format_listing())
        return EXIT_OK

    if args.task is None:
        parser.print_usage(sys.stderr)
        return EXIT_HARNESS

    resolve(args)
    print("run_task: orchestration lands in Task 10 of the plan.", file=sys.stderr)
    return EXIT_HARNESS
