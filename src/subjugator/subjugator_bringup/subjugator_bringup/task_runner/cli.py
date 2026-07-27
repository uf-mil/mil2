"""run_task command line: argument parsing, --list, and exit-code discipline.

Exit codes are load-bearing and split "the robot failed" from "the rig failed":
  0  mission SUCCESS
  1  mission ran and FAILED, or consumed its sim budget
  2  harness error (unknown task, sim never came up, gate timed out, clock stalled)
"""

from __future__ import annotations

import argparse
import sys

from . import tasks

EXIT_OK = 0
EXIT_MISSION_FAILED = 1
EXIT_HARNESS = 2


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="run_task",
        description="Bring up a task in sim, run it, tear it down, report on it.",
    )
    p.add_argument("task", nargs="?", type=int, help="task number, e.g. 5")
    p.add_argument("--list", action="store_true", help="list wired tasks and exit")
    p.add_argument("--stage", help="stage to run (default: the task's default)")
    p.add_argument("--start", help="start preset override")
    p.add_argument("--role", default="survey_repair")
    p.add_argument("--score-level", type=int, dest="score_level")
    # No --gui: gazebo.launch.py declares only `world` and hard-codes gui:true, so
    # there is nothing to toggle. Headless needs a launch-file change, out of scope.
    p.add_argument("--device", default="cpu", help="YOLO device, e.g. cuda:0")
    p.add_argument("--sim-budget", type=float, dest="sim_budget")
    p.add_argument("--stall-timeout", type=float, default=60.0, dest="stall_timeout")
    p.add_argument("--keep-alive", action="store_true", dest="keep_alive")
    p.add_argument("--force-clean", action="store_true", dest="force_clean")
    p.add_argument("--bag", action="store_true")
    p.add_argument("--dry-run", action="store_true", dest="dry_run")
    return p


def format_listing() -> str:
    lines = []
    for num, spec in sorted(tasks.TASKS.items()):
        lines.append(f"task {num}: {spec.name}")
        for name, stage in spec.stages.items():
            default = " (default)" if name == spec.default_stage else ""
            lines.append(
                f"  {name:9s} {stage.mission:22s} "
                f"budget {stage.sim_budget:6.0f} sim-s  start {stage.start}{default}",
            )
    return "\n".join(lines)


def resolve(args) -> tuple:
    """Return (spec, stage_name, stage, start_name). Raises SystemExit(2) on bad input."""
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

    return spec, stage_name, stage, start_name


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    if args.list:
        print(format_listing())
        return EXIT_OK

    if args.task is None:
        build_parser().print_usage(sys.stderr)
        return EXIT_HARNESS

    resolve(args)
    print("run_task: orchestration lands in Task 10 of the plan.", file=sys.stderr)
    return EXIT_HARNESS
