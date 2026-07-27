"""The deterministic half of the run: what commands to issue, what to refuse to
run alongside, and what must be dead afterwards.

Kept free of side effects wherever possible so `--dry-run` can print exactly
what a real run would do, and so the tests can assert the command sequence
without a sim.

The stale-process scan matters more than it looks: this box silently
accumulates whole orphaned sim stacks across launches, and a measurement taken
against a polluted ROS graph is worse than no measurement.
"""

from __future__ import annotations

import contextlib
import datetime
import os
import shutil
import signal
import subprocess
from dataclasses import dataclass

# Every process that belongs to a sim stack. Teardown kills all of them, and
# preflight refuses to start when any are already running.
#
# Matching is by substring against the full command line, so these are
# deliberately biased towards catching too much: a false positive costs one
# refused start (or, under --force-clean, one killed process the operator was
# told about by name), while a false negative costs a whole run's numbers.
STACK_PATTERNS = (
    "gz sim",
    "gz-sim",
    "ruby /usr/bin/gz",
    "parameter_bridge",
    "path_planner",
    "trajectory_planner",
    "thruster_manager",
    "forward_to_sim",
    "pid_controller",
    "subjugator_localization",
    "mission_planner_node",
    "yolo_node",
    "pinger_heading_node",
)

SIM_BRINGUP = "pool_tests/sim_bringup.py"


@dataclass(frozen=True)
class Stray:
    pid: str
    name: str


def list_processes() -> list:
    """Raw `ps` lines: '<pid> <command>'. Injected in tests."""
    out = subprocess.run(
        ["ps", "-eo", "pid=,args="],
        capture_output=True,
        text=True,
        check=False,
    )
    return [line.strip() for line in out.stdout.splitlines() if line.strip()]


def _split_ps_line(line: str):
    """('<pid>', '<command>'), or None when the line is not that shape.

    Everything downstream of this ends in a signal sent to a pid, so a line that
    does not parse cleanly is dropped rather than guessed at.
    """
    pid, _, command = line.strip().partition(" ")
    if not (pid.isascii() and pid.isdigit()) or int(pid) <= 0:
        return None
    command = command.strip()
    if not command:
        return None
    return pid, command


def find_stale(lister=list_processes) -> list:
    """Sim-stack processes already running before we start."""
    stale = []
    for line in lister():
        parsed = _split_ps_line(line)
        if parsed is None:
            continue
        pid, command = parsed
        for pattern in STACK_PATTERNS:
            if pattern in command:
                stale.append(Stray(pid=pid, name=pattern))
                break
    return stale


def kill_stale(strays: list, sig: int = signal.SIGTERM) -> None:
    """Signal each stray's whole process group, best effort.

    Groups, because a sim stack is a tree and killing the parent alone reliably
    leaves gz children behind. The group id is read back from the live pid
    rather than assumed to equal it: a pid that has already exited must not be
    turned into a `killpg` against whatever group has since claimed that number.
    Our own group is never signalled -- the harness has a report to write.
    """
    own_group = os.getpgrp()
    own_pid = os.getpid()
    for stray in strays:
        try:
            pid = int(stray.pid)
        except (TypeError, ValueError):
            continue
        if pid <= 0 or pid == own_pid:
            # os.kill(0) hits our own process group and os.kill(-1) hits
            # everything we own; neither is ever what teardown meant.
            continue
        try:
            group = os.getpgid(pid)
        except OSError:
            continue
        if group != own_group:
            try:
                os.killpg(group, sig)
                continue
            except OSError:
                pass
        with contextlib.suppress(OSError):
            os.kill(pid, sig)


def run_dir_name(task: int, stage: str, when: str | None = None) -> str:
    if when is None:
        when = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"task{task}_{stage}_{when}"


def sim_bringup_path() -> str:
    """Prefer the source copy so a rebuild is not needed to pick up edits.

    realpath, not abspath: under a symlink install this module's installed path
    points back into the source tree, which is exactly where the script lives.
    """
    here = os.path.dirname(os.path.realpath(__file__))
    source = os.path.normpath(os.path.join(here, "..", "..", SIM_BRINGUP))
    return (
        source if os.path.exists(source) else shutil.which("sim_bringup.py") or source
    )


def plan_commands(spec, stage_name: str, stage, start_name: str, args) -> list:
    """The exact command sequence a run issues: bring-up, settle, mission."""
    launch = [
        "ros2",
        "launch",
        spec.launch_pkg,
        spec.launch_file,
        f"device:={args.device}",
    ]
    # No stage:= -- task5_sim.launch.py would start its own mission node on a
    # timer, and the harness owns mission start so it can time it.
    for key, value in spec.launch_args.items():
        launch.append(f"{key}:={value}")

    x, y, z = spec.world_start(start_name)
    settle = [
        "python3",
        sim_bringup_path(),
        "--x",
        f"{x:g}",
        "--y",
        f"{y:g}",
        "--z",
        f"{z:g}",
        "--settle-until-still",
        "--twist-tol",
        f"{spec.settle.twist_tol:g}",
        "--stable-samples",
        str(spec.settle.stable_samples),
        "--settle-cap-sim",
        f"{spec.settle.cap_sim_s:g}",
    ]

    mission = [
        "ros2",
        "run",
        "mission_planner",
        "mission_planner_node",
        "--ros-args",
        "-p",
        f"mission:={stage.mission}",
        "-p",
        "use_sim_time:=true",
    ]
    role = effective_role(spec, args)
    if role:
        mission += ["-p", f"role:={role}"]
    for key, value in stage.params.items():
        mission += ["-p", f"{key}:={value}"]
    # Only when asked for: the mission tree carries its own default, and passing
    # score_level:=None would override it with a string that is not an int.
    if args.score_level is not None:
        mission += ["-p", f"score_level:={args.score_level}"]

    return [launch, settle, mission]


def effective_role(spec, args) -> str:
    """The role the mission command will carry.

    Role is per-task data (TaskSpec.role), not a CLI default: a future task
    whose mission declares no `role` parameter must not be handed one. cli
    .resolve() writes this back into args.role, but recomputing here keeps
    plan_commands correct whether or not it was called first.
    """
    return args.role or spec.role


def describe_plan(spec, stage_name, stage, start_name, args, budget) -> str:
    """What --dry-run prints: the whole resolved run, and nothing launched."""
    x, y, z = spec.world_start(start_name)
    role = effective_role(spec, args)
    lines = [
        f"task {args.task}: {spec.name}",
        f"  stage        {stage_name} -> {stage.mission}",
        f"  start        {start_name} -> ({x:.3f}, {y:.3f}, {z:.3f})",
        f"  role         {role or '(none)'}",
        f"  sim budget   {budget:.0f} sim-s",
        f"  stall watch  {args.stall_timeout:.0f} wall-s with no /clock advance",
        f"  ready gates  {', '.join(spec.ready_topics)}",
        "",
        "commands:",
    ]
    for cmd in plan_commands(spec, stage_name, stage, start_name, args):
        lines.append("  " + " ".join(cmd))
    return "\n".join(lines)
