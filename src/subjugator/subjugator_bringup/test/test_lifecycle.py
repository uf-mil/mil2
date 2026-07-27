"""Tests for the deterministic half of a run: commands, preflight, teardown.

Nothing here launches anything. Where a command has to line up with a file that
really exists (task5_sim.launch.py's declared arguments, sim_bringup.py's
argparse block), the expectation is read out of that file rather than restated,
so the test fails when the two drift apart instead of when someone edits a list.
"""

from __future__ import annotations

import argparse
import ast
import dataclasses
import datetime
import os
import re
import signal
from pathlib import Path

import pytest
from subjugator_bringup.task_runner import lifecycle, tasks

PKG = Path(__file__).resolve().parents[1]


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def args(**overrides):
    # role=None exercises the TaskSpec.role fallback; a test below overrides it.
    base = {
        "task": 5,
        "stage": None,
        "start": None,
        "role": None,
        "score_level": None,
        "device": "cpu",
        "sim_budget": None,
        "stall_timeout": 60.0,
        "keep_alive": False,
        "force_clean": False,
        "bag": False,
        "dry_run": True,
        "list": False,
    }
    base.update(overrides)
    return argparse.Namespace(**base)


def plan(spec=None, **overrides):
    a = args(**overrides)
    spec = spec or tasks.TASKS[5]
    stage_name = a.stage or spec.default_stage
    return lifecycle.plan_commands(
        spec,
        stage_name,
        spec.stages[stage_name],
        a.start or spec.stages[stage_name].start,
        a,
    )


def value_after(cmd, flag):
    """The token following `flag`, or None when the flag is absent."""
    if flag not in cmd:
        return None
    return cmd[cmd.index(flag) + 1]


def declared_launch_arguments():
    """Argument names task5_sim.launch.py really declares."""
    src = (PKG / "launch" / "task5_sim.launch.py").read_text()
    names = set()
    for node in ast.walk(ast.parse(src)):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "DeclareLaunchArgument"
            and node.args
            and isinstance(node.args[0], ast.Constant)
        ):
            names.add(node.args[0].value)
    assert names, "found no DeclareLaunchArgument calls -- parser is broken"
    return names


def sim_bringup_parser():
    """Rebuild sim_bringup.py's argparse block, flags and actions only.

    Importing the script is not an option (it imports rclpy at module scope), so
    the option table is lifted straight out of its source. Types are dropped:
    this parser answers "does the flag exist and take the right shape", and a
    separate test checks the values convert.
    """
    src = (PKG / "pool_tests" / "sim_bringup.py").read_text()
    parser = argparse.ArgumentParser(add_help=False)
    found = 0
    for node in ast.walk(ast.parse(src)):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "add_argument"
        ):
            continue
        flags = [
            a.value
            for a in node.args
            if isinstance(a, ast.Constant) and isinstance(a.value, str)
        ]
        if not flags:
            continue
        kwargs = {}
        for kw in node.keywords:
            if kw.arg == "action" and isinstance(kw.value, ast.Constant):
                kwargs["action"] = kw.value.value
        parser.add_argument(*flags, **kwargs)
        found += 1
    assert found >= 5, f"only lifted {found} sim_bringup.py arguments"
    return parser


class FakeKills:
    """Records the process-signalling syscalls kill_stale makes."""

    def __init__(self, monkeypatch, groups=None, killpg_raises=None):
        self.killpg = []
        self.kill = []
        self.groups = groups or {}
        self._killpg_raises = killpg_raises
        monkeypatch.setattr(os, "getpgid", self._getpgid)
        monkeypatch.setattr(os, "killpg", self._killpg)
        monkeypatch.setattr(os, "kill", self._kill)

    def _getpgid(self, pid):
        if pid in self.groups:
            group = self.groups[pid]
            if isinstance(group, Exception):
                raise group
            return group
        return pid

    def _killpg(self, pgid, sig):
        self.killpg.append((pgid, sig))
        if self._killpg_raises is not None:
            raise self._killpg_raises

    def _kill(self, pid, sig):
        self.kill.append((pid, sig))


# --------------------------------------------------------------------------
# plan_commands: overall shape
# --------------------------------------------------------------------------
def test_three_phases_in_order():
    cmds = plan()
    assert len(cmds) == 3
    assert cmds[0][:2] == ["ros2", "launch"]
    assert "sim_bringup.py" in " ".join(cmds[1])
    assert cmds[2][:3] == ["ros2", "run", "mission_planner"]


def test_every_token_is_a_string():
    # A stray int here is a TypeError inside subprocess at run time, long after
    # the point where it could be understood.
    for cmd in plan():
        assert cmd, "empty command"
        for token in cmd:
            assert isinstance(token, str), (cmd, token)


def test_planning_mutates_nothing():
    a = args()
    spec = tasks.TASKS[5]
    before = dict(vars(a))
    lifecycle.plan_commands(spec, "full", spec.stages["full"], "near", a)
    assert vars(a) == before
    assert spec.launch_args == {"world": "robosub_2025.world"}
    assert spec.stages["full"].params == {"do_pinger": 0}


def test_planning_is_repeatable():
    assert plan() == plan()


# --------------------------------------------------------------------------
# plan_commands: bring-up
# --------------------------------------------------------------------------
def test_bringup_never_passes_a_stage():
    assert "stage:=" not in " ".join(plan()[0])


def test_bringup_names_the_specs_package_and_launch_file():
    spec = tasks.TASKS[5]
    assert plan()[0][:4] == ["ros2", "launch", spec.launch_pkg, spec.launch_file]


def test_bringup_carries_device_and_world():
    cmd = " ".join(plan(device="cuda:0")[0])
    assert "device:=cuda:0" in cmd
    assert "world:=robosub_2025.world" in cmd


def test_bringup_default_device_is_whatever_the_caller_passed():
    assert "device:=cpu" in plan()[0]


def test_bringup_forwards_every_spec_launch_arg():
    spec = dataclasses.replace(
        tasks.TASKS[5],
        launch_args={"world": "other.world", "mission_delay": "0.0"},
    )
    cmd = plan(spec)[0]
    assert "world:=other.world" in cmd
    assert "mission_delay:=0.0" in cmd


def test_bringup_passes_only_declared_launch_arguments():
    # task5_sim.launch.py declares these; passing anything else makes ros2 launch
    # fail outright, so the table must never grow an argument the file lacks.
    declared = declared_launch_arguments()
    for token in plan()[0][4:]:
        assert token.split(":=")[0] in declared, token


def test_the_declared_argument_set_is_the_one_the_plan_expected():
    # Guards the guard: if the launch file's argument list changes, this is the
    # test that says so out loud rather than silently widening the check above.
    assert declared_launch_arguments() == {
        "world",
        "device",
        "model",
        "down_image_topic",
        "namespace",
        "stage",
        "role",
        "score_level",
        "no_pinger",
        "mission_delay",
    }


def test_bringup_arguments_are_all_name_value_pairs():
    for token in plan()[0][4:]:
        assert ":=" in token, token
        name, _, value = token.partition(":=")
        assert name and value, token


def test_bringup_takes_no_mission_parameters():
    joined = " ".join(plan(score_level=3, role="search_rescue")[0])
    for leaked in ("role:=", "score_level:=", "--ros-args", "do_pinger"):
        assert leaked not in joined


# --------------------------------------------------------------------------
# plan_commands: settle
# --------------------------------------------------------------------------
def test_settle_uses_the_resolved_world_start():
    cmd = " ".join(plan(stage="full")[1])
    assert "--x -7.25" in cmd
    assert "--y 12.5" in cmd
    assert "--settle-until-still" in cmd


def test_tuning_stage_starts_over_the_table():
    cmd = " ".join(plan(stage="calib")[1])
    assert "--x -7.7" in cmd


def test_settle_carries_all_three_coordinates_in_order():
    spec = tasks.TASKS[5]
    x, y, z = spec.world_start("over_table")
    cmd = plan(stage="calib")[1]
    assert float(value_after(cmd, "--x")) == pytest.approx(x)
    assert float(value_after(cmd, "--y")) == pytest.approx(y)
    assert float(value_after(cmd, "--z")) == pytest.approx(z)
    assert cmd.index("--x") < cmd.index("--y") < cmd.index("--z")


def test_settle_coordinates_are_not_transposed():
    # x and y are far enough apart in every start that a swap is unmissable.
    cmd = plan(stage="full")[1]
    assert value_after(cmd, "--x") == "-7.25"
    assert value_after(cmd, "--y") == "12.5"
    assert value_after(cmd, "--z") == "-0.35"


def test_start_override_beats_the_stages_own_start():
    cmd = plan(stage="calib", start="near")[1]
    assert float(value_after(cmd, "--y")) == pytest.approx(12.5)


def test_settle_gate_values_come_from_the_specs_settle_block():
    spec = dataclasses.replace(
        tasks.TASKS[5],
        settle=tasks.Settle(twist_tol=0.05, stable_samples=3, cap_sim_s=7.0),
    )
    cmd = plan(spec)[1]
    assert value_after(cmd, "--twist-tol") == "0.05"
    assert value_after(cmd, "--stable-samples") == "3"
    assert value_after(cmd, "--settle-cap-sim") == "7"


def test_settle_defaults_match_the_task_table():
    settle = tasks.TASKS[5].settle
    cmd = plan()[1]
    assert float(value_after(cmd, "--twist-tol")) == pytest.approx(settle.twist_tol)
    assert int(value_after(cmd, "--stable-samples")) == settle.stable_samples
    assert float(value_after(cmd, "--settle-cap-sim")) == pytest.approx(
        settle.cap_sim_s,
    )


def test_settle_flags_actually_parse_against_sim_bringup():
    # The failure mode this catches -- a flag sim_bringup.py does not declare --
    # shows up live as a bare exit 2 halfway through bring-up.
    parser = sim_bringup_parser()
    parsed = parser.parse_args(plan()[1][2:])
    assert parsed.settle_until_still is True


def test_settle_values_convert_to_the_types_sim_bringup_wants():
    cmd = plan()[1]
    for flag in ("--x", "--y", "--z", "--twist-tol", "--settle-cap-sim"):
        float(value_after(cmd, flag))
    assert int(value_after(cmd, "--stable-samples")) >= 1


def test_settle_runs_a_script_that_exists():
    cmd = plan()[1]
    assert cmd[0] == "python3"
    assert os.path.exists(cmd[1]), cmd[1]
    assert cmd[1].endswith("sim_bringup.py")


def test_settle_flags_all_carry_a_value_where_one_is_required():
    parser = sim_bringup_parser()
    known = {
        action.option_strings[0]: action.nargs != 0
        for action in parser._actions
        if action.option_strings
    }
    cmd = plan()[1]
    i = 2
    while i < len(cmd):
        flag = cmd[i]
        assert flag.startswith("--"), flag
        assert flag in known, flag
        if known[flag]:
            assert i + 1 < len(cmd), f"{flag} has no value"
            assert cmd[i + 1] not in known, f"{flag} swallowed another flag"
            i += 2
        else:
            i += 1


# --------------------------------------------------------------------------
# plan_commands: mission
# --------------------------------------------------------------------------
def test_mission_command_contents():
    cmd = " ".join(plan(stage="full")[2])
    assert "mission:=OctagonMission" in cmd
    assert "role:=survey_repair" in cmd  # from TaskSpec.role, not a CLI default
    assert "use_sim_time:=true" in cmd
    assert "do_pinger:=0" in cmd


def test_mission_runs_the_mission_planner_node():
    assert plan()[2][:4] == [
        "ros2",
        "run",
        "mission_planner",
        "mission_planner_node",
    ]


def test_mission_parameters_sit_behind_a_single_ros_args():
    cmd = plan(score_level=2)[2]
    assert cmd.count("--ros-args") == 1
    ros_args = cmd.index("--ros-args")
    rest = cmd[ros_args + 1 :]
    assert rest, "no parameters after --ros-args"
    for flag, param in zip(rest[::2], rest[1::2]):
        # `-p name:=value`; a bare `-p name value` is silently a remap.
        assert flag == "-p", rest
        assert ":=" in param, param
    assert len(rest) % 2 == 0, rest


def test_mission_uses_the_stages_own_mission_name():
    assert "mission:=CenterCameraTest" in plan(stage="calib")[2]


def test_stage_without_params_gets_none():
    assert "do_pinger:=0" not in plan(stage="calib")[2]


def test_mission_carries_every_stage_param():
    spec = tasks.TASKS[5]
    spec = dataclasses.replace(
        spec,
        stages=dict(
            spec.stages,
            full=dataclasses.replace(
                spec.stages["full"],
                params={"do_pinger": 0, "grasp_targets_file": "/tmp/t.yaml"},
            ),
        ),
    )
    cmd = plan(spec)[2]
    assert "do_pinger:=0" in cmd
    assert "grasp_targets_file:=/tmp/t.yaml" in cmd


def test_role_flag_overrides_the_task_default():
    assert "role:=search_rescue" in " ".join(plan(role="search_rescue")[2])


def test_role_is_omitted_entirely_when_the_task_has_none():
    spec = dataclasses.replace(tasks.TASKS[5], role="")
    assert "role:=" not in " ".join(plan(spec)[2])


def test_empty_role_argument_falls_back_to_the_task_role():
    # cli.resolve() writes the spec role into args.role, but plan_commands must
    # not depend on having been called after it.
    assert "role:=survey_repair" in plan(role="")[2]


def test_role_flag_wins_even_for_a_task_with_no_role():
    spec = dataclasses.replace(tasks.TASKS[5], role="")
    assert "role:=search_rescue" in plan(spec, role="search_rescue")[2]


def test_score_level_only_when_given():
    assert "score_level:=" not in " ".join(plan()[2])
    assert "score_level:=3" in " ".join(plan(score_level=3)[2])


def test_score_level_zero_is_still_passed():
    # 0 is a real ladder rung ("run nothing but the pre-checks"), so the guard
    # has to be `is not None` and not a truthiness test.
    assert "score_level:=0" in plan(score_level=0)[2]


# --------------------------------------------------------------------------
# find_stale
# --------------------------------------------------------------------------
def test_find_stale_reports_known_strays():
    listing = [
        "1234 gz sim -s -r robosub_2025.world",
        "1235 /opt/ros/jazzy/lib/subjugator_path_planner/path_planner",
        "1236 something_unrelated --flag",
    ]
    stale = lifecycle.find_stale(lambda: listing)
    assert len(stale) == 2
    assert any("gz" in s.name for s in stale)
    assert any("path_planner" in s.name for s in stale)


def test_find_stale_is_empty_on_a_clean_machine():
    assert lifecycle.find_stale(lambda: ["1 /usr/bin/bash"]) == []


def test_find_stale_keeps_the_pid():
    stale = lifecycle.find_stale(lambda: ["4321 /opt/ros/lib/x/trajectory_planner"])
    assert stale[0].pid == "4321"
    assert stale[0].name == "trajectory_planner"


@pytest.mark.parametrize("pattern", lifecycle.STACK_PATTERNS)
def test_every_pattern_is_detectable(pattern):
    stale = lifecycle.find_stale(lambda: [f"999 /usr/bin/{pattern} --ros-args"])
    assert [s.name for s in stale] == [pattern]


def test_the_whole_stack_is_covered():
    # Teardown that misses one of these leaves an orphaned stack behind, and the
    # next run's numbers are quietly wrong. See sim_teardown_orphaned_stacks.
    for member in (
        "gz sim",
        "parameter_bridge",
        "path_planner",
        "trajectory_planner",
        "thruster_manager",
        "forward_to_sim",
        "pid_controller",
        "subjugator_localization",
        "mission_planner_node",
        "yolo_node",
    ):
        assert member in lifecycle.STACK_PATTERNS, member


def test_a_line_is_reported_once_even_when_it_matches_twice():
    line = "77 /opt/ros/lib/path_planner --params trajectory_planner.yaml"
    stale = lifecycle.find_stale(lambda: [line])
    assert len(stale) == 1


def test_leading_whitespace_from_ps_is_tolerated():
    # `ps -eo pid=,args=` right-aligns the pid column.
    stale = lifecycle.find_stale(lambda: ["   12 gz sim -r x.world"])
    assert [(s.pid, s.name) for s in stale] == [("12", "gz sim")]


MALFORMED = [
    "",
    "   ",
    "1234",  # pid with no command
    "1234 ",  # pid with a blank command
    "gz sim -r robosub.world",  # command with no pid
    "notapid gz sim -r x.world",  # misparsed pid column
    "-5 gz sim",  # negative pid
    "0 gz sim",  # pid 0 is our own process group, never a stray
    "1234.5 gz sim",  # not an integer
    "١٢٣ gz sim",  # non-ascii digits: int() takes them, kill wants ascii
]


@pytest.mark.parametrize("line", MALFORMED)
def test_malformed_ps_lines_never_become_kill_targets(line):
    assert lifecycle.find_stale(lambda: [line]) == []


@pytest.mark.parametrize("line", MALFORMED)
def test_malformed_ps_lines_do_not_parse(line):
    # Pinned on the parser itself, not just on the pattern match that happens to
    # follow it: everything downstream of this ends in a signal sent to a pid.
    assert lifecycle._split_ps_line(line) is None


@pytest.mark.parametrize(
    ("line", "expected"),
    [
        ("1234 gz sim -r x.world", ("1234", "gz sim -r x.world")),
        ("   9 /usr/bin/gz sim", ("9", "/usr/bin/gz sim")),
        ("1234   spaced   out  ", ("1234", "spaced   out")),
        ("1234 a b c", ("1234", "a b c")),
    ],
)
def test_well_formed_ps_lines_parse(line, expected):
    assert lifecycle._split_ps_line(line) == expected


def test_a_clean_listing_with_near_misses_stays_clean():
    listing = [
        "1 /usr/bin/bash",
        "2 python3 -m pytest test/test_lifecycle.py",
        "3 /usr/bin/ros2 topic list",
    ]
    assert lifecycle.find_stale(lambda: listing) == []


def test_find_stale_defaults_to_the_real_process_lister():
    import inspect

    default = inspect.signature(lifecycle.find_stale).parameters["lister"].default
    assert default is lifecycle.list_processes


def test_list_processes_returns_pid_command_lines():
    lines = lifecycle.list_processes()
    assert lines
    assert all(re.match(r"^\d+ ", line) for line in lines[:20]), lines[:5]
    assert any(line.startswith(f"{os.getpid()} ") for line in lines)


# --------------------------------------------------------------------------
# kill_stale
# --------------------------------------------------------------------------
def test_kill_stale_signals_the_whole_process_group(monkeypatch):
    fake = FakeKills(monkeypatch, groups={4242: 4200})
    lifecycle.kill_stale([lifecycle.Stray(pid="4242", name="gz sim")])
    assert fake.killpg == [(4200, signal.SIGTERM)]
    assert fake.kill == []


def test_kill_stale_accepts_a_different_signal(monkeypatch):
    fake = FakeKills(monkeypatch, groups={7: 7})
    lifecycle.kill_stale([lifecycle.Stray(pid="7", name="gz sim")], signal.SIGKILL)
    assert fake.killpg == [(7, signal.SIGKILL)]


def test_kill_stale_skips_a_process_that_already_exited(monkeypatch):
    fake = FakeKills(monkeypatch, groups={9: ProcessLookupError()})
    lifecycle.kill_stale([lifecycle.Stray(pid="9", name="gz sim")])
    # The pid may already have been recycled onto someone else's process group,
    # so neither the group nor the bare pid may be signalled.
    assert fake.killpg == []
    assert fake.kill == []


def test_kill_stale_never_signals_pid_zero_or_negative(monkeypatch):
    # os.kill(0, ...) signals our own process group and os.kill(-1, ...) signals
    # everything we own, so a misparsed pid here is catastrophic, not noisy.
    fake = FakeKills(monkeypatch)
    lifecycle.kill_stale(
        [
            lifecycle.Stray(pid="0", name="gz sim"),
            lifecycle.Stray(pid="-1", name="gz sim"),
        ],
    )
    assert fake.killpg == []
    assert fake.kill == []


def test_kill_stale_ignores_an_unparseable_pid(monkeypatch):
    fake = FakeKills(monkeypatch)
    lifecycle.kill_stale(
        [
            lifecycle.Stray(pid="", name="gz sim"),
            lifecycle.Stray(pid="nonsense", name="gz sim"),
            lifecycle.Stray(pid="12 34", name="gz sim"),
        ],
    )
    assert fake.killpg == []
    assert fake.kill == []


def test_kill_stale_never_kills_the_harness_itself(monkeypatch):
    fake = FakeKills(monkeypatch)
    lifecycle.kill_stale([lifecycle.Stray(pid=str(os.getpid()), name="yolo_node")])
    assert fake.killpg == []
    assert fake.kill == []


def test_kill_stale_signals_a_group_mate_individually(monkeypatch):
    # A child sharing our process group: killing the group would take the
    # harness down with it, along with the report it has not written yet.
    own = os.getpgrp()
    fake = FakeKills(monkeypatch, groups={555: own})
    lifecycle.kill_stale([lifecycle.Stray(pid="555", name="mission_planner_node")])
    assert fake.killpg == []
    assert fake.kill == [(555, signal.SIGTERM)]


def test_kill_stale_falls_back_to_the_pid_when_killpg_fails(monkeypatch):
    fake = FakeKills(
        monkeypatch,
        groups={31: 30},
        killpg_raises=PermissionError(),
    )
    lifecycle.kill_stale([lifecycle.Stray(pid="31", name="gz sim")])
    assert fake.killpg == [(30, signal.SIGTERM)]
    assert fake.kill == [(31, signal.SIGTERM)]


def test_kill_stale_survives_a_process_dying_mid_kill(monkeypatch):
    fake = FakeKills(monkeypatch, groups={41: 40}, killpg_raises=ProcessLookupError())

    def boom(pid, sig):
        raise ProcessLookupError

    monkeypatch.setattr(os, "kill", boom)
    lifecycle.kill_stale([lifecycle.Stray(pid="41", name="gz sim")])
    assert fake.killpg == [(40, signal.SIGTERM)]


def test_kill_stale_keeps_going_after_one_failure(monkeypatch):
    fake = FakeKills(monkeypatch, groups={1: ProcessLookupError(), 2: 2})
    lifecycle.kill_stale(
        [
            lifecycle.Stray(pid="1", name="gz sim"),
            lifecycle.Stray(pid="2", name="yolo_node"),
        ],
    )
    assert fake.killpg == [(2, signal.SIGTERM)]


def test_kill_stale_on_an_empty_list_touches_nothing(monkeypatch):
    fake = FakeKills(monkeypatch)
    lifecycle.kill_stale([])
    assert fake.killpg == [] and fake.kill == []


# --------------------------------------------------------------------------
# run_dir_name
# --------------------------------------------------------------------------
def test_run_dir_name():
    name = lifecycle.run_dir_name(5, "full", when="20260726T031500Z")
    assert name == "task5_full_20260726T031500Z"


def test_run_dir_name_varies_with_task_and_stage():
    assert lifecycle.run_dir_name(7, "calib", when="X") == "task7_calib_X"


def test_run_dir_name_stamps_utc_now_by_default():
    before = datetime.datetime.now(datetime.timezone.utc)
    name = lifecycle.run_dir_name(5, "grasp")
    match = re.fullmatch(r"task5_grasp_(\d{8}T\d{6})Z", name)
    assert match, name
    stamped = datetime.datetime.strptime(match.group(1), "%Y%m%dT%H%M%S").replace(
        tzinfo=datetime.timezone.utc,
    )
    assert abs((stamped - before).total_seconds()) < 120


def test_run_dir_name_is_filesystem_safe():
    assert re.fullmatch(r"[A-Za-z0-9_]+", lifecycle.run_dir_name(5, "full"))


# --------------------------------------------------------------------------
# sim_bringup_path
# --------------------------------------------------------------------------
def test_sim_bringup_path_finds_the_source_copy():
    path = lifecycle.sim_bringup_path()
    assert os.path.exists(path)
    assert Path(path) == PKG / "pool_tests" / "sim_bringup.py"


def test_sim_bringup_path_prefers_the_source_copy_over_one_on_the_path(monkeypatch):
    # Editing pool_tests/sim_bringup.py must take effect without a rebuild.
    monkeypatch.setattr(
        lifecycle.shutil,
        "which",
        lambda name: "/usr/local/bin/sim_bringup.py",
    )
    assert Path(lifecycle.sim_bringup_path()) == PKG / "pool_tests" / "sim_bringup.py"


def test_sim_bringup_path_falls_back_to_the_installed_script(monkeypatch):
    monkeypatch.setattr(os.path, "exists", lambda p: False)
    monkeypatch.setattr(
        lifecycle.shutil,
        "which",
        lambda name: "/usr/local/bin/sim_bringup.py",
    )
    assert lifecycle.sim_bringup_path() == "/usr/local/bin/sim_bringup.py"


def test_sim_bringup_path_returns_a_usable_string_when_nothing_is_found(monkeypatch):
    monkeypatch.setattr(os.path, "exists", lambda p: False)
    monkeypatch.setattr(lifecycle.shutil, "which", lambda name: None)
    path = lifecycle.sim_bringup_path()
    assert isinstance(path, str)
    assert path.endswith("sim_bringup.py")


# --------------------------------------------------------------------------
# describe_plan
# --------------------------------------------------------------------------
def describe(**overrides):
    a = args(**overrides)
    spec = tasks.TASKS[5]
    stage_name = a.stage or spec.default_stage
    stage = spec.stages[stage_name]
    return lifecycle.describe_plan(
        spec,
        stage_name,
        stage,
        a.start or stage.start,
        a,
        stage.sim_budget,
    )


def test_describe_plan_prints_every_command():
    text = describe()
    for cmd in plan():
        assert " ".join(cmd) in text


def test_describe_plan_names_the_stage_start_and_budget():
    text = describe(stage="full")
    assert "OctagonMission" in text
    assert "near" in text
    assert "-7.250" in text and "12.500" in text
    assert "1300" in text


def test_describe_plan_reports_the_effective_role():
    # args.role is None until cli.resolve() fills it in; printing "None" here
    # would be a lie about what the mission command carries.
    assert "None" not in describe()
    assert "survey_repair" in describe()


def test_describe_plan_reports_an_overridden_role():
    assert "search_rescue" in describe(role="search_rescue")


def test_describe_plan_lists_the_ready_gates():
    text = describe()
    for topic in tasks.TASKS[5].ready_topics:
        assert topic in text
