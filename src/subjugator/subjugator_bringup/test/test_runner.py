"""Pure helpers inside the live orchestrator.

runner.py imports rclpy, so these skip rather than fail on an unsourced shell;
everything they cover is deliberately free of ROS at call time.
"""

import argparse

import pytest

pytest.importorskip("rclpy", reason="runner.py needs a sourced ROS 2 workspace")

from subjugator_bringup.task_runner import runner, tasks  # noqa: E402

# The shape `gz topic -e -t /world/<w>/pose/info -n 1` prints: a repeated
# `pose` field, closing brace in column 0. Includes a link entry sharing its
# model's name, which is what select_poses has to collapse.
DUMP = """header {
  stamp {
    sec: 42
  }
}
pose {
  name: "sub9"
  id: 8
  position {
    x: -7.25
    y: 12.5
    z: -0.35
  }
  orientation {
    w: 1
  }
}
pose {
  name: "electric_box"
  id: 15
  position {
    x: -7.1
    y: 13.89
    z: -0.8
  }
  orientation {
    w: 1
  }
}
pose {
  name: "electric_box"
  id: 16
  position {
    x: -9.9
    y: 9.9
    z: -9.9
  }
  orientation {
    w: 1
  }
}
pose {
  name: "no_position_here"
  id: 17
  orientation {
    w: 1
  }
}
"""


def args(**overrides):
    base = {"sim_budget": None}
    base.update(overrides)
    return argparse.Namespace(**base)


# --------------------------------------------------------------------------
# world naming
# --------------------------------------------------------------------------
def test_gz_world_name_strips_the_suffix():
    assert runner.gz_world_name({"world": "robosub_2025.world"}) == "robosub_2025"


def test_gz_world_name_matches_the_task_table():
    # sim_bringup.py talks to /world/robosub_2025/...; a mismatch here would
    # make every ground-truth poll silently return nothing.
    spec = tasks.TASKS[5]
    assert runner.gz_world_name(spec.launch_args) == "robosub_2025"


def test_gz_world_name_falls_back_when_unset():
    assert runner.gz_world_name({}) == runner.DEFAULT_GZ_WORLD


# --------------------------------------------------------------------------
# pose parsing
# --------------------------------------------------------------------------
def test_parse_gz_poses_reads_names_and_positions():
    poses = dict(runner.parse_gz_poses(DUMP))
    assert poses["sub9"] == (-7.25, 12.5, -0.35)


def test_parse_gz_poses_skips_entries_without_a_position():
    assert "no_position_here" not in dict(runner.parse_gz_poses(DUMP))


def test_parse_gz_poses_on_empty_output():
    assert runner.parse_gz_poses("") == []


def test_select_poses_keeps_one_sample_per_model():
    picked = runner.select_poses(
        runner.parse_gz_poses(DUMP),
        {"sub9": [], "electric_box": []},
    )
    assert set(picked) == {"sub9", "electric_box"}
    assert picked["electric_box"] == (-7.1, 13.89, -0.8)


def test_select_poses_ignores_models_we_do_not_track():
    assert runner.select_poses(runner.parse_gz_poses(DUMP), {"table": []}) == {}


# --------------------------------------------------------------------------
# budget
# --------------------------------------------------------------------------
def test_budget_comes_from_the_stage():
    stage = tasks.TASKS[5].stages["calib"]
    assert runner.resolve_budget(args(), stage) == 60.0


def test_budget_flag_overrides_the_stage():
    stage = tasks.TASKS[5].stages["calib"]
    assert runner.resolve_budget(args(sim_budget=12.0), stage) == 12.0


def test_budget_is_floored_above_zero():
    # budget_exhausted is `>=`, so a zero budget is spent before the first tick,
    # and btbudget.declared_bound() legitimately returns 0.0 for a tree with no
    # statically resolvable timeout.
    stage = tasks.Stage("Whatever", 0.0, "over_table")
    assert runner.resolve_budget(args(), stage) >= runner.MIN_BUDGET_S


# --------------------------------------------------------------------------
# mission windowing
# --------------------------------------------------------------------------
# Real shape from a calib run: robot_localization's estimate diverges to ~1e24 m
# before sim_bringup re-anchors it, then settles onto the true pose.
PRE_ANCHOR = [
    (0.129, -7.19e22, -7.97e24, -3.50e19),
    (0.234, 2.40e23, 2.66e25, 1.89e21),
]
POST_ANCHOR = [(7.917, -7.7011, 13.9671, -0.3226), (8.004, -7.7012, 13.9676, -0.3212)]


def test_a_pre_anchor_sample_would_destroy_path_length():
    """Why the window exists, stated as a test rather than a comment."""
    from subjugator_bringup.task_runner import collectors

    assert collectors.path_length(PRE_ANCHOR + POST_ANCHOR) > 1e24
    assert collectors.path_length(POST_ANCHOR) < 0.01


def test_windowing_drops_everything_before_the_mission_start():
    start_sim = 7.0
    windowed = [s for s in PRE_ANCHOR + POST_ANCHOR if s[0] >= start_sim]
    assert windowed == POST_ANCHOR


# --------------------------------------------------------------------------
# clock
# --------------------------------------------------------------------------
def test_clock_advancing_needs_two_distinct_sim_times():
    # A paused gz republishes the same stamp forever, so counting messages
    # would declare a frozen sim ready. This is the gate that caught it.
    assert runner.clock_advancing([(1000.0, 50.0), (1000.5, 50.0)]) is False
    assert runner.clock_advancing([(1000.0, 50.0), (1000.5, 50.1)]) is True


def test_clock_advancing_on_no_samples():
    assert runner.clock_advancing([]) is False


# --------------------------------------------------------------------------
# log scraping
# --------------------------------------------------------------------------
def test_parse_settle_log_reads_a_converged_run(tmp_path):
    path = tmp_path / "sim_bringup.log"
    path.write_text("settle: converged after 12.5 sim-s\n[6] offset=0.006 m\n")
    result = runner.parse_settle_log(str(path))
    assert result == {"converged": True, "sim_s": 12.5, "offset": 0.006}


def test_parse_settle_log_reads_a_capped_run(tmp_path):
    path = tmp_path / "sim_bringup.log"
    path.write_text("settle: capped after 40.0 sim-s\n")
    assert runner.parse_settle_log(str(path))["converged"] is False


def test_parse_settle_log_on_a_missing_file():
    assert runner.parse_settle_log("/nonexistent/sim_bringup.log") == {
        "converged": None,
        "sim_s": None,
        "offset": None,
    }


def test_scan_health_counts_crashes_and_levels(tmp_path):
    path = tmp_path / "console.log"
    path.write_text(
        "[ERROR] one\n[ERROR] two\n[FATAL] three\nterminate called after throwing\n",
    )
    health = runner.scan_health(str(path))
    assert health == {"crashes": 1, "fatal": 1, "error": 2}


def test_scan_health_on_a_missing_file():
    assert runner.scan_health("/nonexistent/console.log")["error"] == 0


# --------------------------------------------------------------------------
# settle-log format contract
# --------------------------------------------------------------------------
def _sim_bringup_source() -> str:
    with open(runner.lifecycle.sim_bringup_path()) as handle:
        return handle.read()


def test_sim_bringup_still_prints_the_settle_line():
    """The settle line is a contract between two files; pin it from both ends.

    sim_bringup builds it with an f-string, so the rendered text never appears
    in its source -- pin the template and the two state words it can carry.
    """
    source = _sim_bringup_source()
    assert "settle: {state} after {elapsed:.1f} sim-s" in source
    assert '"converged" if converged else "capped"' in source


def test_sim_bringup_still_prints_the_anchor_offset():
    assert "offset={off:.4f} m" in _sim_bringup_source()


@pytest.mark.parametrize(
    ("state", "converged"),
    [("converged", True), ("capped", False)],
)
def test_the_parser_reads_both_rendered_states(tmp_path, state, converged):
    # The exact text sim_bringup's template produces, both ways round.
    path = tmp_path / "sim_bringup.log"
    path.write_text(
        f"settle: {state} after {7.25:.1f} sim-s\n"
        f"    settled truth=(-7.700,13.970)  odom=(-7.694,13.972)  "
        f"offset={0.0063:.4f} m\n",
    )
    result = runner.parse_settle_log(str(path))
    assert result["converged"] is converged
    assert result["sim_s"] == 7.2
    assert result["offset"] == 0.0063
