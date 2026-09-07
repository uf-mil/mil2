"""Tests for the terminal report, stats.json, and the exit-code contract.

Rendering tests are unusually easy to write badly: `assert "FAILURE" in text`
passes when the word turns up in an unrelated line, and `assert "0.041" in text`
passes when the number lands in the wrong column. So almost every assertion here
goes through `only_line()`, which pins *which* line a value appears on and fails
if it appears on more than one.
"""

import json

import pytest
from subjugator_bringup.task_runner import exitcodes, report

RUN = {
    "task": 5,
    "task_name": "Octagon: table, grasp, place",
    "stage": "full",
    "mission": "OctagonMission",
    "role": "survey_repair",
    "outcome": "FAILURE",
    "stop_reason": "mission exited normally",
    "sim_duration": 412.0,
    "wall_duration": 8040.0,
    "rtf": 0.051,
    "sim_budget": 1300.0,
    "stages": [
        {
            "name": "AcquireTable",
            "occurrence": 1,
            "result": "SUCCESS",
            "sim_duration": 38.0,
        },
        {
            "name": "CollectOneObject",
            "occurrence": 1,
            "result": "FAILURE",
            "sim_duration": 210.0,
        },
    ],
    # The two ends of the final unwind cascade. The report must annotate with
    # `innermost_node`; `last_node` is the useless outer end (see btlog.py).
    "last_node": "Sequence",
    "innermost_node": "ConfirmGraspByScale",
    "vehicle": {
        "path_length": 24.3,
        "depth_range": (-0.31, -1.92),
        "goal_error": 0.08,
        "anchor_offset": 0.006,
        "settle": {"converged": True, "sim_s": 12.0},
        "drift": {"max": 0.041, "end": 0.038},
    },
    "perception": {
        "topic": "/yolo_down/detections",
        "frames": 4038,
        "rate_hz": 9.8,
        "longest_dropout_s": 1.4,
        "classes": {
            "table": {
                "detections": 3412,
                "presence": 0.98,
                "mean_conf": 0.91,
                "min_conf": 0.62,
            },
        },
    },
    "ground_truth": {
        "role": "survey_repair",
        "basket": "warning",
        "basket_center": (-7.25, 13.6),
        "placed": 0,
        "targets": 2,
        "disagrees_with_bt": False,
        "objects": {
            "electric_box": {
                "verdict": "untouched",
                "displacement": 0.02,
                "max_z_rise": 0.0,
                "end": (-7.1, 13.89, -0.8),
            },
        },
    },
    "health": {"crashes": 0, "fatal": 0, "error": 3},
    "run_dir": "pool_tests/task_runs/task5_full_20260726T031500Z",
    "config": {"stage": "full", "start": "near", "device": "cpu"},
}

# A run the operator killed: the mission never printed a verdict, too few /clock
# samples for an RTF, no subtree ever terminated, no scorer ran. Every optional
# field is missing or None, which is exactly when a renderer crashes or leaks
# the string "None" into the report.
KILLED = {
    "task": 5,
    "task_name": "Octagon: table, grasp, place",
    "stage": "calib",
    "mission": "CenterCameraTest",
    "role": "survey_repair",
    "outcome": None,
    "stop_reason": "killed by operator (SIGINT)",
    "sim_duration": None,
    "wall_duration": None,
    "rtf": None,
    "sim_budget": 60.0,
    "stages": [],
    "last_node": None,
    "innermost_node": "CenterCamera",
    "vehicle": {},
    "perception": {},
    "ground_truth": {},
    "health": {},
    "run_dir": "pool_tests/task_runs/task5_calib_20260726T041500Z",
    "config": {"stage": "calib", "start": "over_table", "device": "cpu"},
}


def merged(run, **overrides):
    """A copy of `run` with top-level keys replaced."""
    return dict(run, **overrides)


def only_line(text, needle):
    """The single line containing `needle`. Fails on zero or several matches.

    This is what makes an assertion discriminating: it proves the value is on
    the line it belongs on, not merely somewhere in the report.
    """
    hits = [line for line in text.splitlines() if needle in line]
    assert len(hits) == 1, f"expected exactly one line with {needle!r}, got {hits}"
    return hits[0]


def line_index(text, needle):
    for index, line in enumerate(text.splitlines()):
        if needle in line:
            return index
    raise AssertionError(f"no line containing {needle!r}")


# --------------------------------------------------------------------------
# header and timing
# --------------------------------------------------------------------------


def test_header_has_outcome_and_measured_rtf():
    text = report.render(RUN)
    assert "FAILURE" in text
    assert "RTF 0.051 measured" in text
    assert "budget 1300" in text


def test_header_names_the_task_stage_mission_and_role():
    text = report.render(RUN)
    assert "Octagon: table, grasp, place" in only_line(text, "run_task 5")
    stage_line = only_line(text, "stage:")
    # the mission is the parenthetical, not the other way round
    assert "stage: full (OctagonMission)" in stage_line
    assert "role: survey_repair" in stage_line


def test_outcome_and_stop_reason_share_a_line():
    line = only_line(report.render(RUN), "outcome:")
    assert "FAILURE" in line
    assert "mission exited normally" in line


def test_outcome_line_shows_the_exit_code_the_process_will_use():
    assert "exit 1" in only_line(report.render(RUN), "outcome:")
    assert "exit 0" in only_line(
        report.render(merged(RUN, outcome="SUCCESS")),
        "outcome:",
    )


def test_duration_line_carries_sim_wall_rtf_and_budget_together():
    line = only_line(report.render(RUN), "duration:")
    assert "412 sim-s" in line
    assert "2h14m wall" in line
    assert "RTF 0.051 measured" in line
    assert "budget 1300 sim-s" in line


def test_never_projects_a_wall_time():
    text = report.render(RUN).lower()
    for banned in ("estimated wall", "expected wall", "projected", "should take"):
        assert banned not in text


def test_a_killed_run_says_no_verdict_and_never_leaks_the_word_none():
    text = report.render(KILLED)
    assert "NO VERDICT" in only_line(text, "outcome:")
    assert "None" not in text


def test_missing_timings_render_as_dashes_not_zeros():
    line = only_line(report.render(KILLED), "duration:")
    assert "-- sim-s" in line
    assert "-- wall" in line
    assert "RTF -- measured" in line
    # the budget is known even when nothing ran
    assert "budget 60 sim-s" in line


def test_report_is_fenced_by_horizontal_rules():
    lines = report.render(RUN).splitlines()
    assert lines[0] == report.HR
    assert lines[-1] == report.HR


@pytest.mark.parametrize(
    ("seconds", "expected"),
    [
        (None, "--"),
        (0, "0s"),
        (42, "42s"),
        (59.9, "59s"),
        (60, "1m00s"),
        (61, "1m01s"),
        (200, "3m20s"),
        (3599, "59m59s"),
        (3600, "1h00m"),
        (3660, "1h01m"),
        (8040, "2h14m"),
    ],
)
def test_hms_covers_seconds_minutes_and_hours(seconds, expected):
    assert report._hms(seconds) == expected


# --------------------------------------------------------------------------
# stage table and the stop annotation
# --------------------------------------------------------------------------


def test_stage_table_uses_sim_seconds():
    text = report.render(RUN)
    assert "AcquireTable" in text
    assert "38 sim-s" in text


def test_each_stage_row_pairs_its_own_result_and_duration():
    text = report.render(RUN)
    first = only_line(text, "AcquireTable")
    assert "SUCCESS" in first
    assert "38 sim-s" in first
    second = only_line(text, "CollectOneObject")
    assert "FAILURE" in second
    assert "210 sim-s" in second


def test_repeat_occurrences_are_numbered_and_first_ones_are_not():
    text = report.render(
        merged(
            RUN,
            stages=[
                dict(RUN["stages"][1], occurrence=1),
                dict(RUN["stages"][1], occurrence=2, result="SUCCESS"),
            ],
        ),
    )
    assert "CollectOneObject#2" in text
    assert "CollectOneObject#1" not in text


def test_stop_annotation_names_the_innermost_node_not_the_last_node():
    text = report.render(RUN)
    assert "ConfirmGraspByScale" in text
    assert "Sequence" not in text


def test_stop_annotation_sits_on_the_last_stage_row():
    text = report.render(RUN)
    assert "stopped here" in only_line(text, "CollectOneObject")
    assert "stopped" not in only_line(text, "AcquireTable")


def test_stop_annotation_renders_with_no_stages_at_all():
    # The `calib` case: CenterCameraTest is a RosTimeout around a leaf, so no
    # subtree ever terminates and the annotation is the only thing the report
    # can say about what happened.
    text = report.render(KILLED)
    assert "behavior tree" in text
    assert "CenterCamera" in only_line(text, "stopped")


def test_stage_section_is_omitted_when_there_is_nothing_to_say():
    text = report.render(merged(KILLED, innermost_node=None))
    assert "behavior tree" not in text
    assert "stopped" not in text


def test_stage_rows_render_without_an_annotation_when_the_node_is_unknown():
    text = report.render(merged(RUN, innermost_node=None))
    assert "AcquireTable" in text
    assert "stopped" not in text


def test_annotation_does_not_repeat_the_stage_name_when_they_are_the_same():
    text = report.render(merged(RUN, innermost_node="CollectOneObject"))
    line = only_line(text, "CollectOneObject")
    assert "stopped here" in line
    assert "in CollectOneObject" not in line


def test_stage_duration_missing_renders_as_a_dash():
    stages = [dict(RUN["stages"][0], sim_duration=None)]
    line = only_line(report.render(merged(RUN, stages=stages)), "AcquireTable")
    assert "-- sim-s" in line


# --------------------------------------------------------------------------
# vehicle
# --------------------------------------------------------------------------


def test_vehicle_section_reports_settle_and_drift():
    text = report.render(RUN)
    assert "settle converged" in text
    assert "0.041" in text


def test_vehicle_motion_line_keeps_path_depth_and_goal_error_apart():
    line = only_line(report.render(RUN), "path 24.3 m")
    assert "-0.31 .. -1.92 m" in line
    assert "final goal error 0.08 m" in line


def test_vehicle_anchor_line_pairs_the_offset_with_the_settle_verdict():
    line = only_line(report.render(RUN), "anchor offset")
    assert "0.006 m" in line
    assert "settle converged" in line
    assert "12 sim-s" in line


def test_settle_that_hit_its_cap_is_reported_as_capped():
    vehicle = dict(RUN["vehicle"], settle={"converged": False, "sim_s": 40.0})
    line = only_line(report.render(merged(RUN, vehicle=vehicle)), "anchor offset")
    assert "settle capped" in line
    assert "converged" not in line


def test_drift_max_and_end_are_not_swapped():
    line = only_line(report.render(RUN), "drift")
    assert "max 0.041 m" in line
    assert "end 0.038 m" in line


def test_vehicle_none_fields_degrade_to_dashes():
    vehicle = dict(
        RUN["vehicle"],
        goal_error=None,
        drift={"max": None, "end": None},
        depth_range=None,
    )
    text = report.render(merged(RUN, vehicle=vehicle))
    assert "final goal error -- m" in only_line(text, "path 24.3 m")
    assert "max -- m, end -- m" in only_line(text, "drift")
    assert "None" not in text


def test_vehicle_section_is_omitted_when_no_motion_was_collected():
    text = report.render(KILLED)
    assert "vehicle" not in text
    assert "anchor offset" not in text


# --------------------------------------------------------------------------
# perception
# --------------------------------------------------------------------------


def test_perception_table_has_a_header_row():
    text = report.render(RUN)
    assert "detections" in text and "present" in text and "mean conf" in text


def test_perception_header_row_names_all_four_columns_in_order():
    header = only_line(report.render(RUN), "mean conf")
    assert "class" in header
    positions = [
        header.index(column)
        for column in ("class", "detections", "present", "mean conf", "min conf")
    ]
    assert positions == sorted(positions)


def test_perception_summary_line_carries_topic_rate_frames_and_dropout():
    line = only_line(report.render(RUN), "/yolo_down/detections")
    assert "9.8 Hz" in line
    assert "4038 frames" in line
    assert "longest dropout 1.4 s" in line


def test_perception_class_row_holds_that_classs_own_numbers():
    row = only_line(report.render(RUN), "table ")
    assert "3412" in row
    assert "0.91" in row
    assert "0.62" in row


def test_perception_values_line_up_under_their_own_headers():
    # The columns are right-aligned, so a value and its header end at the same
    # index. This is what catches two numbers of the same shape being swapped
    # (mean conf against min conf), which a plain `in text` check cannot see.
    text = report.render(RUN)
    header = only_line(text, "mean conf")
    row = only_line(text, "3412")
    for column, value in (
        ("detections", "3412"),
        ("present", "98%"),
        ("mean conf", "0.91"),
        ("min conf", "0.62"),
    ):
        header_end = header.index(column) + len(column)
        value_end = row.index(value) + len(value)
        assert header_end == value_end, f"{value!r} is not under {column!r}"


def test_presence_is_rendered_as_a_percentage_not_a_fraction():
    row = only_line(report.render(RUN), "3412")
    assert "98%" in row
    assert "0.98" not in row


def test_perception_rows_stay_in_their_own_columns():
    classes = {
        "table": {
            "detections": 3412,
            "presence": 0.98,
            "mean_conf": 0.91,
            "min_conf": 0.62,
        },
        # not electric_box: that name is already in RUN's ground-truth rows, and
        # only_line() must be able to isolate the perception row.
        "nut_cylinder": {
            "detections": 2870,
            "presence": 0.82,
            "mean_conf": 0.88,
            "min_conf": 0.41,
        },
    }
    perception = dict(RUN["perception"], classes=classes)
    text = report.render(merged(RUN, perception=perception))
    box = only_line(text, "nut_cylinder")
    assert "2870" in box
    assert "82%" in box
    assert "0.88" in box
    assert "0.41" in box
    assert "3412" not in box


def test_perception_none_rate_and_dropout_degrade_to_dashes():
    perception = dict(RUN["perception"], rate_hz=None, longest_dropout_s=None)
    text = report.render(merged(RUN, perception=perception))
    line = only_line(text, "/yolo_down/detections")
    assert "-- Hz" in line
    assert "longest dropout -- s" in line
    assert "None" not in text


def test_a_topic_that_saw_nothing_still_says_so():
    perception = dict(RUN["perception"], frames=0, classes={})
    text = report.render(merged(RUN, perception=perception))
    assert "/yolo_down/detections" in text
    assert "no detections" in text
    assert "mean conf" not in text


def test_a_partial_class_entry_dashes_out_instead_of_crashing():
    classes = {"table": {"detections": None, "presence": None, "mean_conf": None}}
    text = report.render(
        merged(RUN, perception=dict(RUN["perception"], classes=classes)),
    )
    row = only_line(text, "table ")
    assert row.count("--") == 4  # one per numeric column, min_conf included
    assert "None" not in text


def test_perception_section_is_omitted_when_no_probe_ran():
    text = report.render(KILLED)
    assert "perception" not in text


# --------------------------------------------------------------------------
# ground truth
# --------------------------------------------------------------------------


def test_ground_truth_object_row_carries_its_own_verdict():
    row = only_line(report.render(RUN), "electric_box")
    assert "moved 0.02 m" in row
    assert "rose 0.00 m" in row
    assert "untouched" in row


def test_ground_truth_verdict_line_names_count_basket_and_centre():
    line = only_line(report.render(RUN), "verdict:")
    assert "0/2 placed" in line
    assert "warning basket" in line
    assert "(-7.25, 13.60)" in line


def test_basket_centre_is_dropped_when_unknown():
    truth = dict(RUN["ground_truth"], basket_center=None)
    text = report.render(merged(RUN, ground_truth=truth))
    line = only_line(text, "verdict:")
    assert "warning basket" in line
    assert "(" not in line
    assert "None" not in text


def test_ground_truth_object_with_no_pose_still_renders():
    truth = dict(
        RUN["ground_truth"],
        objects={
            "nut_cylinder": {
                "verdict": "absent",
                "displacement": None,
                "max_z_rise": None,
                "end": None,
            },
        },
    )
    text = report.render(merged(RUN, ground_truth=truth))
    row = only_line(text, "nut_cylinder")
    assert "absent" in row
    assert "None" not in text


def test_disagreement_is_loud():
    run = dict(RUN)
    run["outcome"] = "SUCCESS"
    run["ground_truth"] = dict(RUN["ground_truth"], disagrees_with_bt=True)
    assert "WARNING" in report.render(run)


def test_disagreement_line_says_what_the_contradiction_is():
    truth = dict(RUN["ground_truth"], disagrees_with_bt=True)
    text = report.render(merged(RUN, outcome="SUCCESS", ground_truth=truth))
    line = only_line(text, "WARNING")
    assert "SUCCESS" in line
    assert "placed" in line
    # loud means it stands out from the object rows around it
    assert line.strip().startswith("WARNING")
    assert line_index(text, "WARNING") > line_index(text, "verdict:")


def test_agreement_is_quiet():
    assert "WARNING" not in report.render(RUN)


def test_ground_truth_section_is_omitted_when_no_scorer_ran():
    text = report.render(KILLED)
    assert "ground truth" not in text
    assert "verdict:" not in text


# --------------------------------------------------------------------------
# health and artifacts
# --------------------------------------------------------------------------


def test_health_line_carries_crash_fatal_and_error_counts():
    line = only_line(report.render(RUN), "health")
    assert "0 crashes" in line
    assert "0 FATAL" in line
    assert "3 ERROR" in line


def test_missing_health_counts_default_to_zero():
    line = only_line(report.render(KILLED), "health")
    assert "0 crashes" in line
    assert "0 FATAL" in line
    assert "0 ERROR" in line


def test_artifacts_line_points_at_the_run_dir():
    assert RUN["run_dir"] in only_line(report.render(RUN), "artifacts")


# --------------------------------------------------------------------------
# stats.json
# --------------------------------------------------------------------------


def test_stats_json_round_trips(tmp_path):
    path = tmp_path / "stats.json"
    report.write_json(RUN, str(path))
    loaded = json.loads(path.read_text())
    assert loaded["mission"] == "OctagonMission"
    assert loaded["config"]["start"] == "near"


def test_stats_json_keeps_the_whole_resolved_config(tmp_path):
    path = tmp_path / "stats.json"
    report.write_json(RUN, str(path))
    loaded = json.loads(path.read_text())
    assert loaded["config"] == RUN["config"]
    assert loaded["sim_budget"] == 1300.0
    assert loaded["stages"][1]["name"] == "CollectOneObject"


def test_stats_json_turns_tuples_into_lists(tmp_path):
    path = tmp_path / "stats.json"
    report.write_json(RUN, str(path))
    loaded = json.loads(path.read_text())
    assert loaded["vehicle"]["depth_range"] == [-0.31, -1.92]
    assert loaded["ground_truth"]["basket_center"] == [-7.25, 13.6]


def test_stats_json_survives_an_unserialisable_value(tmp_path):
    path = tmp_path / "stats.json"
    report.write_json(merged(RUN, weird={1, 2}), str(path))
    assert json.loads(path.read_text())["weird"]


def test_stats_json_writes_a_killed_run_too(tmp_path):
    path = tmp_path / "stats.json"
    report.write_json(KILLED, str(path))
    loaded = json.loads(path.read_text())
    assert loaded["outcome"] is None
    assert loaded["rtf"] is None


# --------------------------------------------------------------------------
# exit codes
# --------------------------------------------------------------------------


def test_exit_codes():
    assert report.exit_code({"outcome": "SUCCESS", "harness_error": None}) == 0
    assert report.exit_code({"outcome": "FAILURE", "harness_error": None}) == 1
    assert report.exit_code({"outcome": None, "harness_error": None}) == 1
    assert report.exit_code({"outcome": None, "harness_error": "gate timeout"}) == 2


def test_exit_codes_come_from_the_shared_constants():
    assert report.exit_code({"outcome": "SUCCESS"}) == exitcodes.EXIT_OK
    assert report.exit_code({"outcome": "FAILURE"}) == exitcodes.EXIT_MISSION_FAILED
    assert report.exit_code({"harness_error": "sim never came up"}) == (
        exitcodes.EXIT_HARNESS
    )


def test_a_budget_exhausted_run_exits_one():
    assert (
        report.exit_code(
            {"outcome": None, "stop_reason": "sim budget exhausted"},
        )
        == exitcodes.EXIT_MISSION_FAILED
    )


def test_a_harness_error_outranks_a_success_verdict():
    # A rig that broke after the tree won is still a rig that broke; exit 2 must
    # not be masked by the mission's own verdict.
    assert (
        report.exit_code({"outcome": "SUCCESS", "harness_error": "clock stalled"})
        == exitcodes.EXIT_HARNESS
    )
