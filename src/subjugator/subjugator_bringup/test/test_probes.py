import pytest
from subjugator_bringup.task_runner import probes

# (sim_s, [(label, confidence), ...]) -- one entry per detection message.
FRAMES = [
    (0.0, [("table", 0.9), ("electric_box", 0.8)]),
    (0.1, [("table", 0.7)]),
    (0.6, [("table", 0.8), ("table", 0.6)]),
]


def test_frame_count_and_rate():
    summary = probes.summarize_detections(FRAMES)
    assert summary["frames"] == 3
    assert summary["rate_hz"] == pytest.approx(3 / 0.6)


def test_longest_dropout():
    assert probes.summarize_detections(FRAMES)["longest_dropout_s"] == pytest.approx(
        0.5,
    )


def test_per_class_counts_and_presence():
    table = probes.summarize_detections(FRAMES)["classes"]["table"]
    assert table["detections"] == 4  # two in the last frame
    assert table["present_frames"] == 3  # but present in three frames
    assert table["presence"] == pytest.approx(1.0)


def test_per_class_confidence():
    table = probes.summarize_detections(FRAMES)["classes"]["table"]
    assert table["mean_conf"] == pytest.approx((0.9 + 0.7 + 0.8 + 0.6) / 4)
    assert table["min_conf"] == pytest.approx(0.6)


def test_empty_detections_are_safe():
    summary = probes.summarize_detections([])
    assert summary["frames"] == 0
    assert summary["classes"] == {}


POSES = {
    "electric_box": [
        (0.0, -7.1, 13.89, -0.80),
        (5.0, -7.1, 13.89, -0.60),
        (9.0, -7.25, 13.60, -0.90),
    ],
    "nut_cylinder": [(0.0, -7.3, 13.9, -0.80), (9.0, -7.3, 13.9, -0.80)],
}


def test_model_displacement_and_lift():
    summary = probes.summarize_model_poses(POSES)
    box = summary["electric_box"]
    assert box["end"] == (-7.25, 13.60, -0.90)
    assert box["max_z_rise"] == pytest.approx(0.20)
    assert box["displacement"] > 0.3


def test_untouched_model_reports_zero_displacement():
    nut = probes.summarize_model_poses(POSES)["nut_cylinder"]
    assert nut["displacement"] == pytest.approx(0.0)
    assert nut["max_z_rise"] == pytest.approx(0.0)


def test_missing_model_is_reported_absent():
    summary = probes.summarize_model_poses({"ghost": []})
    assert summary["ghost"]["present"] is False


# --- strengthened versions of the plan's tests --------------------------
#
# The plan's own fixtures happen not to discriminate between several
# plausible implementations; the tests below pin down what the summaries
# actually have to mean.


# Three 0.9s and one 0.1: mean 0.7, median 0.9, mean-of-distinct-values 0.5.
# The plan's FRAMES use four distinct confidences whose mean and median are
# both 0.75, so median or set-based implementations pass it unnoticed.
SKEWED = [
    (0.0, [("pill_cylinder", 0.9)]),
    (1.0, [("pill_cylinder", 0.9)]),
    (2.0, [("pill_cylinder", 0.9)]),
    (3.0, [("pill_cylinder", 0.1)]),
]


def test_mean_conf_is_the_mean_not_the_median():
    # A median would read 0.9 here and a mean over distinct values 0.5, and
    # both of those agree with the plan's fixture at 0.75.
    pill = probes.summarize_detections(SKEWED)["classes"]["pill_cylinder"]
    assert pill["mean_conf"] == pytest.approx(0.7)
    assert pill["min_conf"] == pytest.approx(0.1)


def test_repeat_detections_in_one_frame_contribute_every_confidence():
    # The 0.6 of the doubled last frame must reach mean_conf and min_conf,
    # not be dropped as a second sighting of an already-seen class.
    table = probes.summarize_detections(FRAMES)["classes"]["table"]
    assert table["min_conf"] == pytest.approx(0.6)
    assert table["mean_conf"] == pytest.approx(0.75)


def test_presence_is_a_fraction_of_all_frames():
    # electric_box appears in one frame out of three. Only this class pins
    # presence to len(frames); "table" is in every frame, so its presence of
    # 1.0 is consistent with several wrong denominators.
    box = probes.summarize_detections(FRAMES)["classes"]["electric_box"]
    assert box["detections"] == 1
    assert box["present_frames"] == 1
    assert box["presence"] == pytest.approx(1 / 3)


def test_longest_dropout_is_not_merely_the_last_gap():
    frames = [(0.0, [("table", 0.9)]), (5.0, [("table", 0.9)]), (5.1, [("table", 0.9)])]
    assert probes.summarize_detections(frames)["longest_dropout_s"] == pytest.approx(
        5.0,
    )


def test_longest_dropout_is_a_gap_not_the_whole_span():
    # Evenly spaced frames: the span is 0.6 but no dropout exceeds 0.2.
    frames = [(t / 5, [("table", 0.9)]) for t in range(4)]
    assert probes.summarize_detections(frames)["longest_dropout_s"] == pytest.approx(
        0.2,
    )


def test_empty_detections_report_no_rate_or_dropout():
    summary = probes.summarize_detections([])
    assert summary["rate_hz"] is None
    assert summary["longest_dropout_s"] is None


def test_model_start_is_reported():
    box = probes.summarize_model_poses(POSES)["electric_box"]
    assert box["start"] == (-7.1, 13.89, -0.80)
    assert box["present"] is True


def test_model_displacement_is_start_to_end():
    box = probes.summarize_model_poses(POSES)["electric_box"]
    assert box["displacement"] == pytest.approx(0.3414674, abs=1e-6)


def test_max_z_rise_is_the_peak_not_the_final_height():
    # The box is lifted 0.20 then dropped below where it started; the lift is
    # what the ground-truth scorer needs to see, so the peak is what counts.
    box = probes.summarize_model_poses(POSES)["electric_box"]
    assert box["max_z_rise"] == pytest.approx(0.20)
    assert box["end"][2] < box["start"][2]


def test_untouched_model_start_equals_end():
    nut = probes.summarize_model_poses(POSES)["nut_cylinder"]
    assert nut["start"] == nut["end"] == (-7.3, 13.9, -0.8)
    assert nut["max_displacement"] == pytest.approx(0.0)


# --- edge cases beyond the plan's tests ---------------------------------


def test_a_frame_with_no_detections_still_counts_as_a_frame():
    # An empty detection message is perception saying "nothing here", which is
    # a frame; counting only non-empty frames would inflate every presence.
    frames = [(0.0, []), (1.0, [("table", 0.5)])]
    summary = probes.summarize_detections(frames)
    assert summary["frames"] == 2
    assert summary["rate_hz"] == pytest.approx(2.0)
    assert summary["longest_dropout_s"] == pytest.approx(1.0)
    assert summary["classes"]["table"]["presence"] == pytest.approx(0.5)


def test_all_empty_frames_report_no_classes():
    summary = probes.summarize_detections([(0.0, []), (1.0, [])])
    assert summary["frames"] == 2
    assert summary["classes"] == {}


def test_single_frame_has_no_rate_and_no_dropout():
    # One sample gives a zero span; a rate would be a division by zero.
    summary = probes.summarize_detections([(4.0, [("table", 0.9)])])
    assert summary["frames"] == 1
    assert summary["rate_hz"] is None
    assert summary["longest_dropout_s"] == pytest.approx(0.0)


def test_zero_span_across_several_frames_has_no_rate():
    # Several messages stamped with the same sim time -- possible when /clock
    # is coarser than the detector -- must not divide by zero.
    frames = [(2.0, [("table", 0.9)]), (2.0, [("table", 0.8)])]
    summary = probes.summarize_detections(frames)
    assert summary["frames"] == 2
    assert summary["rate_hz"] is None
    assert summary["longest_dropout_s"] == pytest.approx(0.0)


def test_summarize_detections_does_not_mutate_its_input():
    frames = [(0.0, [("table", 0.9)])]
    probes.summarize_detections(frames)
    assert frames == [(0.0, [("table", 0.9)])]


def test_class_summary_carries_no_leftover_confidence_list():
    # The report and stats.json only want the aggregates; a raw sample list
    # would balloon stats.json across a 400 s run.
    table = probes.summarize_detections(FRAMES)["classes"]["table"]
    assert set(table) == {
        "detections",
        "present_frames",
        "presence",
        "mean_conf",
        "min_conf",
    }


def test_classes_are_kept_separate():
    summary = probes.summarize_detections(FRAMES)
    assert set(summary["classes"]) == {"table", "electric_box"}
    assert summary["classes"]["electric_box"]["mean_conf"] == pytest.approx(0.8)


def test_max_displacement_differs_from_displacement_on_a_round_trip():
    # A model knocked off the table and pushed back would report a
    # displacement of zero; max_displacement is what catches the excursion.
    poses = {
        "wanderer": [(0.0, 0.0, 0.0, 0.0), (1.0, 3.0, 4.0, 0.0), (2.0, 0.0, 0.0, 0.0)],
    }
    wanderer = probes.summarize_model_poses(poses)["wanderer"]
    assert wanderer["displacement"] == pytest.approx(0.0)
    assert wanderer["max_displacement"] == pytest.approx(5.0)


def test_max_displacement_is_measured_from_the_start_pose():
    # Out to 10 and part-way back. Anchored at the start the excursion is 10;
    # anchored at the final pose it would read 7, and "how far did this object
    # get from where it was placed" is the question the scorer asks.
    poses = {
        "excursion": [
            (0.0, 0.0, 0.0, 0.0),
            (1.0, 10.0, 0.0, 0.0),
            (2.0, 3.0, 0.0, 0.0),
        ],
    }
    excursion = probes.summarize_model_poses(poses)["excursion"]
    assert excursion["displacement"] == pytest.approx(3.0)
    assert excursion["max_displacement"] == pytest.approx(10.0)


def test_displacement_is_three_dimensional():
    # A straight lift is displacement too -- unlike odom drift, which is
    # deliberately horizontal-only.
    poses = {"riser": [(0.0, 1.0, 2.0, -1.0), (1.0, 1.0, 2.0, 1.0)]}
    riser = probes.summarize_model_poses(poses)["riser"]
    assert riser["displacement"] == pytest.approx(2.0)
    assert riser["max_displacement"] == pytest.approx(2.0)
    assert riser["max_z_rise"] == pytest.approx(2.0)


def test_a_model_that_only_sinks_reports_no_rise():
    # max_z_rise is a lift height, floored at zero by the starting sample; a
    # dropped object reports 0.0 rather than a negative "rise".
    poses = {"sinker": [(0.0, 0.0, 0.0, -0.8), (1.0, 0.0, 0.0, -1.5)]}
    sinker = probes.summarize_model_poses(poses)["sinker"]
    assert sinker["max_z_rise"] == pytest.approx(0.0)
    assert sinker["displacement"] == pytest.approx(0.7)


def test_single_sample_model_is_present_and_still():
    poses = {"lonely": [(3.0, 1.0, 2.0, 3.0)]}
    lonely = probes.summarize_model_poses(poses)["lonely"]
    assert lonely["present"] is True
    assert lonely["start"] == lonely["end"] == (1.0, 2.0, 3.0)
    assert lonely["displacement"] == pytest.approx(0.0)
    assert lonely["max_displacement"] == pytest.approx(0.0)
    assert lonely["max_z_rise"] == pytest.approx(0.0)


def test_absent_model_carries_no_pose_numbers():
    # Consumers must branch on `present`; zeros would read as "never moved".
    ghost = probes.summarize_model_poses({"ghost": []})["ghost"]
    assert ghost == {"present": False}


def test_absent_model_does_not_hide_the_others():
    summary = probes.summarize_model_poses({"ghost": [], **POSES})
    assert summary["ghost"]["present"] is False
    assert summary["electric_box"]["present"] is True
    assert summary["nut_cylinder"]["present"] is True


def test_no_models_summarizes_to_nothing():
    assert probes.summarize_model_poses({}) == {}


def test_positions_are_rounded_for_reporting():
    poses = {"jitter": [(0.0, 1.23456, -2.98765, 0.00051), (1.0, 0.0, 0.0, 0.0)]}
    jitter = probes.summarize_model_poses(poses)["jitter"]
    assert jitter["start"] == (1.235, -2.988, 0.001)
    assert isinstance(jitter["start"], tuple)


def test_displacement_is_not_rounded_away():
    # Rounding the reported corners is fine; rounding the maths is not -- a
    # few-millimetre creep still has to show up as a nonzero number.
    poses = {"creep": [(0.0, 0.0, 0.0, 0.0), (1.0, 0.0004, 0.0, 0.0)]}
    creep = probes.summarize_model_poses(poses)["creep"]
    assert creep["displacement"] == pytest.approx(0.0004)
