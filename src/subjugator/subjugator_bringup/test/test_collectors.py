import pytest
from subjugator_bringup.task_runner import collectors as c

# (wall, sim) pairs: 10 sim seconds elapsed over 100 wall seconds -> RTF 0.1
CLOCK = [(1000.0, 50.0), (1050.0, 55.0), (1100.0, 60.0)]


def test_rtf():
    assert c.rtf(CLOCK) == pytest.approx(0.1)


def test_rtf_needs_two_samples():
    assert c.rtf([(1000.0, 50.0)]) is None


def test_wall_to_sim_interpolates():
    assert c.wall_to_sim(CLOCK, 1025.0) == pytest.approx(52.5)


def test_wall_to_sim_clamps_outside_range():
    assert c.wall_to_sim(CLOCK, 900.0) == pytest.approx(50.0)
    assert c.wall_to_sim(CLOCK, 2000.0) == pytest.approx(60.0)


def test_path_length():
    poses = [(0.0, 0.0, 0.0, 0.0), (1.0, 3.0, 4.0, 0.0), (2.0, 3.0, 4.0, 1.0)]
    assert c.path_length(poses) == pytest.approx(6.0)


def test_depth_range():
    poses = [(0.0, 0.0, 0.0, -0.31), (1.0, 0.0, 0.0, -1.92), (2.0, 0.0, 0.0, -0.8)]
    assert c.depth_range(poses) == (-0.31, -1.92)


def test_drift_max_and_end():
    odom = [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0)]
    truth = [(0.0, 0.5, 0.0, 0.0), (1.0, 1.02, 0.0, 0.0)]
    result = c.drift(odom, truth)
    assert result["max"] == pytest.approx(0.5)
    assert result["end"] == pytest.approx(0.02)


def test_stalled_when_clock_is_old():
    assert c.stalled(CLOCK, now_wall=1200.0, tol_s=60.0) is True
    assert c.stalled(CLOCK, now_wall=1120.0, tol_s=60.0) is False


def test_stalled_is_false_with_no_samples_yet():
    assert c.stalled([], now_wall=1200.0, tol_s=60.0) is False


def test_budget_exhausted():
    assert c.budget_exhausted(CLOCK, start_sim=50.0, budget_s=9.0) is True
    assert c.budget_exhausted(CLOCK, start_sim=50.0, budget_s=30.0) is False


# --- edge cases beyond the plan's tests ---------------------------------


def test_rtf_none_for_empty_and_for_zero_wall_span():
    assert c.rtf([]) is None
    # A paused sim can emit two samples with the same wall stamp; no division.
    assert c.rtf([(1000.0, 50.0), (1000.0, 50.0)]) is None


def test_rtf_of_a_stalled_sim_is_zero_not_none():
    # Wall advanced, sim did not: that is a real, reportable RTF of 0.0.
    assert c.rtf([(1000.0, 50.0), (1100.0, 50.0)]) == pytest.approx(0.0)


def test_wall_to_sim_none_without_samples():
    assert c.wall_to_sim([], 1025.0) is None


def test_wall_to_sim_single_sample_is_that_sample():
    assert c.wall_to_sim([(1000.0, 50.0)], 1234.0) == pytest.approx(50.0)


def test_wall_to_sim_hits_sample_stamps_exactly():
    assert c.wall_to_sim(CLOCK, 1050.0) == pytest.approx(55.0)


def test_wall_to_sim_interpolates_in_the_later_span():
    # Guards against only ever consulting the first pair.
    assert c.wall_to_sim(CLOCK, 1075.0) == pytest.approx(57.5)


def test_wall_to_sim_survives_duplicate_wall_stamps():
    clock = [(1000.0, 50.0), (1050.0, 55.0), (1050.0, 90.0), (1100.0, 95.0)]
    assert c.wall_to_sim(clock, 1050.0) == pytest.approx(55.0)
    assert c.wall_to_sim(clock, 1075.0) == pytest.approx(92.5)


def test_path_length_of_empty_or_single_pose_is_zero():
    assert c.path_length([]) == pytest.approx(0.0)
    assert c.path_length([(0.0, 1.0, 2.0, 3.0)]) == pytest.approx(0.0)


def test_path_length_counts_backtracking():
    # Distance travelled, not net displacement.
    poses = [(0.0, 0.0, 0.0, 0.0), (1.0, 2.0, 0.0, 0.0), (2.0, 0.0, 0.0, 0.0)]
    assert c.path_length(poses) == pytest.approx(4.0)


def test_depth_range_of_empty_poses():
    assert c.depth_range([]) == (0.0, 0.0)


def test_depth_range_single_pose_is_that_depth_twice():
    assert c.depth_range([(0.0, 0.0, 0.0, -0.35)]) == (-0.35, -0.35)


def test_depth_range_shallowest_is_the_max_z():
    # Sign convention: below the surface is negative, so shallowest == max z.
    poses = [(0.0, 0.0, 0.0, -1.9), (1.0, 0.0, 0.0, -0.35)]
    shallowest, deepest = c.depth_range(poses)
    assert shallowest == pytest.approx(-0.35)
    assert deepest == pytest.approx(-1.9)


def test_drift_is_horizontal_only():
    # A pure-z discrepancy is depth, not horizontal drift.
    odom = [(0.0, 0.0, 0.0, 0.0)]
    truth = [(0.0, 0.0, 0.0, -5.0)]
    result = c.drift(odom, truth)
    assert result["max"] == pytest.approx(0.0)
    assert result["end"] == pytest.approx(0.0)


def test_drift_matches_by_nearest_sim_time():
    # Truth is sampled at a different rate than odom; pairing must be by time.
    odom = [(0.0, 0.0, 0.0, 0.0), (10.0, 0.0, 0.0, 0.0), (20.0, 0.0, 0.0, 0.0)]
    truth = [(0.4, 3.0, 0.0, 0.0), (19.0, 0.0, 1.0, 0.0)]
    result = c.drift(odom, truth)
    # t=0 -> truth[0], 3.0 away.  t=10 and t=20 -> truth[1], 1.0 away.
    assert result["max"] == pytest.approx(3.0)
    assert result["end"] == pytest.approx(1.0)


def test_drift_none_when_either_series_is_empty():
    poses = [(0.0, 0.0, 0.0, 0.0)]
    assert c.drift([], poses) == {"max": None, "end": None}
    assert c.drift(poses, []) == {"max": None, "end": None}


def test_goal_error_is_three_dimensional():
    poses = [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 2.0, 2.0)]
    assert c.goal_error((1.0, 2.0, 0.0), poses) == pytest.approx(2.0)


def test_goal_error_none_without_a_goal_or_poses():
    assert c.goal_error(None, [(0.0, 0.0, 0.0, 0.0)]) is None
    assert c.goal_error((0.0, 0.0, 0.0), []) is None


def test_stalled_false_when_clock_stamp_is_in_the_future():
    assert c.stalled(CLOCK, now_wall=1000.0, tol_s=60.0) is False


def test_stalled_is_exclusive_at_the_tolerance():
    assert c.stalled(CLOCK, now_wall=1160.0, tol_s=60.0) is False
    assert c.stalled(CLOCK, now_wall=1160.1, tol_s=60.0) is True


def test_consumed_sim():
    assert c.consumed_sim(CLOCK, start_sim=50.0) == pytest.approx(10.0)
    assert c.consumed_sim([], start_sim=50.0) == pytest.approx(0.0)


def test_budget_exhausted_at_exactly_the_budget():
    assert c.budget_exhausted(CLOCK, start_sim=50.0, budget_s=10.0) is True


def test_budget_not_exhausted_before_any_clock_arrives():
    assert c.budget_exhausted([], start_sim=50.0, budget_s=30.0) is False
