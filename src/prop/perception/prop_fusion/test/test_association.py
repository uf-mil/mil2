import numpy as np
import pytest
from prop_fusion.association import Votes, associate, corners, project, unoccluded

# 640x480, 500 px focal length, centred.
P = np.array([[500.0, 0.0, 320.0, 0.0], [0.0, 500.0, 240.0, 0.0], [0.0, 0.0, 1.0, 0.0]])


def test_box_ahead_projects_around_the_centre():
    box = project(corners([0.0, 0.0, 5.0], [1.0, 1.0, 1.0], np.eye(3)), P, 640, 480)
    left, top, right, bottom = box
    assert left < 320 < right
    assert top < 240 < bottom
    # Near face at 4.5 m, 1 m wide.
    assert right - left == pytest.approx(500 / 4.5, rel=1e-6)


def test_box_behind_the_camera_is_not_seen():
    assert (
        project(corners([0.0, 0.0, -5.0], [1.0, 1.0, 1.0], np.eye(3)), P, 640, 480)
        is None
    )


def test_box_off_the_side_of_the_image_is_not_seen():
    assert (
        project(corners([20.0, 0.0, 5.0], [1.0, 1.0, 1.0], np.eye(3)), P, 640, 480)
        is None
    )


def test_box_straddling_the_lens_spreads_to_the_edge_it_is_on():
    box = project(corners([1.0, 0.0, 0.6], [1.0, 1.0, 1.0], np.eye(3)), P, 640, 480)
    assert box is not None
    assert box[2] == 640


def test_far_buoy_behind_a_near_one_sits_out():
    spans = [(300.0, 340.0), (310.0, 330.0), (100.0, 120.0)]
    assert unoccluded(spans, [3.0, 8.0, 8.0]) == [0, 2]


def test_buoy_mostly_clear_of_a_nearer_one_still_counts():
    assert unoccluded([(300.0, 340.0), (330.0, 370.0)], [3.0, 8.0]) == [0, 1]


def test_associate_is_one_to_one_and_nearest():
    tracks = [(100.0, 120.0), (300.0, 320.0)]
    detections = [(302.0, 318.0), (98.0, 124.0)]
    assert sorted(associate(tracks, detections, 10.0)) == [(0, 1), (1, 0)]


def test_associate_leaves_a_far_detection_unmatched():
    assert associate([(100.0, 120.0)], [(200.0, 220.0)], 10.0) == []


def test_associate_takes_a_detection_just_outside_the_span_within_tolerance():
    assert associate([(100.0, 120.0)], [(125.0, 140.0)], 10.0) == [(0, 0)]


def test_one_detection_goes_to_one_track():
    assert associate([(100.0, 120.0), (105.0, 125.0)], [(104.0, 124.0)], 10.0) == [
        (1, 0),
    ]


def test_votes_need_a_clear_lead():
    votes = Votes(margin=2.0)
    for _ in range(3):
        votes.add(7, "red_buoy", 0.8)
    assert votes.label(7) == "red_buoy"
    for _ in range(2):
        votes.add(7, "green_buoy", 0.8)
    assert votes.label(7) == ""
    assert votes.label(8) == ""


def test_votes_forget_dropped_tracks():
    votes = Votes(margin=0.5)
    votes.add(1, "red_buoy", 1.0)
    votes.add(2, "green_buoy", 1.0)
    votes.keep({2})
    assert votes.label(1) == ""
    assert votes.label(2) == "green_buoy"
