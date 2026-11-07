import numpy as np
import pytest
from prop_fusion.projection import Lens, corners, unoccluded

# 640x480, 500 px focal length, centred.
LENS = Lens(
    np.array(
        [[500.0, 0.0, 320.0, 0.0], [0.0, 500.0, 240.0, 0.0], [0.0, 0.0, 1.0, 0.0]],
    ),
    640,
    480,
)


def box_at(x, y, z):
    return LENS.box(corners([x, y, z], [1.0, 1.0, 1.0]))


def test_box_ahead_projects_around_the_centre():
    left, top, right, bottom = box_at(0.0, 0.0, 5.0)
    assert left < 320 < right
    assert top < 240 < bottom
    assert right - left == pytest.approx(500 / 4.5)


def test_box_behind_the_camera_is_not_seen():
    assert box_at(0.0, 0.0, -5.0) is None


def test_box_off_the_side_is_not_seen():
    assert box_at(20.0, 0.0, 5.0) is None


def test_box_straddling_the_lens_spreads_to_its_edge():
    assert box_at(1.0, 0.0, 0.6)[2] == 640


def test_far_buoy_behind_a_near_one_sits_out():
    spans = [(300.0, 340.0), (310.0, 330.0), (100.0, 120.0)]
    assert unoccluded(spans, [3.0, 8.0, 8.0]) == [0, 2]


def test_buoy_mostly_clear_of_a_nearer_one_still_counts():
    assert unoccluded([(300.0, 340.0), (330.0, 370.0)], [3.0, 8.0]) == [0, 1]
