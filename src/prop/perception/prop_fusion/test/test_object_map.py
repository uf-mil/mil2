import numpy as np
import pytest
from prop_fusion.object_map import ObjectMap, Window

WINDOW = Window(-10.0, -10.0, 0.25)
BUOY = [(2.0, 1.0), (2.25, 1.0)]


def grid_with(points, evidence=100, window=WINDOW):
    cells = np.zeros((80, 80), dtype=np.int8)
    for x, y in points:
        row = int((y - window.y) // window.resolution)
        column = int((x - window.x) // window.resolution)
        cells[row, column] = evidence
    return cells


def new_map():
    return ObjectMap(occupied=60, label_radius=0.6, max_cluster_size=2.5)


def map_with_label(points, label="red_buoy"):
    object_map = new_map()
    object_map.update_grid(grid_with(points), WINDOW)
    object_map.add_label(label, *points[0])
    return object_map


def test_a_box_names_the_cluster_it_sits_on():
    found = map_with_label(BUOY).objects()
    assert [o.label for o in found] == ["red_buoy"]
    assert (found[0].x, found[0].y) == pytest.approx((2.25, 1.125))
    assert (found[0].size_x, found[0].size_y) == pytest.approx((0.5, 0.25))


def test_a_box_with_nothing_solid_near_it_names_nothing():
    object_map = map_with_label(BUOY)
    object_map.add_label("green_buoy", -5.0, -5.0)
    assert [o.label for o in object_map.objects()] == ["red_buoy"]


def test_the_closest_cluster_takes_the_label():
    object_map = new_map()
    object_map.update_grid(grid_with([(0.0, 0.0), (1.0, 0.0)]), WINDOW)
    object_map.add_label("red_buoy", 0.7, 0.0)
    found = object_map.objects()
    assert len(found) == 1
    assert found[0].x == pytest.approx(1.125)


def test_a_label_outlives_its_cells_stopping_being_solid():
    object_map = map_with_label(BUOY)
    object_map.update_grid(grid_with(BUOY, evidence=30), WINDOW)
    assert [o.label for o in object_map.objects()] == ["red_buoy"]


def test_a_label_goes_when_its_cells_have_no_evidence():
    object_map = map_with_label(BUOY)
    object_map.update_grid(grid_with([]), WINDOW)
    assert object_map.objects() == []

    object_map.update_grid(grid_with(BUOY), WINDOW)
    assert object_map.objects() == []


def test_a_newer_label_replaces_an_older_one():
    object_map = map_with_label(BUOY)
    object_map.add_label("green_buoy", *BUOY[0])
    assert [o.label for o in object_map.objects()] == ["green_buoy"]


def test_new_cells_of_an_object_take_its_label():
    object_map = map_with_label(BUOY)
    object_map.update_grid(grid_with([*BUOY, (2.5, 1.0)]), WINDOW)
    assert object_map.objects()[0].size_x == pytest.approx(0.75)


def test_a_cluster_too_wide_to_be_a_buoy_takes_no_label():
    dock = [(0.0, y) for y in np.arange(-5.0, 5.0, 0.25)]
    object_map = new_map()
    object_map.update_grid(grid_with(dock), WINDOW)
    object_map.add_label("red_buoy", 0.0, 0.0)
    assert object_map.objects() == []


def test_a_label_stays_put_when_its_buoy_joins_a_wider_cluster():
    object_map = map_with_label(BUOY)
    before = object_map.objects()

    dock = [(x, 1.0) for x in np.arange(2.5, 8.0, 0.25)]
    object_map.update_grid(grid_with([*BUOY, *dock]), WINDOW)
    assert object_map.objects() == before


def test_a_label_stays_in_place_as_the_window_moves():
    object_map = map_with_label(BUOY)
    before = object_map.objects()

    moved = Window(WINDOW.x + 2.0, WINDOW.y - 1.0, WINDOW.resolution)
    object_map.update_grid(grid_with(BUOY, window=moved), moved)
    assert object_map.objects() == before


def test_a_label_goes_when_the_window_leaves_it_behind():
    object_map = map_with_label(BUOY)
    far_away = Window(30.0, 30.0, WINDOW.resolution)
    object_map.update_grid(grid_with([], window=far_away), far_away)
    assert object_map.objects() == []


def test_two_buoys_of_one_class_are_two_objects():
    object_map = new_map()
    object_map.update_grid(grid_with([(0.0, 0.0), (0.0, 1.8)]), WINDOW)
    object_map.add_label("red_buoy", 0.0, 0.0)
    object_map.add_label("red_buoy", 0.0, 1.8)
    assert len(object_map.objects()) == 2
