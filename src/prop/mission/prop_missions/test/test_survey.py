import math

import pytest
from nav_msgs.msg import OccupancyGrid
from prop_missions.survey import Grid, lawnmower, polygon_lanes


def test_lawnmower_goes_back_and_forth_to_starboard():
    route = lawnmower((0.0, 0.0), 0.0, 10.0, 10.0, 5.0, 5.0, "starboard")
    points = [(round(x, 6), round(y, 6)) for x, y, _ in route]
    assert points == [
        (5, 0),
        (10, 0),
        (10, -5),
        (5, -5),
        (0, -5),
        (0, -10),
        (5, -10),
        (10, -10),
    ]
    assert [h for _, _, h in route] == pytest.approx(
        [0, 0, math.pi, math.pi, math.pi, 0, 0, 0],
    )


def test_lawnmower_follows_heading_and_side():
    route = lawnmower((2.0, 3.0), math.pi / 2, 10.0, 4.0, 4.0, 10.0, "port")
    points = [(round(x, 6), round(y, 6)) for x, y, _ in route]
    assert points == [(2, 13), (-2, 13), (-2, 3)]


def test_lawnmower_never_spaces_wider_than_asked():
    route = lawnmower((0.0, 0.0), 0.0, 7.0, 7.0, 5.0, 5.0, "port")
    xs = sorted({round(x, 6) for x, _, _ in route})
    ys = sorted({round(y, 6) for _, y, _ in route})
    assert xs == [0, 3.5, 7]
    assert ys == [0, 3.5, 7]


def grid_with(cells, width=10, height=10, resolution=1.0, origin=(0.0, 0.0)):
    msg = OccupancyGrid()
    msg.info.width, msg.info.height = width, height
    msg.info.resolution = resolution
    msg.info.origin.position.x, msg.info.origin.position.y = origin
    data = [-1] * (width * height)
    for (col, row), value in cells.items():
        data[row * width + col] = value
    msg.data = data
    return Grid(msg)


def test_blocked_near_an_occupied_cell_only():
    grid = grid_with({(5, 5): 100, (1, 1): 40})
    assert grid.blocked(5.5, 5.5, 0.5, 60)
    assert grid.blocked(7.0, 5.5, 1.6, 60)
    assert not grid.blocked(7.0, 5.5, 1.4, 60)
    assert not grid.blocked(1.5, 1.5, 1.0, 60)


def test_blocked_off_the_grid_is_free():
    grid = grid_with({(0, 0): 100}, origin=(-5.0, -5.0))
    assert grid.blocked(-4.5, -4.5, 0.5, 60)
    assert not grid.blocked(50.0, 50.0, 2.0, 60)
    assert not grid.blocked(-6.0, -6.0, 0.5, 60)


SQUARE = [(0.0, 0.0), (40.0, 0.0), (40.0, 20.0), (0.0, 20.0), (0.0, 0.0)]


def test_polygon_lanes_stay_margin_inside():
    route = polygon_lanes(SQUARE, 5.0, 5.0, 5.0, (0.0, 0.0))
    xs = [x for x, _, _ in route]
    ys = [y for _, y, _ in route]
    assert min(xs) == pytest.approx(5.0)
    assert max(xs) == pytest.approx(35.0)
    assert sorted({round(y, 6) for y in ys}) == [5, 10, 15]


def test_polygon_lanes_run_along_the_longest_side_from_the_nearest_corner():
    route = polygon_lanes(SQUARE, 5.0, 10.0, 5.0, (41.0, 21.0))
    points = [(round(x, 6), round(y, 6)) for x, y, _ in route]
    assert points[:4] == [(35, 15), (25, 15), (15, 15), (5, 15)]
    assert points[4:8] == [(5, 10), (15, 10), (25, 10), (35, 10)]
    assert points[-1] == (5, 5)
    assert route[0][2] == pytest.approx(math.pi)
    assert route[4][2] == pytest.approx(0.0)


def test_polygon_lanes_follow_a_turned_boundary():
    turn = math.radians(30)

    def turned(x, y):
        return (
            x * math.cos(turn) - y * math.sin(turn),
            x * math.sin(turn) + y * math.cos(turn),
        )

    route = polygon_lanes([turned(*p) for p in SQUARE], 5.0, 5.0, 5.0, (0.0, 0.0))
    back = [
        (
            x * math.cos(-turn) - y * math.sin(-turn),
            x * math.sin(-turn) + y * math.cos(-turn),
        )
        for x, y, _ in route
    ]
    assert all(
        5.0 - 1e-6 <= x <= 35.0 + 1e-6 and 5.0 - 1e-6 <= y <= 15.0 + 1e-6
        for x, y in back
    )
    assert route[0][2] == pytest.approx(turn)


def test_polygon_lanes_too_small_for_the_margin():
    assert polygon_lanes(SQUARE, 5.0, 5.0, 11.0, (0.0, 0.0)) == []
    assert polygon_lanes([(0.0, 0.0), (1.0, 1.0)], 5.0, 5.0, 0.0, (0.0, 0.0)) == []


def test_rounding_noise_does_not_add_a_waypoint():
    exact = polygon_lanes(SQUARE, 5.0, 5.0, 5.0, (0.0, 0.0))
    noisy = polygon_lanes(
        [(x + 5e-4 * (i % 2), y - 5e-4) for i, (x, y) in enumerate(SQUARE)],
        5.0,
        5.0,
        5.0,
        (0.0, 0.0),
    )
    assert len(noisy) == len(exact)
