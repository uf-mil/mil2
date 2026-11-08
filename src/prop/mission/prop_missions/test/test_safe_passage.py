import math

import numpy as np
import pytest
from prop_missions.geo import GeoReference, offset_latlon
from prop_missions.safe_passage import circle, passage


def sweep(points, center):
    """Signed degrees turned about center going through points (+ is counter-clockwise)."""
    angles = [math.atan2(y - center[1], x - center[0]) for x, y, _ in points]
    return math.degrees(
        sum(math.remainder(b - a, 2 * math.pi) for a, b in zip(angles, angles[1:])),
    )


@pytest.mark.parametrize("clockwise", [True, False])
def test_circle_goes_all_the_way_round_the_right_way(clockwise):
    center, start = (4.0, -2.0), (10.0, -2.0)
    points = circle(center, start, 3.0, clockwise)
    turned = sweep(points, center)
    assert abs(turned) >= 360.0
    assert (turned < 0) == clockwise
    assert all(math.isclose(math.dist(p[:2], center), 3.0) for p in points)
    assert math.dist(points[0][:2], (7.0, -2.0)) < 1e-9


@pytest.mark.parametrize("clockwise", [True, False])
def test_circle_headings_follow_the_way_round(clockwise):
    points = circle((0.0, 0.0), (5.0, 0.0), 3.0, clockwise)
    for (x0, y0, heading), (x1, y1, _) in zip(points, points[1:]):
        travel = math.atan2(y1 - y0, x1 - x0)
        assert abs(math.remainder(travel - heading, 2 * math.pi)) < math.radians(30)


def sides(entry, waypoints, exit_, buoys):
    """
    Which side each buoy is on when abeam of the path entry -> waypoints -> exit:
    +1 port (left), -1 starboard (right).
    """
    path = (
        [np.array(entry, float)]
        + [np.array(w[:2], float) for w in waypoints]
        + [np.array(exit_, float)]
    )
    out = []
    for x, y, _ in buoys:
        b = np.array([x, y], float)
        best = None
        for a, c in zip(path, path[1:]):
            d = c - a
            t = np.clip((b - a) @ d / (d @ d), 0.0, 1.0)
            near = a + t * d
            if best is None or np.linalg.norm(b - near) < best[0]:
                best = (
                    np.linalg.norm(b - near),
                    np.sign(d[0] * (b - near)[1] - d[1] * (b - near)[0]),
                )
        out.append(best[1])
    return out


def test_red_to_starboard_green_to_port():
    entry, exit_ = (0.0, 0.0), (20.0, 0.0)
    buoys = [
        (5.0, 1.0, "FLASHING_RED"),
        (10.0, -1.0, "FLASHING_GREEN"),
        (12.0, 4.0, "OFF"),
    ]
    waypoints = passage(entry, exit_, buoys)
    assert [w[:2] for w in waypoints] == [(5.0, 3.0), (10.0, -3.0)]
    assert sides(entry, waypoints, exit_, buoys[:2]) == [-1, 1]


def test_a_red_and_green_abreast_are_passed_between():
    buoys = [(15.0, -1.5, "FLASHING_RED"), (15.5, 1.5, "FLASHING_GREEN")]
    waypoints = passage((0.0, 0.0), (20.0, 0.0), buoys)
    assert waypoints == [(15.25, 0.0, 15.25)]


def test_buoys_outside_the_passage_are_left_alone():
    buoys = [(-3.0, 0.0, "FLASHING_RED"), (25.0, 1.0, "FLASHING_GREEN")]
    assert passage((0.0, 0.0), (20.0, 0.0), buoys) == []


@pytest.mark.parametrize("seed", range(50))
def test_random_channels_pass_every_buoy_on_its_side(seed):
    rng = np.random.default_rng(seed)
    entry = rng.uniform(-30, 30, 2)
    heading = rng.uniform(-math.pi, math.pi)
    along = np.array([math.cos(heading), math.sin(heading)])
    left = np.array([-along[1], along[0]])
    exit_ = entry + 30.0 * along
    buoys = []
    for s in np.arange(5.0, 26.0, 5.0):
        centre = entry + s * along + rng.uniform(-2, 2) * left
        half = rng.uniform(2.0, 3.5)
        if rng.random() < 0.8:
            red = centre - half * left + rng.uniform(-0.5, 0.5) * along
            buoys.append((*red, "FLASHING_RED"))
        if rng.random() < 0.8:
            green = centre + half * left + rng.uniform(-0.5, 0.5) * along
            buoys.append((*green, "FLASHING_GREEN"))
    waypoints = passage(entry, exit_, buoys)
    wanted = [-1 if state == "FLASHING_RED" else 1 for *_, state in buoys]
    assert sides(entry, waypoints, exit_, buoys) == wanted


class NoNode:
    def create_subscription(self, *args, **kwargs):
        return None


def test_latlon_and_back_in_a_turned_frame():
    geo = GeoReference(NoNode())
    geo.fix = (1.2891, 103.8612, 2.0, -1.0)
    geo.turn = math.radians(37.0)
    for x, y in ((2.0, -1.0), (30.0, 12.0), (-15.0, 40.0)):
        latitude, longitude = geo.latlon(x, y)
        back = geo.local(latitude, longitude)
        assert math.dist(back, (x, y)) < 1e-3


def test_ten_metres_east_and_north():
    lat, lon = offset_latlon(1.2891, 103.8612, 10.0, 10.0)
    assert math.isclose((lat - 1.2891) * math.pi / 180 * 6378137.0, 10.0, rel_tol=1e-9)
    assert lon > 103.8612


def test_an_unturned_frame_puts_x_east():
    geo = GeoReference(NoNode())
    geo.fix = (1.2891, 103.8612, 0.0, 0.0)
    geo.turn = 0.0
    lat, lon = geo.latlon(10.0, 0.0)
    assert math.isclose(lat, 1.2891) and lon > 103.8612


def test_one_mismatched_reading_barely_moves_the_turn():
    geo = GeoReference(NoNode())
    for _ in range(50):
        geo.add_turn(math.radians(30.0))
    geo.add_turn(math.radians(120.0))
    assert abs(math.degrees(geo.turn) - 30.0) < 4.0
    for _ in range(100):
        geo.add_turn(math.radians(30.0))
    assert abs(math.degrees(geo.turn) - 30.0) < 0.1


def test_the_turn_averages_across_plus_minus_pi():
    geo = GeoReference(NoNode())
    for i in range(200):
        geo.add_turn(math.pi - 0.01 if i % 2 else -math.pi + 0.01)
    assert abs(abs(geo.turn) - math.pi) < 0.02
