import math

import pytest
from rf2o_cov_mapper.object_vote import GyroHeading, ObjectVote, rotate

# Our velocity in base_link and yaw rate, held for the whole drive.
VX, VY, WZ = 1.0, 0.2, 0.3


def drive(static_objects, kayak=None, flicker=False, seconds=2.0):
    """Last vote from driving past world-frame objects; kayak is (start, velocity) if one crosses.

    With flicker, each static object is missing from every third scan, staggered.
    """
    heading = GyroHeading(kept_s=3.0)
    vote = ObjectVote(window=1.0, match_gate=0.5, min_agreeing=3, agree_speed=0.3)
    x = y = yaw = 0.0
    result = None
    for ms in range(int(seconds * 1000) + 1):
        t = ms / 1000
        if ms % 10 == 0:
            heading.add(t, WZ)
        if ms % 100 == 0:
            scan = ms // 100
            objects = [
                p
                for i, p in enumerate(static_objects)
                if not (flicker and (scan + i) % 3 == 0)
            ]
            if kayak:
                (kx, ky), (kvx, kvy) = kayak
                objects.append((kx + kvx * t, ky + kvy * t))
            seen = [rotate(px - x, py - y, -yaw) for px, py in objects]
            result = vote.update(t, seen, heading.at)
        dx, dy = rotate(VX, VY, yaw)
        x, y, yaw = x + dx / 1000, y + dy / 1000, yaw + WZ / 1000
    return result


BUOYS = [(8.0, 4.0), (6.0, -5.0), (12.0, 0.0), (-4.0, 7.0)]
KAYAK = ((5.0, 10.0), (1.0, -0.5))


def test_static_objects_give_our_velocity_while_turning():
    result = drive(BUOYS)
    assert (result.vx, result.vy) == pytest.approx((VX, VY), abs=0.01)
    assert (result.agreeing, result.voters) == (4, 4)


def test_objects_missing_from_some_scans_still_vote():
    result = drive(BUOYS, flicker=True)
    assert (result.vx, result.vy) == pytest.approx((VX, VY), abs=0.01)
    assert result.agreeing == result.voters >= 3


def test_a_moving_object_is_outvoted_only_with_three_static_ones_in_view():
    result = drive(BUOYS[:3], KAYAK)
    assert (result.vx, result.vy) == pytest.approx((VX, VY), abs=0.01)
    assert (result.agreeing, result.voters) == (3, 4)
    assert drive(BUOYS[:2], KAYAK) is None


def test_gyro_heading_interpolates_and_forgets_old_samples():
    heading = GyroHeading(kept_s=1.0)
    for t in (0.0, 0.5, 1.0, 1.5):
        heading.add(t, 0.2)
    assert heading.at(1.25) == pytest.approx(0.25)
    assert heading.at(0.25) is None
    assert math.isclose(heading.at(1.5) - heading.at(0.5), 0.2)
