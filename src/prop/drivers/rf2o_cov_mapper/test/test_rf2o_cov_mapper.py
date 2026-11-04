import math

import pytest
from nav_msgs.msg import Odometry
from rf2o_cov_mapper.object_vote import Vote
from rf2o_cov_mapper.rf2o_cov_mapper import (
    Referee,
    TrustLimits,
    beam_count,
    largest_clump_share,
    referee_from,
    sideways_speed,
    verdict,
)
from sensor_msgs.msg import LaserScan


def odom(x, y, yaw):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    return msg


def full_scan(ranges):
    increment = 2 * math.pi / len(ranges)
    scan = LaserScan(
        range_min=0.1,
        range_max=30.0,
        angle_min=-math.pi,
        angle_max=math.pi - increment,
        angle_increment=increment,
    )
    scan.ranges = ranges
    return scan


def test_beam_count_ignores_missing_and_out_of_range_returns():
    scan = LaserScan(range_min=0.1, range_max=30.0)
    scan.ranges = [5.0, math.inf, math.nan, 0.05, 31.0, 29.0]
    assert beam_count(scan) == 2


def test_clumps_break_at_gaps_and_range_jumps_and_wrap_around_the_scan():
    gap = math.inf
    # An object split across the scan's start and end (3 + 2 beams), a kayak at
    # 10 m (4 beams), then a buoy behind it at 15 m (1 beam).
    scan = full_scan([8.0, 8.0, 8.0, gap, 10.0, 10.1, 10.2, 10.1, 15.0, gap, 8.1, 8.1])
    assert largest_clump_share(scan, max_jump=0.5) == pytest.approx(5 / 10)
    # A partial scan's two ends are not neighbours.
    scan.angle_max = 0.0
    assert largest_clump_share(scan, max_jump=0.5) == pytest.approx(4 / 10)
    assert largest_clump_share(full_scan([gap] * 4), max_jump=0.5) == 0.0


def test_sideways_speed_is_measured_across_the_heading():
    # Facing +y, a step of -1 m in x is a step to the boat's left (+y in ROS).
    assert sideways_speed(
        odom(0, 0, math.pi / 2),
        odom(-1, 0, math.pi / 2),
        0.5,
    ) == pytest.approx(2.0)
    assert sideways_speed(
        odom(0, 0, math.pi / 2),
        odom(0, 1, math.pi / 2),
        0.5,
    ) == pytest.approx(0.0)


def test_straight_ahead_while_turning_is_not_sideways():
    # Turning 0.2 rad, the boat travels along the chord at the mid-step heading.
    step = (math.cos(0.1), math.sin(0.1))
    assert sideways_speed(odom(0, 0, 0.0), odom(*step, 0.2), 0.1) == pytest.approx(
        0.0,
        abs=1e-9,
    )


LIMITS = TrustLimits(
    min_beams=10,
    full_trust_beams=50,
    max_scan_gap=0.5,
    max_clump_share=0.5,
    max_reading_gap=0.6,
)


def judge(dt=0.1, velocity=(1.0, 0.0), beams=(90, 90), shares=(0.2, 0.2), referee=None):
    return verdict(dt, velocity, list(beams), list(shares), referee, LIMITS)


def test_trust_follows_the_sparser_of_the_two_matched_scans():
    assert judge(beams=(90, 90)) == "full trust"
    assert judge(beams=(30, 90)) == "low trust"
    assert judge(beams=(90, 6)) == "dropped: too few beams"


def test_stale_and_repeated_readings_are_dropped():
    assert judge(dt=0.0) == "dropped: repeated stamp"
    assert judge(dt=2.4) == "dropped: 2.4 s since the last reading"
    assert judge(beams=(90, None)) == "dropped: no matching scan"


def test_one_dominant_object_lowers_trust_unless_the_objects_vouch_for_rf2o():
    agree = Referee(end=0.0, agrees=True, gap=0.05, objects=4, vx=1.0, vy=0.0)
    disagree = Referee(end=0.0, agrees=False, gap=0.6, objects=4, vx=1.0, vy=0.0)
    assert judge(shares=(0.3, 0.77)) == "low trust: one object is 77% of the scan"
    assert judge(shares=(0.3, 0.77), referee=agree) == "full trust"
    assert (
        judge(referee=disagree)
        == "dropped: rf2o's last second is 0.60 m/s off what 4 objects say"
    )
    # A wild reading inside a window that agrees on average.
    assert (
        judge(velocity=(1.0, -0.8), referee=agree)
        == "dropped: this reading is 0.80 m/s off what 4 objects say"
    )


def test_referee_compares_rf2o_over_the_votes_own_window():
    vote = Vote(vx=1.0, vy=0.0, start=10.0, end=11.0, agreeing=3, voters=4)
    # Readings before the window must not count, and one wild reading inside it
    # must not carry the window off.
    readings = [(9.5, 5.0, 5.0), (10.2, 1.1, 0.0), (10.5, 9.0, 4.0), (10.8, 0.7, 0.0)]
    referee = referee_from(vote, readings, agree_speed=0.3)
    assert referee.agrees and referee.gap == pytest.approx(0.1)
    assert not referee_from(vote, [(10.5, 0.5, 0.0)], agree_speed=0.3).agrees
    assert referee_from(vote, readings[:1], agree_speed=0.3) is None
