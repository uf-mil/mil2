import math

import pytest
from nav_msgs.msg import Odometry
from rf2o_cov_mapper.rf2o_cov_mapper import beam_count, sideways_speed
from sensor_msgs.msg import LaserScan


def odom(x, y, yaw):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    return msg


def test_beam_count_ignores_missing_and_out_of_range_returns():
    scan = LaserScan(range_min=0.1, range_max=30.0)
    scan.ranges = [5.0, math.inf, math.nan, 0.05, 31.0, 29.0]
    assert beam_count(scan) == 2


def test_sideways_speed_is_measured_across_the_earlier_heading():
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
