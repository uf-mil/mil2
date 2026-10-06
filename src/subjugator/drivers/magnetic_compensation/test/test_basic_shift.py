"""End-to-end launch test for the HardsoftCompensator node.

Brings up the real component with an identity calibration (zero shift, identity
scale) and checks the live correction path: a matching-frame message passes
through unchanged, and a wrong-frame message is dropped.

NOTE: requires a built+sourced ROS 2 workspace (run via ``colcon test``); it is
not exercised by the pure-Python ``test_ellipsoid.py`` suite.
"""

import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from sensor_msgs.msg import MagneticField

FRAME_ID = "imu_link"


@pytest.mark.launch_test
def generate_test_description():
    compensator = ComposableNodeContainer(
        name="imu_fixup_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="magnetic_compensation",
                plugin="mil::magnetic_compensation::HardsoftCompensator",
                name="hardsoft_compensator",
                parameters=[
                    {"frame_id": FRAME_ID},
                    {"shift": [0.0, 0.0, 0.0]},
                    {"scale": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]},
                ],
            ),
        ],
    )

    return launch.LaunchDescription(
        [
            compensator,
            launch_testing.actions.ReadyToTest(),
        ],
    )


class TestCompensatorNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_compensator")
        self.received = None
        self.pub = self.node.create_publisher(MagneticField, "/imu/mag_raw", 10)
        self.sub = self.node.create_subscription(
            MagneticField,
            "imu/mag",
            self._on_msg,
            10,
        )

    def tearDown(self):
        self.node.destroy_node()

    def _on_msg(self, msg):
        self.received = msg

    def _publish_and_wait(self, frame_id, xyz, timeout_sec=5.0):
        """Publish a raw reading repeatedly while spinning until a reply or timeout.

        Republishing covers pub/sub discovery latency; spinning the node is what
        the original test was missing (it slept without ever spinning, so the
        callback never fired and the test always failed -> it was disabled).
        """
        msg = MagneticField()
        msg.header.frame_id = frame_id
        msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z = xyz

        deadline = time.monotonic() + timeout_sec
        while self.received is None and time.monotonic() < deadline:
            self.pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.received

    def test_identity_passes_through_unchanged(self):
        result = self._publish_and_wait(FRAME_ID, (1.0, 2.0, 3.0))
        self.assertIsNotNone(result, "no message received from compensator")
        self.assertAlmostEqual(result.magnetic_field.x, 1.0)
        self.assertAlmostEqual(result.magnetic_field.y, 2.0)
        self.assertAlmostEqual(result.magnetic_field.z, 3.0)

    def test_wrong_frame_id_is_dropped(self):
        result = self._publish_and_wait(
            "not_imu_link",
            (1.0, 2.0, 3.0),
            timeout_sec=2.0,
        )
        self.assertIsNone(result, "message with mismatched frame_id should be dropped")
