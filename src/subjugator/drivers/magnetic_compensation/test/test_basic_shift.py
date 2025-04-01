import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from rclpy.duration import Duration
from sensor_msgs.msg import MagneticField


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
                    {"shift": [0.0, 0.0, 0.0]},
                    {
                        "scale": [
                            1.0,
                            1.0,
                            1.0,
                            1.0,
                            1.0,
                            1.0,
                            1.0,
                            1.0,
                            1.0,
                        ],
                    },
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


class TestBasicShift(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_basic_shift")
        self.mag_raw_pub = self.node.create_publisher(MagneticField, "imu/mag_raw", 10)
        self.mag_sub = self.node.create_subscription(
            MagneticField,
            "imu/mag",
            self.imu_callback,
            10,
        )

    def __init__(self, *args):
        super().__init__(*args)
        self.most_recent_imu_msg = None

    def imu_callback(self, msg):
        self.most_recent_imu_msg = msg

    def test_shifted(self):
        # send an arbitrary measurement and ensure that it has not shifted
        mf_message = MagneticField()
        mf_message.magnetic_field.x = 1.0
        mf_message.magnetic_field.y = 1.0
        mf_message.magnetic_field.z = 1.0
        mf_message.header.frame_id = "imu_link"
        self.mag_raw_pub.publish(mf_message)
        self.node.get_clock().sleep_for(Duration(seconds=20.0))
        self.assertIsNot(self.most_recent_imu_msg, None)
        self.assertEqual(self.most_recent_imu_msg.magnetic_field.x, 1.0)
        self.assertEqual(self.most_recent_imu_msg.magnetic_field.y, 1.0)
        self.assertEqual(self.most_recent_imu_msg.magnetic_field.z, 1.0)
