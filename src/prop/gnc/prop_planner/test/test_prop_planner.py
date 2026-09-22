"""Exercise the actual planner executable using ROS odometry messages."""

import os
import subprocess
import time
import uuid

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.qos import qos_profile_sensor_data


@pytest.fixture
def planner(tmp_path):
    """Start an isolated planner and always clean up its process and ROS context."""
    context = Context()
    rclpy.init(context=context)
    suffix = uuid.uuid4().hex
    node = rclpy.create_node(f"planner_test_{suffix}", context=context)
    process = None
    log_path = tmp_path / "planner.log"
    with log_path.open("w") as log:

        def start(custom_topic=True):
            nonlocal process
            topic = f"/planner_test_{suffix}/odom"
            # Isolate the default relative node namespace as well as the test input.
            args = [os.environ["PROP_PLANNER_EXECUTABLE"], "--ros-args",
                    "-r", f"__ns:=/planner_test_{suffix}"]
            if custom_topic:
                args += ["-p", f"odom_topic:={topic}"]
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
            publisher = node.create_publisher(Odometry, topic, qos_profile_sensor_data)
            return publisher

        def wait_for(predicate, publisher=None, message=None):
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                assert process.poll() is None, log_path.read_text()
                if publisher is not None:
                    publisher.publish(message)
                if predicate(log_path.read_text()):
                    return
                time.sleep(0.05)
            pytest.fail(f"Timed out waiting for planner output:\n{log_path.read_text()}")

        try:
            yield start, wait_for, log_path
        finally:
            if process is not None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            node.destroy_node()
            context.shutdown()


def test_default_topic(planner):
    start, wait_for, _ = planner
    start(custom_topic=False)
    wait_for(lambda text: "Waiting for odometry on /odometry/filtered/global" in text)


def test_no_position_before_message_and_zero_position_is_valid(planner):
    start, wait_for, log_path = planner
    publisher = start()
    wait_for(lambda _: publisher.get_subscription_count() > 0)
    time.sleep(1.1)
    assert "Boat position" not in log_path.read_text()

    message = Odometry()
    message.header.frame_id = "map"
    message.pose.pose.orientation.w = 1.0
    wait_for(
        lambda text: "frame 'map': x=0.00 m, y=0.00 m, z=0.00 m" in text,
        publisher, message,
    )


def test_later_message_updates_position_and_frame(planner):
    start, wait_for, _ = planner
    publisher = start()
    wait_for(lambda _: publisher.get_subscription_count() > 0)
    message = Odometry()
    message.header.frame_id = "map"
    message.pose.pose.orientation.w = 1.0
    message.pose.pose.position.x = 4.5
    message.pose.pose.position.y = 2.0
    wait_for(
        lambda text: "frame 'map': x=4.50 m, y=2.00 m, z=0.00 m" in text,
        publisher, message,
    )

    message.header.frame_id = "test_frame"
    message.pose.pose.position.x = -3.0
    message.pose.pose.position.y = 8.25
    message.pose.pose.position.z = 1.5
    wait_for(
        lambda text: "frame 'test_frame': x=-3.00 m, y=8.25 m, z=1.50 m" in text,
        publisher, message,
    )
