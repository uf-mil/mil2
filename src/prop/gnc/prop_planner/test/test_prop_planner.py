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


def test_planning_service(tmp_path):
    """Exercise missing inputs, obstacle avoidance, and invalid/unreachable goals."""
    from nav_msgs.msg import OccupancyGrid
    from nav_msgs.srv import GetPlan
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy

    context = Context()
    rclpy.init(context=context)
    suffix = uuid.uuid4().hex
    ns = f"/planning_{suffix}"
    node = rclpy.create_node(f"service_test_{suffix}", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps = node.create_publisher(OccupancyGrid, ns + "/map", qos)
    odom = node.create_publisher(Odometry, ns + "/odom", qos_profile_sensor_data)
    client = node.create_client(GetPlan, ns + "/prop_planner/plan")
    with (tmp_path / "service.log").open("w") as log:
        process = subprocess.Popen([
            os.environ["PROP_PLANNER_EXECUTABLE"], "--ros-args",
            "-r", f"__ns:={ns}", "-p", f"global_map_topic:={ns}/map",
            "-p", f"odom_topic:={ns}/odom",
        ], stdout=log, stderr=subprocess.STDOUT)
        try:
            assert client.wait_for_service(timeout_sec=10)
            request = GetPlan.Request()
            request.start.header.frame_id = "map"
            request.start.pose.position.x = 0.5
            request.start.pose.position.y = 0.5
            request.start.pose.orientation.w = 1.0
            request.goal.header.frame_id = "map"
            request.goal.pose.position.x = 4.5
            request.goal.pose.position.y = 0.5
            request.goal.pose.orientation.z = 1.0

            def call():
                future = client.call_async(request)
                executor.spin_until_future_complete(future, timeout_sec=5)
                assert future.done()
                return future.result().plan

            def publish_map(grid):
                maps.publish(grid)
                # Allow subscription callbacks to run before the independent service request.
                time.sleep(0.2)

            assert not call().poses  # No map.
            grid = OccupancyGrid()
            grid.header.frame_id = "map"
            grid.info.width = 5
            grid.info.height = 5
            grid.info.resolution = 1.0
            grid.info.origin.orientation.w = 1.0
            grid.data = [0] * 25
            for y in range(4):
                grid.data[y * 5 + 2] = 100
            deadline = time.monotonic() + 10
            while maps.get_subscription_count() == 0:
                assert time.monotonic() < deadline
                time.sleep(0.05)
            publish_map(grid)
            path = call()
            assert path.header.frame_id == "map"
            assert path.poses[0].pose.position == request.start.pose.position
            assert path.poses[-1].pose == request.goal.pose
            assert max(p.pose.position.y for p in path.poses) == 4.5
            for pose in path.poses:
                assert pose.header == path.header
                index = int(pose.pose.position.y) * 5 + int(pose.pose.position.x)
                assert grid.data[index] == 0
            for a, b in zip(path.poses, path.poses[1:]):
                assert abs(a.pose.position.x - b.pose.position.x) + abs(
                    a.pose.position.y - b.pose.position.y) <= 1.0

            request.start.header.frame_id = ""
            assert not call().poses  # Implicit start requires odometry.
            msg = Odometry()
            msg.header.frame_id = "map"
            msg.pose.pose.position.x = 0.5
            msg.pose.pose.position.y = 0.5
            msg.pose.pose.orientation.w = 1.0
            deadline = time.monotonic() + 5
            while True:
                odom.publish(msg)
                time.sleep(0.05)
                if call().poses:
                    break
                assert time.monotonic() < deadline
            request.goal.header.frame_id = "odom"
            assert not call().poses
            request.goal.header.frame_id = "map"
            request.tolerance = 1.0
            assert not call().poses
            request.tolerance = 0.0
            request.goal.pose.position.x = float("nan")
            assert not call().poses
            request.goal.pose.position.x = 9.0
            assert not call().poses
            request.goal.pose.position.x = 4.5
            grid.data[4] = -1
            publish_map(grid)
            assert not call().poses  # Unknown goal.
            grid.data[4] = 0
            grid.data[22] = 100
            publish_map(grid)
            assert not call().poses  # Complete wall.
            grid.data = [0] * 24
            publish_map(grid)
            assert not call().poses  # Malformed grid.
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            executor.shutdown()
            node.destroy_node()
            context.shutdown()
