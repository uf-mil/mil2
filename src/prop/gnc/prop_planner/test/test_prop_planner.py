"""Exercise the real planning service, perception input, and TF."""
import math
import os
import subprocess
import time
import uuid

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


@pytest.fixture
def planner(tmp_path):
    context = Context()
    rclpy.init(context=context)
    ns = "/planner_test_" + uuid.uuid4().hex
    node = rclpy.create_node("client", namespace=ns, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    tracks = node.create_publisher(MarkerArray, ns + "/tracks", 10)
    odom = node.create_publisher(Odometry, ns + "/odom", qos_profile_sensor_data)
    client = node.create_client(GetPlan, ns + "/prop_planner/plan")
    logs = tmp_path / "planner.log"
    with logs.open("w") as output:
        process = subprocess.Popen([
            os.environ["PROP_PLANNER_EXECUTABLE"], "--ros-args", "-r", f"__ns:={ns}",
            "-p", f"tracks_topic:={ns}/tracks", "-p", f"odom_topic:={ns}/odom",
        ], stdout=output, stderr=subprocess.STDOUT)
        try:
            assert client.wait_for_service(timeout_sec=10), logs.read_text()

            def call(req):
                future = client.call_async(req)
                executor.spin_until_future_complete(future, timeout_sec=5)
                assert future.done(), logs.read_text()
                return future.result().plan

            def send(message, publisher=tracks):
                deadline = time.monotonic() + 5
                while publisher.get_subscription_count() == 0:
                    assert time.monotonic() < deadline
                    time.sleep(0.02)
                publisher.publish(message)
                time.sleep(0.2)

            yield node, call, send, odom, logs
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


def request():
    req = GetPlan.Request()
    req.start.header.frame_id = "map"
    req.start.pose.orientation.w = 1.0
    req.goal.header.frame_id = "map"
    req.goal.pose.position.x = 10.0
    req.goal.pose.orientation.z = 1.0
    req.goal.pose.orientation.w = 0.0
    return req


def detection(frame="map", x=5.0):
    marker = Marker()
    marker.header.frame_id = frame
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 2.0
    return MarkerArray(markers=[marker])


def test_missing_input_and_empty_scan(planner):
    _, call, send, _, _ = planner
    req = request()
    assert not call(req).poses
    send(MarkerArray())
    path = call(req)
    assert len(path.poses) == 2
    assert path.poses[0].pose == req.start.pose
    assert path.poses[-1].pose == req.goal.pose
    assert all(p.header == path.header for p in path.poses)
    assert path.header.frame_id == "map"
    req.start.header.frame_id = ""
    assert not call(req).poses


def test_confirmation_clearance_and_persistent_map(planner):
    _, call, send, _, _ = planner
    req = request()
    send(detection())
    assert len(call(req).poses) == 2
    send(detection())
    send(detection())
    path = call(req)
    assert len(path.poses) > 2
    for a, b in zip(path.poses, path.poses[1:]):
        ax, ay = a.pose.position.x, a.pose.position.y
        dx, dy = b.pose.position.x - ax, b.pose.position.y - ay
        t = max(0.0, min(1.0, ((5.0 - ax) * dx - ay * dy) / (dx * dx + dy * dy)))
        assert math.hypot(ax + t * dx - 5.0, ay + t * dy) >= math.sqrt(2) + 2 - 1e-6
    clear = Marker()
    clear.action = Marker.DELETEALL
    send(MarkerArray(markers=[clear]))
    assert len(call(req).poses) > 2
    req.goal.pose.position.x = 5.0
    assert not call(req).poses
    req = request()
    req.start.pose.position.x = 5.0
    assert not call(req).poses


def test_invalid_requests_and_detections(planner):
    _, call, send, _, _ = planner
    req = request()
    bad = detection()
    bad.markers[0].scale.x = float("nan")
    send(bad)
    assert not call(req).poses
    send(detection(frame="missing_sensor_tf"))
    assert not call(req).poses
    send(MarkerArray())
    req.goal.header.frame_id = "missing_goal_tf"
    assert not call(req).poses
    req.goal.header.frame_id = "map"
    req.goal.pose.position.x = float("nan")
    assert not call(req).poses
    req = request()
    req.tolerance = 1.0
    assert not call(req).poses
    req = request()
    req.goal.pose.orientation.z = 2.0
    assert not call(req).poses


def test_odometry_zero_and_updates(planner):
    _, call, send, odom, logs = planner
    assert "Boat position" not in logs.read_text()
    send(MarkerArray())
    req = request()
    req.start.header.frame_id = ""
    msg = Odometry()
    msg.header.frame_id = "map"
    msg.pose.pose.orientation.w = 1.0
    send(msg, odom)
    assert call(req).poses[0].pose.position.x == 0.0
    assert "x=0.00 m, y=0.00 m, z=0.00 m" in logs.read_text()
    msg.pose.pose.position.x = -3.0
    msg.pose.pose.position.y = 8.25
    send(msg, odom)
    assert call(req).poses[0].pose.position == msg.pose.pose.position


def test_transform_detection_start_and_goal(planner):
    node, call, send, _, _ = planner
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.child_frame_id = "sensor_" + uuid.uuid4().hex
    transform.transform.translation.x = 5.0
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    send(MarkerArray())
    req = request()
    req.start.header.frame_id = transform.child_frame_id
    req.start.pose.position.x = -5.0
    req.goal.header.frame_id = transform.child_frame_id
    req.goal.pose.position.x = 5.0
    req.goal.pose.orientation.z = 0.0
    deadline = time.monotonic() + 5
    while True:
        path = call(req)
        if path.poses:
            break
        assert time.monotonic() < deadline
    assert path.poses[0].pose.position.x == pytest.approx(0.0)
    assert path.poses[-1].pose.position.x == pytest.approx(10.0)
    assert path.poses[-1].pose.orientation.w == 0.0
    for _ in range(3):
        send(detection(frame=transform.child_frame_id, x=0.0))
    assert len(call(req).poses) > 2
