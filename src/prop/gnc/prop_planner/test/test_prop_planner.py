"""Exercise continuous planning, status, perception input, and TF."""

import math
import os
import subprocess
import time
import uuid
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


@pytest.fixture
def planner(tmp_path, request):
    context = Context()
    rclpy.init(context=context)
    ns = "/planner_test_" + uuid.uuid4().hex
    node = rclpy.create_node("client", namespace=ns, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    tracks = node.create_publisher(MarkerArray, ns + "/tracks", 10)
    odom = node.create_publisher(Odometry, ns + "/odom", qos_profile_sensor_data)
    client = node.create_client(Trigger, ns + "/prop_planner/active")
    goals = node.create_publisher(PoseStamped, ns + "/goal_pose", 10)
    paths = []
    node.create_subscription(Path, ns + "/path", paths.append, 10)
    logs = tmp_path / "planner.log"
    with logs.open("w") as output:
        process = subprocess.Popen(
            [
                os.environ["PROP_PLANNER_EXECUTABLE"],
                "--ros-args",
                "-r",
                f"__ns:={ns}",
                "-r",
                f"/path:={ns}/path",
                *(["-p", "active:=true"] if getattr(request, "param", True) else []),
                "-p",
                f"tracks_topic:={ns}/tracks",
                "-p",
                f"odom_topic:={ns}/odom",
            ],
            stdout=output,
            stderr=subprocess.STDOUT,
        )
        try:
            assert client.wait_for_service(timeout_sec=10), logs.read_text()

            def status():
                future = client.call_async(Trigger.Request())
                executor.spin_until_future_complete(future, timeout_sec=5)
                assert future.done(), logs.read_text()
                return future.result()

            def call(req):
                if req.start.header.frame_id:
                    msg = Odometry()
                    msg.header = req.start.header
                    msg.pose.pose = req.start.pose
                    send(msg, odom)
                send(req.goal, goals)
                # Drain messages generated before the new inputs were processed.
                deadline = time.monotonic() + 0.4
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.02)
                paths.clear()
                deadline = time.monotonic() + 5
                while not paths:
                    assert time.monotonic() < deadline, logs.read_text()
                    executor.spin_once(timeout_sec=0.1)
                return paths[-1]

            def send(message, publisher=tracks):
                deadline = time.monotonic() + 5
                while publisher.get_subscription_count() == 0:
                    assert time.monotonic() < deadline
                    time.sleep(0.02)
                publisher.publish(message)
                time.sleep(0.2)

            node.planner_status = status
            node.planner_paths = paths
            node.planner_executor = executor
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
    req = SimpleNamespace(start=PoseStamped(), goal=PoseStamped())
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
    node, call, send, _, _ = planner
    assert node.planner_status().success
    req = request()
    req.start.header.frame_id = ""
    assert not call(req).poses
    req = request()
    send(MarkerArray())
    path = call(req)
    assert len(path.poses) == 2
    assert path.poses[0].pose == req.start.pose
    assert path.poses[-1].pose == req.goal.pose
    assert all(p.header == path.header for p in path.poses)
    assert path.header.frame_id == "map"
    assert node.planner_status().success


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
    assert "Boat position" not in logs.read_text()
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


def test_continuous_publication_without_goal(planner):
    node, _, _, _, _ = planner
    paths = node.planner_paths
    deadline = time.monotonic() + 3
    while len(paths) < 3:
        assert time.monotonic() < deadline
        node.planner_executor.spin_once(timeout_sec=0.1)
    assert all(not path.poses for path in paths)
    assert all(path.header.frame_id == "map" for path in paths)
    assert paths[-1].header.stamp != paths[0].header.stamp


@pytest.mark.parametrize("planner", [False], indirect=True)
def test_toggle_active_without_inputs(planner):
    node, _, _, _, _ = planner
    toggle = node.create_client(
        Trigger,
        node.get_namespace() + "/prop_planner/toggle_active",
    )
    assert toggle.wait_for_service(timeout_sec=5)
    assert not node.planner_status().success
    for active in (True, False, True, False):
        future = toggle.call_async(Trigger.Request())
        node.planner_executor.spin_until_future_complete(future, timeout_sec=5)
        assert future.done()
        assert future.result().success == active
        assert future.result().message == ("active" if active else "inactive")
        # Querying repeatedly must not change the flag.
        assert node.planner_status().success == active
        assert node.planner_status().success == active


@pytest.mark.parametrize("planner", [False], indirect=True)
def test_inactive_gates_inputs_and_publication(planner):
    node, call, send, odom, logs = planner
    parameters = AsyncParameterClient(node, node.get_namespace() + "/prop_planner")
    assert parameters.wait_for_services(timeout_sec=5)

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            node.planner_executor.spin_once(timeout_sec=0.02)

    def set_active(active):
        future = parameters.set_parameters([Parameter("active", value=active)])
        node.planner_executor.spin_until_future_complete(future, timeout_sec=5)
        assert future.done() and future.result().results[0].successful
        assert node.planner_status().success == active

    goals = node.create_publisher(PoseStamped, node.get_namespace() + "/goal_pose", 10)
    msg = Odometry()
    msg.header.frame_id = "map"
    msg.pose.pose.orientation.w = 1.0
    send(msg, odom)
    send(MarkerArray())
    send(request().goal, goals)
    spin_for(2.2)
    assert not node.planner_paths
    assert "not active; toggle active to true first" in logs.read_text()
    set_active(True)
    spin_for(0.3)
    assert node.planner_paths and all(not p.poses for p in node.planner_paths)
    # Every input sent while inactive was ignored.
    send(msg, odom)
    spin_for(0.2)
    assert not node.planner_paths[-1].poses
    send(MarkerArray())
    spin_for(0.2)
    assert not node.planner_paths[-1].poses
    assert call(request()).poses
    set_active(False)
    spin_for(0.3)  # Drain messages published before deactivation.
    node.planner_paths.clear()
    spin_for(0.4)
    assert not node.planner_paths
    assert logs.read_text().count("prop_planner is active now") == 1
    assert "Planning failed" not in logs.read_text()
    assert "Boat position" not in logs.read_text()
