"""Sensor geometry tests with randomized truth used only to generate input data."""

import math
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np
from nav_msgs.msg import Odometry
from prop_vision.dock_perception import color_centers, detect_bay
from prop_missions.docking_mission import DockingMission, settled


def scene(yaw=0.0, offset=(0.0, 0.0), colors=("red", "green", "red")):
    rng = np.random.default_rng(12)
    points = [[11.75, y] for y in np.linspace(-4.5, 4.5, 180)]
    for y in (-4.5, -1.5, 1.5, 4.5):
        for side in (-0.2, 0.2):
            points += [[x, y + side] for x in np.linspace(8, 11.75, 80)]
        points += [[8, yy] for yy in np.linspace(y - 0.2, y + 0.2, 12)]
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    points = np.array(points) @ rotation.T + offset
    points += rng.normal(0, 0.008, points.shape)
    camera = np.array([0.1, 0.0])
    beacons = np.array([[11.6, y] for y in (3, 0, -3)]) @ rotation.T + offset
    pixels = {}
    for color, p in zip(colors, beacons):
        pixels.setdefault(color, []).append(
            320 - 310 * (p[1] - camera[1]) / (p[0] - camera[0])
        )
    return points, pixels, camera, rotation


class PerceptionTests(unittest.TestCase):
    def test_translated_rotated_reordered_bays(self):
        for yaw, offset, colors in [
            (0.12, (2, -1), ("green", "red", "red")),
            (-0.1, (1, 1), ("red", "green", "red")),
            (0.05, (1, -1), ("red", "red", "green")),
        ]:
            points, pixels, camera, rotation = scene(yaw, offset, colors)
            with self.subTest(yaw=yaw, colors=colors):
                bay = detect_bay(points, pixels, 310, 320, camera, 0)
                self.assertIsNotNone(bay)
                expected = (
                    rotation @ np.array([10.5, [3, 0, -3][colors.index("green")]])
                    + offset
                )
                np.testing.assert_allclose(bay.goal, expected, atol=0.16)
                self.assertAlmostEqual(bay.yaw, yaw, delta=0.03)
                self.assertEqual(bay.number, colors.index("green") + 1)
                self.assertIs(type(bay.number), int)

    def test_recorded_lidar_with_occluded_finger_face(self):
        # Actual Gazebo camera centroids and sparse VLP-16 returns, seed 3.
        data = np.load(Path(__file__).parent / "data" / "occluded_dock.npz")
        # Retain the sparse geometry regression; relabel the old blue target
        # green and the two other observations red. This is not new RGB evidence.
        pixels = {"red": list(data["pixels"][:2]), "green": [data["pixels"][2]]}
        bay = detect_bay(
            data["points"],
            pixels,
            data["fx"],
            data["cx"],
            data["camera"],
            data["yaw"],
        )
        self.assertIsNotNone(bay)
        np.testing.assert_allclose(bay.goal, [11.0889, -3.2746], atol=0.08)
        self.assertLess(bay.width, 2.8)

    def test_missing_beacon_never_produces_goal(self):
        points, pixels, camera, _ = scene()
        del pixels["green"]
        self.assertIsNone(detect_bay(points, pixels, 310, 320, camera, 0))

    def test_color_without_geometry_never_produces_goal(self):
        _, pixels, camera, _ = scene()
        self.assertIsNone(
            detect_bay(np.empty((0, 2)), pixels, 310, 320, camera, 0),
        )

    def test_wall_without_fingers_rejected(self):
        points, pixels, camera, _ = scene()
        points = points[points[:, 0] > 11.5]
        self.assertIsNone(detect_bay(points, pixels, 310, 320, camera, 0))

    def test_repeated_red_indicators_and_ambiguous_green(self):
        image = np.zeros((120, 180, 3), np.uint8)
        image[30:60, 20:30] = [255, 0, 0]
        image[30:60, 80:90] = [0, 255, 0]
        image[30:60, 140:150] = [255, 0, 0]
        pixels = color_centers(image)
        self.assertEqual(pixels, {"red": [24.5, 144.5], "green": [84.5]})
        image[80:100, 140:150] = [0, 255, 0]
        pixels = color_centers(image)
        self.assertEqual(len(pixels["green"]), 2)
        points, _, camera, _ = scene()
        self.assertIsNone(detect_bay(points, pixels, 310, 320, camera, 0))

    def test_rgb_detection_through_geometry(self):
        points, pixels, camera, _ = scene()
        image = np.zeros((120, 640, 3), np.uint8)
        for color, positions in pixels.items():
            for position in positions:
                x = round(position)
                image[30:60, x-3:x+4] = [0, 255, 0] if color == "green" else [255, 0, 0]
        bay = detect_bay(points, color_centers(image), 310, 320, camera, 0)
        self.assertIsNotNone(bay)
        np.testing.assert_allclose(bay.goal, [10.5, 0], atol=0.16)

    def test_stale_data_clears_plan_without_reporting_docked(self):
        node = Mock()
        node.seconds.return_value = 4.0
        node.start = 1.0
        node.timeout = 240.0
        node.sensor_timeout = 1.0
        node.state = "ALIGN"
        node.paused = False
        old = Odometry()
        old.header.stamp.sec = 2
        node.odom = [old]
        node.cloud = node.image = node.info = old
        DockingMission.step(node)
        self.assertTrue(node.paused)
        self.assertEqual(node.state, "ALIGN")
        self.assertEqual(len(node.pub.publish.call_args.args[0].poses), 0)
        node.plan.assert_not_called()
        node.sample.assert_not_called()

    def test_settled_requires_heading_position_and_motion(self):
        msg = Odometry()
        msg.pose.pose.orientation.w = 1.0
        self.assertTrue(settled(msg, np.array([0.0, 0.0]), 0.0))
        self.assertFalse(settled(msg, np.array([0.0, 3.0]), 0.0))
        self.assertFalse(settled(msg, np.array([0.0, 0.0]), 0.3))
        msg.twist.twist.linear.x = 0.2
        self.assertFalse(settled(msg, np.array([0.0, 0.0]), 0.0))


if __name__ == "__main__":
    unittest.main()
