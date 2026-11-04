"""Sensor geometry tests with randomized truth used only to generate input data."""

import math
import unittest
from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np
from nav_msgs.msg import Odometry
from prop_missions.docking_mission import DockingMission, settled
from prop_vision.dock_perception import color_centers, detect_bay


def scene(
    yaw=0.0,
    offset=(0.0, 0.0),
    colors=("red", "green", "red"),
    compact=False,
):
    rng = np.random.default_rng(12)
    wall, entrance = (11.0, 9.0) if compact else (11.75, 8.0)
    spacing, half_finger = (2.0296, 0.2537) if compact else (3.0, 0.2)
    points = [[wall, y] for y in np.linspace(-1.5 * spacing, 1.5 * spacing, 180)]
    for y in np.array((-1.5, -0.5, 0.5, 1.5)) * spacing:
        for side in (-half_finger, half_finger):
            points += [[x, y + side] for x in np.linspace(entrance, wall, 80)]
        points += [
            [entrance, yy] for yy in np.linspace(y - half_finger, y + half_finger, 12)
        ]
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    points = np.array(points) @ rotation.T + offset
    points += rng.normal(0, 0.008, points.shape)
    camera = np.array([0.1, 0.0])
    beacons = (
        np.array([[wall - 0.15, y] for y in (spacing, 0, -spacing)]) @ rotation.T
        + offset
    )
    pixels = {}
    for color, p in zip(colors, beacons):
        pixels.setdefault(color, []).append(
            320 - 310 * (p[1] - camera[1]) / (p[0] - camera[0]),
        )
    return points, pixels, camera, rotation


class PerceptionTests(unittest.TestCase):
    def test_two_metre_fingers_in_each_bay(self):
        for index in range(3):
            colors = ["red"] * 3
            colors[index] = "green"
            points, pixels, camera, rotation = scene(
                yaw=0.08,
                offset=(0.4, 1.0),
                colors=colors,
                compact=True,
            )
            with self.subTest(bay=index + 1):
                bay = detect_bay(points, pixels, 310, 320, camera, 0)
                self.assertIsNotNone(bay)
                expected = rotation @ np.array([9.75, (1 - index) * 2.0296]) + [
                    0.4,
                    1.0,
                ]
                np.testing.assert_allclose(bay.goal, expected, atol=0.16)
                self.assertEqual(bay.number, index + 1)

    def test_recorded_platform_collinear_with_finger(self):
        # Seed 1 of the compact dock: RANSAC combines a platform and a finger.
        data = np.load(Path(__file__).parent / "data" / "dock_with_platform.npz")
        bay = detect_bay(
            data["points"],
            {"red": list(data["red"]), "green": list(data["green"])},
            float(data["fx"]),
            float(data["cx"]),
            data["camera"],
            float(data["yaw"]),
        )
        self.assertIsNotNone(bay)
        self.assertEqual(bay.number, 1)
        # Independent world pose: translation (0.4031, 1.0423), yaw 0.07913.
        np.testing.assert_allclose(bay.goal, [10.091, 3.847], atol=0.16)
        self.assertGreater(bay.entrance, 9.0)

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
        image[30:40, 20:30] = [255, 0, 0]
        image[30:40, 80:90] = [0, 255, 0]
        image[30:40, 140:150] = [255, 0, 0]
        pixels = color_centers(image)
        self.assertEqual(pixels, {"red": [24.5, 144.5], "green": [84.5]})
        image[80:90, 140:150] = [0, 255, 0]
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
                image[30:37, x - 3 : x + 4] = (
                    [0, 255, 0] if color == "green" else [255, 0, 0]
                )
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


# Median RGB of the green indicator in the handbook render; the real green is dark.
RENDER_GREEN = [46, 94, 51]
# Handbook delivery-platform circle colour #089404.
PLATFORM_GREEN = [8, 148, 4]


class IndicatorShapeTests(unittest.TestCase):
    def test_dark_real_indicator_green_is_found(self):
        image = np.zeros((60, 80, 3), np.uint8)
        image[20:32, 30:42] = RENDER_GREEN
        self.assertEqual(color_centers(image), {"green": [35.5]})

    def test_shaded_real_indicator_green_is_found(self):
        image = np.zeros((60, 80, 3), np.uint8)
        image[20:32, 30:42] = [28, 56, 31]
        self.assertEqual(color_centers(image), {"green": [35.5]})

    def test_washed_out_indicator_green_is_found(self):
        # Gazebo's lighting renders the indicator at this colour, saturation 78.
        image = np.zeros((60, 80, 3), np.uint8)
        image[20:32, 30:42] = [52, 75, 55]
        self.assertEqual(color_centers(image), {"green": [35.5]})

    def test_tall_green_bar_is_rejected(self):
        image = np.zeros((60, 80, 3), np.uint8)
        image[10:50, 30:34] = PLATFORM_GREEN
        self.assertEqual(color_centers(image), {})

    def test_hollow_green_outline_is_rejected(self):
        image = np.zeros((60, 80, 3), np.uint8)
        image[10:30, 10:30] = PLATFORM_GREEN
        image[12:28, 12:28] = 0
        self.assertEqual(color_centers(image), {})

    def test_flat_platform_circle_is_rejected(self):
        image = np.zeros((60, 120, 3), np.uint8)
        cv2.ellipse(image, (60, 30), (40, 4), 0, 0, 360, PLATFORM_GREEN, -1)
        self.assertEqual(color_centers(image), {})


if __name__ == "__main__":
    unittest.main()
