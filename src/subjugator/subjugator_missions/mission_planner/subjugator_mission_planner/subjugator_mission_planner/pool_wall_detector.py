"""
Pool wall side detector for Subjugator initialization.

Subscribes to the front camera, uses Canny edge detection + Hough line
transform to find near-vertical lines (the pool wall), and determines
whether the wall is on the LEFT or RIGHT side of the sub.

Exposes a ROS2 service /pool_wall_side (std_srvs/Trigger):
  - success=True  → wall detected; message is "LEFT" or "RIGHT"
  - success=False → not enough data yet; message is "UNKNOWN"
"""

import math
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

# Near-vertical line: angle between 70° and 110° (0°=horizontal, 90°=vertical)
VERTICAL_ANGLE_MIN = 70.0
VERTICAL_ANGLE_MAX = 110.0

# Minimum line length (pixels) to count a line as a wall segment
MIN_LINE_LENGTH = 40

# Number of frames to accumulate votes over before returning a stable answer
VOTE_BUFFER_SIZE = 15

# Minimum fraction of frames that must agree for a confident answer
CONFIDENCE_THRESHOLD = 0.6


class PoolWallDetector(Node):
    def __init__(self):
        super().__init__("pool_wall_detector")

        self.bridge = CvBridge()

        # When True, horizontally flip the frame before processing.
        # Set to True for Gazebo sim (no 180° rotation applied by camera driver).
        # Set to False for real hardware (front_cam_driver.py already rotates 180°).
        self.declare_parameter("mirror", True)
        self._mirror = self.get_parameter("mirror").get_parameter_value().bool_value

        # Ring buffer of per-frame votes: "LEFT", "RIGHT", or None
        self._votes: deque[str | None] = deque(maxlen=VOTE_BUFFER_SIZE)

        self.create_subscription(
            Image,
            "front_cam/image_raw",
            self._image_callback,
            10,
        )

        self._srv = self.create_service(
            Trigger,
            "pool_wall_side",
            self._service_callback,
        )

        self.get_logger().info("Pool wall detector ready — service: /pool_wall_side")

    # Image processing

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        if self._mirror:
            frame = cv2.flip(
                frame,
                1,
            )  # horizontal flip to match real camera orientation

        vote = self._detect_wall_side(frame)
        self._votes.append(vote)

    def _detect_wall_side(self, frame: np.ndarray) -> str | None:
        """
        Returns "LEFT", "RIGHT", or None if no wall lines are found.

        Strategy:
          1. Grayscale + Gaussian blur to reduce noise.
          2. Canny edge detection.
          3. Probabilistic Hough line transform.
          4. Keep only near-vertical lines (70-110°).
          5. Compute the length-weighted mean x-centre of those lines.
          6. Compare to the image midpoint.
        """
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=60,
            minLineLength=MIN_LINE_LENGTH,
            maxLineGap=15,
        )

        if lines is None:
            return None

        weighted_x_sum = 0.0
        total_weight = 0.0

        for line in lines:
            x1, y1, x2, y2 = line[0]

            length = math.hypot(x2 - x1, y2 - y1)
            if length == 0:
                continue

            # Angle relative to horizontal (0° = horizontal, 90° = vertical)
            angle = math.degrees(math.atan2(abs(y2 - y1), abs(x2 - x1)))
            # atan2 gives 0-90; map to 0-180 by treating vertical as ~90°
            # A line with dx≈0 gives angle≈90 → near-vertical.
            if not (VERTICAL_ANGLE_MIN <= angle <= VERTICAL_ANGLE_MAX):
                continue

            center_x = (x1 + x2) / 2.0
            weighted_x_sum += center_x * length
            total_weight += length

        if total_weight == 0:
            return None

        mean_x = weighted_x_sum / total_weight
        return "LEFT" if mean_x < w / 2 else "RIGHT"

    # ------------------------------------------------------------------
    # Service
    # ------------------------------------------------------------------

    def _service_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if len(self._votes) < VOTE_BUFFER_SIZE:
            frames_needed = VOTE_BUFFER_SIZE - len(self._votes)
            response.success = False
            response.message = (
                f"UNKNOWN — still collecting data ({frames_needed} frames remaining)"
            )
            self.get_logger().info(response.message)
            return response

        valid_votes = [v for v in self._votes if v is not None]
        if not valid_votes:
            response.success = False
            response.message = "UNKNOWN — no vertical lines detected in any frame"
            self.get_logger().info(response.message)
            return response

        left_count = valid_votes.count("LEFT")
        right_count = valid_votes.count("RIGHT")
        total = len(valid_votes)

        left_frac = left_count / total
        right_frac = right_count / total

        if left_frac >= CONFIDENCE_THRESHOLD:
            side = "LEFT"
        elif right_frac >= CONFIDENCE_THRESHOLD:
            side = "RIGHT"
        else:
            side = "UNKNOWN"

        response.success = side != "UNKNOWN"
        response.message = (
            f"{side} (left={left_count}/{total}, right={right_count}/{total})"
        )
        self.get_logger().info(f"Pool wall side: {response.message}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PoolWallDetector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
