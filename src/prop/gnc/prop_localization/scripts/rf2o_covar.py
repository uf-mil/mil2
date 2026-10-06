#!/usr/bin/env python3
"""Republish rf2o laser odometry with a usable covariance matrix.

rf2o_laser_odometry leaves pose.covariance and twist.covariance filled with
zeros. robot_localization does not reject a zero covariance -- it clamps the
diagonal to a tiny epsilon, which reads as an almost perfect measurement. The
filter then snaps onto the laser odometry, effectively ignoring the GPS and
IMU, and frequently goes numerically unstable.

This node is a pass-through that stamps a fixed diagonal covariance on the
way past. It does not touch the pose or twist values themselves.

Tuning: the variances are what tell the EKF how much to trust rf2o relative to
everything else. Defaults below assume a reasonable scan match on a vessel with
usable structure in view. If the fused estimate lags the laser, lower them; if
it gets yanked around by bad scan matches, raise them. On open water with
nothing to match against, rf2o output is close to meaningless -- consider
gating this node off rather than widening the covariance to compensate.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

# Indices of the diagonal in a 6x6 row-major covariance array.
DIAGONAL = (0, 7, 14, 21, 28, 35)

# Large value for unobserved dimensions, per robot_localization convention.
UNOBSERVED = 1e6


class Rf2oCovarianceRelay(Node):
    def __init__(self) -> None:
        super().__init__("rf2o_covariance_relay")

        self.declare_parameter("input_topic", "/odom_rf2o")
        self.declare_parameter("output_topic", "/lidar/odom")

        # Order: x, y, z, roll, pitch, yaw
        self.declare_parameter(
            "pose_variances",
            [0.001, 0.001, UNOBSERVED, UNOBSERVED, UNOBSERVED, 0.003],
        )
        # Order: vx, vy, vz, vroll, vpitch, vyaw
        self.declare_parameter(
            "twist_variances",
            [0.0002, 0.0002, UNOBSERVED, UNOBSERVED, UNOBSERVED, 0.0001],
        )

        # Optional: rewrite frame ids so rf2o's own "odom" frame does not
        # collide with the frame the EKF owns. Leave empty to pass through.
        self.declare_parameter("frame_id", "")
        self.declare_parameter("child_frame_id", "")

        self._pose_var = list(
            self.get_parameter("pose_variances")
            .get_parameter_value()
            .double_array_value,
        )
        self._twist_var = list(
            self.get_parameter("twist_variances")
            .get_parameter_value()
            .double_array_value,
        )
        self._frame_id = self.get_parameter("frame_id").value
        self._child_frame_id = self.get_parameter("child_frame_id").value

        out_topic = self.get_parameter("output_topic").value
        in_topic = self.get_parameter("input_topic").value

        self._pub = self.create_publisher(Odometry, out_topic, 10)
        self._sub = self.create_subscription(Odometry, in_topic, self._on_odom, 10)

        self.get_logger().info(f"relaying {in_topic} -> {out_topic} with covariance")

    def _on_odom(self, msg: Odometry) -> None:
        for slot, variance in zip(DIAGONAL, self._pose_var):
            msg.pose.covariance[slot] = variance
        for slot, variance in zip(DIAGONAL, self._twist_var):
            msg.twist.covariance[slot] = variance

        if self._frame_id:
            msg.header.frame_id = self._frame_id
        if self._child_frame_id:
            msg.child_frame_id = self._child_frame_id

        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = Rf2oCovarianceRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
