"""Publishes the static transforms that place every sensor in sensors.yaml."""

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from prop_fusion.mounts import Sensors, default_sensors_file
from prop_fusion.run import run_until_stopped


def transform(mount, stamp):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = mount.parent
    msg.child_frame_id = mount.child
    t, q = msg.transform.translation, msg.transform.rotation
    t.x, t.y, t.z = mount.xyz
    q.x, q.y, q.z, q.w = mount.quat
    return msg


class SensorTf(Node):
    def __init__(self):
        super().__init__("sensor_tf")
        path = self.declare_parameter("sensors_file", default_sensors_file()).value
        sensors = Sensors.load(path)

        stamp = self.get_clock().now().to_msg()
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform([transform(m, stamp) for m in sensors.mounts])

        frames = ", ".join(m.child for m in sensors.mounts)
        self.get_logger().info(f"{path}: {sensors.base_frame} -> {frames}")


def main():
    run_until_stopped(SensorTf)


if __name__ == "__main__":
    main()
