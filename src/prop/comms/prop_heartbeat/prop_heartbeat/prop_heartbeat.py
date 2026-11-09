#!/usr/bin/env python3
"""
The boat's heartbeat, which the OCS turns into RoboCommand's, at 2 Hz.

  in   gps/fix                sensor_msgs/NavSatFix  position
       imu/data               sensor_msgs/Imu        heading, roll, pitch
       odometry               nav_msgs/Odometry      speed
       /manual_control        std_msgs/Bool          true manual, false autonomous
       /rf_kill/manual        std_msgs/Bool          the same, as the RF kill board reports it
       /mission/current_task  std_msgs/String        SAFE_PASSAGE, NONE, ...
  out  /heartbeats            mil_msgs/Heartbeat

For the boat, position.x is longitude and position.y latitude, in degrees, and
NaN until there is a fix. heading, roll and pitch are radians, heading being the
IMU's ENU yaw: the IMU, not odometry, because ekf_local's yaw starts at zero
wherever the boat pointed when it came up.

/manual_control is what prop-auto and prop-manual set, and piss_tower passes on
to the RF kill board; /rf_kill/manual is the mode piss_tower hears back from the
board, which also changes when the remote does. Whichever said something last
is the state, which is MANUAL until one says false.
Nothing on the boat reports the kill switch, so KILLED is never sent.

A mission says which task it is on with /mission/current_task, latched, and the
change into a task is what starts that task's attempt on RoboCommand.
"""

import math

import rclpy
from mil_msgs.msg import Heartbeat
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
STALE_AFTER = 2.0  # s


def euler(q):
    """Roll, pitch and yaw of a quaternion, in radians."""
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


class PropHeartbeat(Node):
    def __init__(self):
        super().__init__("prop_heartbeat")
        self.fix = self.imu = self.odom = None
        self.manual = True
        self.task = "NONE"

        self.heartbeat_pub = self.create_publisher(Heartbeat, "/heartbeats", 10)
        self.create_subscription(NavSatFix, "gps/fix", self.on_fix, 10)
        self.create_subscription(Imu, "imu/data", self.on_imu, 10)
        self.create_subscription(Odometry, "odometry", self.on_odom, 10)
        self.create_subscription(String, "/prop/state", self.on_manual, LATCHED)
        self.create_subscription(Bool, "/rf_kill/manual", self.on_manual, LATCHED)
        self.create_subscription(String, "/mission/current_task", self.on_task, LATCHED)
        self.create_timer(0.5, self.publish)

    def on_fix(self, msg):
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self.fix = (self.now(), msg)

    def on_imu(self, msg):
        self.imu = (self.now(), msg)

    def on_odom(self, msg):
        self.odom = (self.now(), msg)

    def on_manual(self, msg):
        self.manual = msg.data

    def on_task(self, msg):
        self.task = msg.data

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def fresh(self, stamped):
        return stamped is not None and self.now() - stamped[0] < STALE_AFTER

    def publish(self):
        heartbeat = Heartbeat()
        heartbeat.type = "USV"
        heartbeat.state = self.manual
        heartbeat.task = self.task

        heartbeat.position.x = heartbeat.position.y = math.nan
        if self.fresh(self.fix):
            heartbeat.position.x = self.fix[1].longitude
            heartbeat.position.y = self.fix[1].latitude

        heartbeat.heading = heartbeat.roll = heartbeat.pitch = math.nan
        if self.fresh(self.imu):
            roll, pitch, yaw = euler(self.imu[1].orientation)
            heartbeat.heading, heartbeat.roll, heartbeat.pitch = yaw, roll, pitch

        heartbeat.speed = math.nan
        if self.fresh(self.odom):
            v = self.odom[1].twist.twist.linear
            heartbeat.speed = math.hypot(v.x, v.y)

        self.heartbeat_pub.publish(heartbeat)


def main():
    rclpy.init()
    node = PropHeartbeat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
