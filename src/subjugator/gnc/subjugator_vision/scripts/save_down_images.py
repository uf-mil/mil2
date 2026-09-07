#!/usr/bin/env python3

import os
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class DownCamSaver(Node):
    def __init__(self):
        super().__init__("down_cam_saver")

        default_dir = os.path.join(os.path.expanduser("~"), "sim_images/down")
        self.declare_parameter("save_dir", default_dir)
        self.declare_parameter(
            "save_every",
            0,
        )  # 0 is save on Enter, >0 also auto saves every Nth frame

        self.save_every = self.get_parameter("save_every").value
        self.count = 0

        self.dir = os.path.abspath(
            os.path.expanduser(self.get_parameter("save_dir").value),
        )

        os.makedirs(self.dir, exist_ok=True)
        self.bridge = CvBridge()
        self.latest_msg = None
        self.lock = threading.Lock()
        self.saved = 0

        topic = "/down_cam/image_raw"
        self.create_subscription(Image, topic, self.callback, 10)

        mode = (
            f"burst every {self.save_every} frames + Enter"
            if self.save_every > 0
            else "Enter only"
        )
        self.get_logger().info(
            f"DownCamSaver: subscribed to {topic}, saving to {self.dir}. "
            f"Mode: {mode}. Press Enter to save current frame, Ctrl-C to quit.",
        )

    def callback(self, msg: Image):
        with self.lock:  # Cache most recent frame
            self.latest_msg = msg

        # Burst path if enabled
        if self.save_every > 0:
            self.count += 1
            if self.count % self.save_every == 0:
                self.save_msg(msg, "burst")

    def save_latest(self):
        # Manual path
        with self.lock:
            msg = self.latest_msg
        if msg is None:
            self.get_logger().warn("No frame received yet; nothing to save.")
            return
        self.save_msg(msg, "manual")

    def save_msg(self, msg: Image, reason: str):
        # Shared write logic used by both the burst and manual path
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        stamp = self.get_clock().now().to_msg()
        filename = f"down_cam_{stamp.sec}_{stamp.nanosec}.png"
        cv2.imwrite(os.path.join(self.dir, filename), img)
        self.saved += 1
        self.get_logger().info(f"Saved {filename}  ({reason}, total: {self.saved})")


def main():
    rclpy.init()
    node = DownCamSaver()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while True:
            input()
            node.save_latest()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
