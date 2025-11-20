#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import torch
import numpy as np
import cv2

from transformers import pipeline
from PIL import Image as PILImage


class DepthAnythingNode(Node):
    def __init__(self):
        super().__init__("depth_anything_v2_node")

        self.get_logger().info("===== DEPTH ANYTHING V2 DEBUG NODE STARTING =====")

        # Device selection
        device = 0 if torch.cuda.is_available() else -1
        self.get_logger().info(f"[INIT] Using device: {'cuda' if device == 0 else 'cpu'}")

        # Load model with logging
        model_id = "depth-anything/Depth-Anything-V2-Small-hf"
        self.get_logger().info(f"[INIT] Loading model: {model_id}")

        try:
            self.pipe = pipeline(
                task="depth-estimation",
                model=model_id,
                device=device,
            )
            self.get_logger().info("[INIT] Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"[INIT] FAILED to load model: {e}")
            raise

        # ROS setup
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, "/front_cam/image_raw", self.image_callback, 10
        )

        self.depth_pub = self.create_publisher(
            Image, "/depth_anything_v2/depth", 10
        )

        self.get_logger().info("[INIT] Node initialized and waiting for images...")


    def image_callback(self, msg: Image):
        self.get_logger().info("===== CALLBACK START =====")

        try:
            # Convert ROS → CV image
            cv_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.get_logger().info(f"[STEP] Received image: {cv_bgr.shape}")

        except Exception as e:
            self.get_logger().error(f"[ERROR] Converting ROS → CV: {e}")
            return

        try:
            # Convert BGR → RGB → PIL
            cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
            pil_img = PILImage.fromarray(cv_rgb)
            self.get_logger().info("[STEP] Converted to PIL")

        except Exception as e:
            self.get_logger().error(f"[ERROR] Converting to PIL: {e}")
            return

        try:
            # ---- RUN DEPTH ANYTHING ----
            self.get_logger().info("[STEP] Running DepthAnythingV2...")
            out = self.pipe(pil_img)
            self.get_logger().info("[STEP] Pipeline returned output")

        except Exception as e:
            self.get_logger().error(f"[ERROR] MODEL INFERENCE FAILED: {e}")
            return

        try:
            depth_pil = out["depth"]
            depth = np.array(depth_pil, dtype=np.float32)

            self.get_logger().info(f"[STEP] Depth map shape: {depth.shape}")

        except Exception as e:
            self.get_logger().error(f"[ERROR] Processing depth output: {e}")
            return

        try:
            # Resize to match input
            h, w = cv_bgr.shape[:2]
            if depth.shape[0] != h or depth.shape[1] != w:
                depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_LINEAR)

            depth = np.maximum(depth, 0.0).astype(np.float32)

            self.get_logger().info("[STEP] Depth resized + normalized")

        except Exception as e:
            self.get_logger().error(f"[ERROR] Resizing depth: {e}")
            return

        try:
            # Convert to ROS2 image
            depth_msg = Image()
            depth_msg.header = msg.header
            depth_msg.height = depth.shape[0]
            depth_msg.width = depth.shape[1]
            depth_msg.encoding = "32FC1"
            depth_msg.is_bigendian = False
            depth_msg.step = depth.shape[1] * 4
            depth_msg.data = depth.tobytes()

            self.depth_pub.publish(depth_msg)

            self.get_logger().info("[STEP] Depth image published!")

        except Exception as e:
            self.get_logger().error(f"[ERROR] Publishing depth image: {e}")

        self.get_logger().info("===== CALLBACK END =====")


def main(args=None):
    rclpy.init(args=args)
    node = DepthAnythingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()