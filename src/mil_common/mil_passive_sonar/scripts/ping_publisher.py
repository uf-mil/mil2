#!/usr/bin/env python3

import json
import socket

import numpy as np
import rclpy
import transforms3d
from geometry_msgs.msg import PoseStamped
from mil_msgs.msg import ProcessedPing


def main():
    HOST = "127.0.0.1"
    PORT = 2007

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        pub = node.create_publisher(ProcessedPing, "hydrophones/solved", 10)
        pose_pub = node.create_publisher(PoseStamped, "hydrophones/pose", 10)
        s.connect((HOST, PORT))
        node.get_logger().info(
            f"\nping_publisher connected to {HOST}:{PORT}, forwarding TCP messages to {pub.topic_name}...",
        )

        # Need to ignore the first 2 JSON Parse Errors (nothing wrong)
        parse_error_count = 0

        while rclpy.ok():
            data = s.recv(1024)
            if not data:
                break

            try:
                json_data = json.loads(data.decode("utf-8"))

                ping_msg = ProcessedPing()
                x = float(
                    json_data["origin_direction_body"][0],
                )
                y = -float(
                    json_data["origin_direction_body"][1],
                )
                z = -float(
                    json_data["origin_direction_body"][2],
                )
                ping_msg.origin_direction_body.x = x
                ping_msg.origin_direction_body.y = y
                ping_msg.origin_direction_body.z = z
                ping_msg.frequency = int(json_data["frequency_Hz"])
                ping_msg.origin_distance_m = float(json_data["origin_distance_m"])

                pub.publish(ping_msg)

                if False:  # 25_000 <= ping_msg.frequency <= 35_000:
                    pose_msg = PoseStamped()
                    pose_msg.header.frame_id = "base_link"
                    # calculate quaternion
                    vec = np.array([x, y, z])
                    left = np.cross(np.array([0, 0, 1]), vec)
                    up = np.cross(vec, left)

                    mat = np.array([vec, left, up]).T
                    quat = transforms3d.quaternions.mat2quat(mat)
                    pose_msg.pose.orientation.w = quat[0]
                    pose_msg.pose.orientation.x = quat[1]
                    pose_msg.pose.orientation.y = quat[2]
                    pose_msg.pose.orientation.z = quat[3]
                    pose_pub.publish(pose_msg)
            except json.JSONDecodeError as e:
                parse_error_count += 1
                # ignore first two (normal behavior)
                if parse_error_count > 2:
                    node.get_logger().error(f"JSONDecodeError: {e}")
            except KeyError as e:
                node.get_logger().error(f"Key Error: {e}")


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("pingpublisher")
    rclpy.get_global_executor().add_node(node)
    main()
