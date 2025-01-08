#!/usr/bin/env python3

import contextlib
import json
import socket

import rclpy
from geometry_msgs.msg import Point
from mil_passive_sonar.msg import ProcessedPing
from std_msgs.msg import Header


def main():
    # Define the server address and port
    HOST = "127.0.0.1"
    PORT = 2007

    # Create a socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        pub = node.create_publisher(ProcessedPing, 'hydrophones/solved', 10)
        s.connect((HOST, PORT))
        node.get_logger().info(
            f"\nping_publisher connected to {HOST}:{PORT}, forwarding TCP messages to {pub.topic_name}...",
        )

        # Need to ignore the first 2 JSON Parse Errors (nothing wrong)
        parse_error_count = 0

        while rclpy.ok():
            # Receive data
            data = s.recv(1024)
            if not data:
                break

            # Parse the JSON data
            try:
                json_data = json.loads(data.decode("utf-8"))
                ping_msg = ProcessedPing()

                # Populate the header
                ping_msg.header = Header()
                ping_msg.header.stamp = node.get_clock().now().to_msg()
                ping_msg.header.frame_id = "hydrophones"

                # Populate the position
                ping_msg.position = Point()
                ping_msg.position.x = json_data["origin_direction_body"][0]
                ping_msg.position.y = json_data["origin_direction_body"][1]
                ping_msg.position.z = json_data["origin_direction_body"][2]

                # Populate the frequency and amplitude
                ping_msg.freq = float(json_data["frequency_Hz"])
                ping_msg.amplitude = float(json_data["origin_distance_m"])
                ping_msg.valid = True

                # Publish the message
                pub.publish(ping_msg)
            except json.JSONDecodeError as e:
                parse_error_count += 1
                # ignore first two (normal behavior)
                if parse_error_count > 2:
                    node.get_logger().error(f"JSONDecodeError: {e}")
            except KeyError as e:
                node.get_logger().error(f"Key Error: {e}")


if __name__ == "__main__":
    # initialize rclpy
    rclpy.init()

    # create publisher node
    node = rclpy.create_node('pingpublisher')
    rclpy.get_global_executor().add_node(node)
    
    main()
