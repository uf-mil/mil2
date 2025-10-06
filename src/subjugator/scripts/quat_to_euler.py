import math
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class FloatPublisher(Node):
    def __init__(self):
        super().__init__("float_publisher")
        self.publisher_ = self.create_publisher(Float32, "/hi_adam", 10)

    def publish_message(self, msgg):
        msg = Float32()
        msg.data = msgg
        self.publisher_.publish(msg)


def quaternion_to_euler_xyz(x, y, z, w):
    """
    Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw) in radians.
    Returns: (roll, pitch, yaw)
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw  # In radians


def echo_ros2_topic(topic_name, xx):
    try:
        # Start the subprocess to run 'ros2 topic echo <topic_name>'
        process = subprocess.Popen(
            [
                "ros2",
                "topic",
                "echo",
                "/odometry/filtered",
                "--field",
                "pose.pose.orientation",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,  # Ensures output is decoded as text (str)
        )

        print(f"Listening to topic: {topic_name}\nPress Ctrl+C to stop.\n")

        buffer = []
        # Continuously read line by line
        while True:
            line = process.stdout.readline()
            if not line:
                break  # End of stream

            line = line.strip()

            if line:  # Skip empty lines
                buffer.append(line)

                # Once we collect 4 lines, process the batch
                if len(buffer) == 5:
                    x = float(buffer[0][3:])
                    y = float(buffer[1][3:])
                    z = float(buffer[2][3:])
                    w = float(buffer[3][3:])

                    roll, pitch, yaw = quaternion_to_euler_xyz(x, y, z, w)
                    xx.publish_message(math.degrees(yaw))
                    print(
                        f"Roll: {math.degrees(roll):.2f}°, Pitch: {math.degrees(pitch):.2f}°, Yaw: {math.degrees(yaw):.2f}°",
                    )

                    buffer = []

    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting.")
        process.terminate()
    except Exception as e:
        print(f"Error: {e}")


# Replace '/your/topic' with your actual topic name
if __name__ == "__main__":
    rclpy.init()
    x = FloatPublisher()
    echo_ros2_topic("/your/topic", x)
