# AI literally wrote this whole file :P

import tkinter as tk

# Get screen resolution
root = tk.Tk()
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
root.destroy()

import rclpy
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image  # Change this to your message type

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,  # Replace with your message type if different
            "/vs_some_name_here/some_name_here/binThresholding_0",
            self.listener_callback,
            10  # QoS history depth
        )
        self.get_logger().info('Subscriber node started, listening on /image_topic')
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rotated_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)

            # Limit image size to screen dimensions
            max_width = int(screen_width * 0.9)
            max_height = int(screen_height * 0.9)

            height, width = cv_image.shape[:2]
            scale = min(max_width / width, max_height / height)
            new_size = (int(width * scale), int(height * scale))

            resized_image = cv2.resize(rotated_image, new_size, interpolation=cv2.INTER_AREA)

            # Display the image using OpenCV
            cv2.imshow("Received Image", resized_image)
            cv2.waitKey(1)  # 1ms delay for the image window to update

        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
