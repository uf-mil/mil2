import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import depthai as dai
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
# from vision_stack import VisionStack # sorry Daniel

class DepthAICameraNode(Node):
    def __init__(self):
        super().__init__('depthai_camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.other_pub = self.create_publisher(PoseWithCovarianceStamped, 'i/am/joe/handsome', 10)
        self.bridge = CvBridge()

        # Set up DepthAI pipeline
        self.pipeline = dai.Pipeline()
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(640, 480)
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(30)

        xout = self.pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("video")
        cam_rgb.preview.link(xout.input)

        self.device = dai.Device(self.pipeline)
        self.video_queue = self.device.getOutputQueue(name="video")

        # Timer to run at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        in_frame = self.video_queue.tryGet()
        if in_frame is not None:
            frame = in_frame.getCvFrame() # lsp angy
            # find orange center

            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define orange color range in HSV
            lower_orange = np.array([30, 20, 20])
            upper_orange = np.array([80, 255, 255])

            # Create a mask for orange
            mask = cv2.inRange(hsv, lower_orange, upper_orange)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Get the largest contour
                largest = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest)

                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    print(f"Orange blob center: ({cx}, {cy})")
                    final_x = cx/700
                    final_y = cx/500

                    msg = PoseWithCovarianceStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "odom"

                    msg.pose.pose.position.x = final_x
                    msg.pose.pose.position.y = final_y 

                    msg.pose.covariance = [
                        0.1, 0,   0,   0, 0, 0,
                        0,   0.1, 0,   0, 0, 0,
                        0,   0,   999, 0, 0, 0,
                        0,   0,   0,   999, 0, 0,
                        0,   0,   0,   0, 999, 0,
                        0,   0,   0,   0, 0, 0.1
                    ]
                    self.other_pub.publish(msg)


                    # Draw it
                    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                    cv2.drawContours(frame, [largest], -1, (0, 255, 255), 2)

            # Show results
            cv2.imshow("Frame", frame)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)

        
            # pub it
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthAICameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
