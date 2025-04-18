# Right now, it only subscribes to /front_cam/image_raw topic
# However, more sensor data and subscribers to be incorporated in the future
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import threading
import time
import cv2
from cv_bridge import CvBridge
import os

cvBridge = CvBridge()

class CamSubscriber(Node):
    def __init__(self):
        super().__init__('cam_subscriber')
        self.cam_subscription = self.create_subscription(
            Image,
            '/front_cam/image_raw', #topic for sub9 cam
            self.image_callback,
            10
        )
        self.cam_subscription
        self.cam_data = None
    

    def image_callback(self, msg):
        time.sleep(0.5) # Delay for image callback
        self.get_logger().info("Image received")

        cv2.destroyAllWindows()
        # Front cam image size is 600x960, before max pooling
        cv2.imshow("Front cam", cvBridge.imgmsg_to_cv2(msg, "bgr8"))
        cv2.waitKey(40)

        # Simple max pooling algorithm for image
        pool_size = 12 
        # Divide original image dimensions by pool size to get pooled image dimensions
        net = cv2.dnn.Net()
        params = {
            "kernel_w": pool_size,
            "kernel_h": pool_size,
            "stride_w": pool_size,
            "stride_h": pool_size,
            "pool": "max",
        }
        net.addLayerToPrev("pool", "Pooling", cv2.CV_32F, params)

        image = cvBridge.imgmsg_to_cv2(msg, "bgr8")
        net.setInput(cv2.dnn.blobFromImage(image))
        out = net.forward()

        pooled_image = cv2.dnn.imagesFromBlob(out)[0].astype("uint8")
        cv2.imshow("Pooled image", pooled_image)
        cv2.waitKey(20)

        self.cam_data = pooled_image
        time.sleep(0.1)
        
cam_node = CamSubscriber()

def run(args=None):
    def spin():
        # Declare node and spin it
        rclpy.spin(cam_node)
        cam_node.destroy_node()
        rclpy.shutdown()
        
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
