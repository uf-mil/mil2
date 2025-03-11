# Right now, it only subscribes to /front_cam/image_raw topic
# However, more sensor data and subscribers to be incorporated in the future
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import threading
import time
import cv2
from cv_bridge import CvBridge

class RL_subscriber(Node):
    def __init__(self):
        super().__init__('RL_subscriber')
        self.image_subscription = self.create_subscription(
            Image,
            '/front_cam/image_raw', #topic for sub9 cam
            self.image_callback,
            10
        )
        self.lock = threading.Lock()
        self.image_subscription
        self.front_cam_image = None        

    def image_callback(self, msg):
        self.get_logger().info("Image received")
        with self.lock:
            self.front_cam_image = msg

    def get_image(self):
        with self.lock:
            return self.front_cam_image
        
def main(args=None):
    rclpy.init(args=args)
    cvBridge = CvBridge()

    # Declare node and spin it
    node = RL_subscriber()
    threading.Thread(target=rclpy.spin, args=(node,)).start()
    while True:
        time.sleep(1)
        cv2.destroyAllWindows()
        cv2.imshow("Front cam", cvBridge.imgmsg_to_cv2(node.get_image(), "bgr8"))
        cv2.waitKey(40)

        # Simple max pooling algorithm for image
        net = cv2.dnn.Net()
        params = {
            "kernel_w": 3,
            "kernel_h": 3,
            "stride_w": 3,
            "stride_h": 3,
            "pool": "max",
        }
        net.addLayerToPrev("pool", "Pooling", cv2.CV_32F, params)

        image = cvBridge.imgmsg_to_cv2(node.get_image(), "bgr8")
        net.setInput(cv2.dnn.blobFromImage(image))
        out = net.forward()

        cv2.imshow("Pooled image", cv2.dnn.imagesFromBlob(out)[0].astype("uint8"))
        cv2.waitKey(40)


    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
