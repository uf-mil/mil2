import rclpy
from rclpy.node import Node

from mavros_msgs.srv import SetMode, CommandBool
# from mavros_msgs.msg import NavSatFix # lmao this is actually wrong
from sensor_msgs.msg import NavSatFix

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import LivelinessPolicy

from geographic_msgs.msg import GeoPoseStamped

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
    liveliness=LivelinessPolicy.AUTOMATIC
)

# Infinite durations are default, but you can set explicitly like this:
from rclpy.duration import Duration

qos_profile.lifespan = Duration(seconds=0)   # 0 = infinite
qos_profile.deadline = Duration(seconds=0)   # 0 = infinite
qos_profile.liveliness_lease_duration = Duration(seconds=0)  # infinite

class MavrosStartupNode(Node):
    def __init__(self):
        super().__init__('mavros_startup_node')

        # listen to lat and long
        self.gps_pos_sub = self.create_subscription(NavSatFix, "/mavros/global_position/global", self.gps_pos_cb, qos_profile)
        self.gps_pos = NavSatFix()

        # tell where to go
        self.gps_pub = self.create_publisher(GeoPoseStamped, "/mavros/setpoint_position/global", qos_profile)

        # Create service clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.get_logger().info("Waiting for MAVROS services...")

        # Wait for services to be available
        self.set_mode_client.wait_for_service()
        self.arming_client.wait_for_service()

        self.get_logger().info("Services available. Sending requests...")

        # Call services
        self.set_mode()

    def gps_pos_cb(self, msg):
        self.gps_pos = msg

    def set_mode(self):
        req = SetMode.Request()
        req.custom_mode = "GUIDED"

        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.set_mode_callback)

    def arm_vehicle(self):
        req = CommandBool.Request()
        req.value = True

        future = self.arming_client.call_async(req)
        future.add_done_callback(self.arm_callback)

    def set_mode_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Set mode response: mode_sent={response.mode_sent}")
            self.arm_vehicle()
        except Exception as e:
            self.get_logger().error(f"Set mode call failed: {e}")

    def arm_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Arming response: success={response.success}, result={response.result}"
            )

            self.create_timer(5, self.goto_gps_pos)
        except Exception as e:
            self.get_logger().error(f"Arming call failed: {e}")

    def goto_gps_pos(self):
        msg = GeoPoseStamped()
        msg.header = self.gps_pos.header

        msg.pose.position.latitude = self.gps_pos.latitude
        msg.pose.position.longitude = self.gps_pos.longitude
        msg.pose.position.altitude = self.gps_pos.altitude

        self.get_logger().info(f"goto lat: {msg.pose.position.latitude}; lon: {msg.pose.position.longitude}; altitude: {msg.pose.position.altitude};")

        # msg.pose.orientation.x =
        # msg.pose.orientation.y =
        # msg.pose.orientation.z =
        # msg.pose.orientation.w =

        self.gps_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MavrosStartupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
