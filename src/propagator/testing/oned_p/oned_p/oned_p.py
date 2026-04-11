import rclpy
from rclpy.node import Node

from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import NavSatFix 

from nav_msgs.msg import Odometry

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

        # listen to pose
        self._rel_odom_sub = self.create_subscription(
            Odometry,
            "infix2/odom/rel",
            self.odom_rel_cb,
            10,
        )
        self.last_odom = Odometry()

        # tell where to go
        # don't use this! use self.send_pwm instead!
        self._raw_pwm_pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)

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

    def send_pwm(self, left=1500, right=1500):
        # https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/OverrideRCIn.html
        # uint16 CHAN_NOCHANGE=65535
        CHAN_NOCHANGE=65535
        
        msg = OverrideRCIn()
        for i in range(18):
            msg.channels[i] = 65535 # this way we respect the non-motor pwm channels

        msg.channels[0] = left
        msg.channels[1] = right

        self._raw_pwm_pub.publish(msg)

    def odom_rel_cb(self, msg: Odometry):
        self.last_odom = msg

    def gps_pos_cb(self, msg):
        self.gps_pos = msg

    def set_mode(self):
        req = SetMode.Request()
        req.custom_mode = "MANUAL"

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

            self.create_timer(1/20, self.goto_gps_pos)
        except Exception as e:
            self.get_logger().error(f"Arming call failed: {e}")

    def goto_gps_pos(self):
        self.send_pwm()

def main(args=None):
    rclpy.init(args=args)
    node = MavrosStartupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
