import enum
import transforms3d
from rclpy.node import Node
from geometry_msgs.msg import Pose, Wrench

class State(Enum):
    GO_DOWN = 1
    ROLL = 2
    ROLL_STOP = 3

class RollClosed(Node):
    def __init__(self):
        super().__init__("roll_closed")
        self.goal_pub = self.create_publisher(Pose, "/goal_pose", 10)
        self.odom_sub = self.create_subscription(Pose, "/odometry/filtered",
                                                 self.odom_cb, 10)
        self.wrench_pub = self.create_publisher(Wrench, "/add_wrench", 10)

        self.goal = Goal()
        self.goal.pose.x = 0
        self.goal.pose.y = 0
        self.goal.pose.z = -1
        self.goal_pub.publish(self.goal)

        self.state = 0

        self.timer = self.create_timer(1 / 20, self.tick)

    def odom_cb(odom):
        if self.state == GO_DOWN:
            dx = self.odom.pose.pose.position.x - self.goal.x
            dy = self.odom.pose.pose.position.y - self.goal.y
            dz = self.odom.pose.pose.position.z - self.goal.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            if dist < 0.2:
                self.state = ROLL
                self.roll_timeout = self.create_timer(4, self.roll_timeout_cb)
        elif self.state == ROLL:
            qn = odom.pose.pose.orientation
            roll, _, _ = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])

            if self.roll_prev is None:
                self.roll_prev = roll
                self.roll_amt = 0
            else:
                droll = roll - self.roll_prev
                if droll > math.pi or droll < -math.pi:
                    droll = math.pi - droll
                self.roll_amt += math.remainder(, math.tau)
                if roll_amt >= 315 * (math.pi / 180):
                    self.state = ROLL_STOP
                self.roll_prev = roll

    def tick(self):
        if self.state == ROLL:
            wrench = Wrench()
            wrench.torque.x = 10
            self.wrench_pub.publish(wrench)
        else:
            wrench = Wrench()
            self.wrench_pub.publish(wrench)

    def roll_timeout_cb():
        self.roll_timeout.cancel()
        self.state = ROLL_STOP

rclpy.init()
node = RollClosed()
rclpy.spin(node)
rclpy.shutdown()
