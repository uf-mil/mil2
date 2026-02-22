from geometry_msgs.msg import Wrench
from rclpy.node import Node
from std_msgs.msg import String


class TeleopNode(Node):
    """
    Node to publish wrenches and which keys are actively being pressed.
    """

    def __init__(self):
        super().__init__("tkinter_teleop")

        self.pub = self.create_publisher(Wrench, "cmd_wrench", 10)
        self.keypress_pub = self.create_publisher(String, "keypress", 10)

        self.force = [0.0, 0.0, 0.0]
        self.torque = [0.0, 0.0, 0.0]

        self.linear = 100.0
        self.angular = 100.0

        self.active_keys = set()

    def update_from_keys(self):
        """
        Update wrench from keys pressed.
        """
        fx = fy = fz = 0.0
        tx = ty = tz = 0.0

        if "w" in self.active_keys:
            fx += self.linear
        if "s" in self.active_keys:
            fx -= self.linear
        if "a" in self.active_keys:
            fy += self.linear
        if "d" in self.active_keys:
            fy -= self.linear
        if "x" in self.active_keys:
            fz += self.linear
        if "z" in self.active_keys:
            fz -= self.linear

        if "Up" in self.active_keys:
            ty -= self.angular
        if "Down" in self.active_keys:
            ty += self.angular
        if "Left" in self.active_keys:
            tz += self.angular
        if "Right" in self.active_keys:
            tz -= self.angular
        if "e" in self.active_keys:
            tx -= self.angular
        if "r" in self.active_keys:
            tx += self.angular

        self.force = [fx, fy, fz]
        self.torque = [tx, ty, tz]

    def publish(self):
        """
        Publish wrenches.
        """
        self.update_from_keys()

        msg = Wrench()
        msg.force.x, msg.force.y, msg.force.z = self.force
        msg.torque.x, msg.torque.y, msg.torque.z = self.torque
        self.pub.publish(msg)

    def spawn_marble(self):
        m = String()
        m.data = "m"
        self.keypress_pub.publish(m)
