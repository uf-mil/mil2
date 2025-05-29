import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import String
import numpy as np
from nav_msgs.msg import Odometry

class WrenchTuner(Node):

    def __init__(self):
        super().__init__('wrench_tuner')
        self.cmd_subscription = self.create_subscription(
            Wrench,
            'cmd_wrench',
            self.listener_callback,
            10)
        self.cmd_subscription  # prevent unused variable warning

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odometry_callback,
            10)
        
        self.control_wrench_publisher = self.create_publisher(
            Wrench,
            'control_wrench',
            10)
        self.control_wrench_publisher # prevent unused variable warning
        
        self.declare_parameter('c1', 0.0)
        self.declare_parameter('c2', 0.0)
        self.declare_parameter('c3', 0.0)
        self.declare_parameter('c4', 0.0)
        self.declare_parameter('c5', 0.0)
        self.declare_parameter('c6', 0.0)

        self.params = [self.get_parameter('c1').value,
                       self.get_parameter('c2').value,
                       self.get_parameter('c3').value,
                       self.get_parameter('c4').value,
                       self.get_parameter('c5').value,
                       self.get_parameter('c6').value]
        
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.velocity = np.zeros(3)

    def listener_callback(self, msg):
        self.cmd_wrench = np.array([msg.force.x, 
                            msg.force.y,
                            msg.force.z,
                            msg.torque.x,
                            msg.torque.y,
                            msg.torque.z])
        
        self.drag_wrench = np.array([self.params[0] * self.velocity[0],
                                     self.params[1] * self.velocity[1],
                                     self.params[2] * self.velocity[2],
                                     self.params[3] * self.velocity[0],
                                     self.params[4] * self.velocity[1],
                                     self.params[5] * self.velocity[2]])
        
        self.sum_wrench = self.cmd_wrench + self.drag_wrench

        control_wrench = Wrench()
        control_wrench.force.x = self.sum_wrench[0]
        control_wrench.force.y = self.sum_wrench[1]
        control_wrench.force.z = self.sum_wrench[2]
        control_wrench.torque.x = self.sum_wrench[3]
        control_wrench.torque.y = self.sum_wrench[4]
        control_wrench.torque.z = self.sum_wrench[5]
                                     
        self.control_wrench_publisher.publish(control_wrench)
        
    def odometry_callback(self, msg):
        #stores the most recent velocity from odom
        self.velocity = np.array([msg.twist.twist.linear.x,
                                  msg.twist.twist.linear.y,
                                  msg.twist.twist.linear.z])
                                  

    def parameter_callback(self, params):
        for param in params:
            for i, name in enumerate(['c1', 'c2', 'c3', 'c4', 'c5', 'c6']):
                if param.name == name:
                    self.params[i] = param.value
                    self.get_logger().info(f"{name} updated to {param.value}")
        return rclpy.parameter.ParameterEventHandler.Result(successful=True)


def main(args=None):
    rclpy.init(args=args)

    wrench_tuner = WrenchTuner()

    rclpy.spin(wrench_tuner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wrench_tuner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()