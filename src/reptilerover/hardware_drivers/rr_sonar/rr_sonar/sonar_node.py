import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from rr_sonar.sonar_driver import Sonar
import time

from collections import deque

class SonarNode(Node):
    def __init__(self):
        super().__init__('sonar_node')
        self.publisher = self.create_publisher(Float64MultiArray, 'sonar/data', 10)

        self.subscription = self.create_subscription(String, "gpio/write", self.gpio_write_callback, 20)

        self.timer = self.create_timer(0.01, self.timer_cb)

        # here are the lines that determine how many sonar and pin outs
        self.sonars = [
            Sonar(20, 21),
        ]

        self.readings = []
        for sonar in self.sonars:
            self.readings.append(deque(maxlen=12))

    def gpio_write_callback(self, msgs):
        for msg in msgs.data.split('\n'):
            if len(msg) == 0:
                continue

            msg = msg.split()
            pin = int(msg[0])
            level = int(msg[1])
            self.sonars[0].gpio_write(pin, level)

    def avg(self, measurements):
        for i, meas in enumerate(measurements):
            if meas == -1:
                continue
            self.readings[i].append(meas)

        avg = []
        for reading in self.readings:
            if len(reading) == 0:
                avg.append(-1)
                continue

            avg.append(sum(reading)/len(reading))
        return avg

    def timer_cb(self):
        measurements = []

        for sonar in self.sonars:
            meas = sonar.single_measure() # could be -1 on bad read
            measurements.append(meas)
            # time.sleep(2 * 10**-3) # delay to help w noise NOT need when only one sonar

        avg_measurements = self.avg(measurements)

        msg = Float64MultiArray()
        msg.data = avg_measurements
        print(avg_measurements)
        self.publisher.publish(msg)

def main():
    rclpy.init()
    my_node = SonarNode()
    
    rclpy.spin(my_node)

    my_node.destroy_node()
    rclpy.shutdown()


