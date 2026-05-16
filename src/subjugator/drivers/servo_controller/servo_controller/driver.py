import time

import rclpy
from pca9685_driver import Device
from rclpy.node import Node
from subjugator_msgs.srv import Servo

dev = Device(0x40, bus_number=7)

dev.set_pwm_frequency(50)

# add startup command to hold the bearings in


class Servo_Controller(Node):
    def __init__(self):
        super().__init__("servo_controller")
        self.srv1 = self.create_service(Servo, "dropper", self.dropper_callback)
        self.srv2 = self.create_service(Servo, "gripper", self.gripper_callback)
        self.srv3 = self.create_service(Servo, "torpedo", self.torpedo_callback)

    def dropper_callback(self, request: Servo, response):
        if request.angle == 1:
            dev.set_pwm(0, 310)
            time.sleep(3)
        elif request.angle == 2:
            dev.set_pwm(0, 432)
            time.sleep(3)
            dev.set_pwm(0, 138)
            time.sleep(2)
        else:
            print("Nonvalid input, please input 1 or 2")
        # dev.set_pwm(0, 310)
        # time.sleep(1)
        # dev.set_pwm(0, 432)
        # time.sleep(1)
        # dev.set_pwm(0, 138)
        # time.sleep(1)
        return response

    def gripper_callback(self, request: Servo, response):
        return response

    def torpedo_callback(self, request: Servo, response):
        # dev.set_pwm(4, 450)
        # dev.set_pwm(4, request.angle)
        # time.sleep(1)
        # time.sleep(0.5)
        # dev.set_pwm(4, 300)
        # dev.set_pwm(4, 450)
        # time.sleep(1)
        dev.set_pwm(4, request.angle)
        if request.angle == 1:
            dev.set_pwm(4, 380)
            time.sleep(1)
            dev.set_pwm(4, 330)
        elif request.angle == 2:
            dev.set_pwm(4, 200)
            time.sleep(1)
            dev.set_pwm(4, 330)
        else:
            pass
            # print("Nonvalid input, please input 1 or 2")
        return response


def main():
    rclpy.init()
    node = Servo_Controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
