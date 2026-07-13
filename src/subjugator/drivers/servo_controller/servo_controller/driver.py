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
        # full range of dropper servo seems to be 488-150
        # should be 150 to load 1st, 200 to load 2nd, 255 for holding, 285 to drop 1st, 350 to drop 2nd
        if request.angle == -2:
            dev.set_pwm(0, 150)
        elif request.angle == -1:
            dev.set_pwm(0, 200)
        elif request.angle == 0:
            dev.set_pwm(0, 255)
        elif request.angle == 1:
            dev.set_pwm(0, 285)
            time.sleep(1)
        elif request.angle == 2:
            dev.set_pwm(0, 350)
            time.sleep(1)
            dev.set_pwm(0, 255)
            time.sleep(1)
        return response

    def gripper_callback(self, request: Servo, response):
        # full servo range is 489-149, note other servo may be different
        # should be 318 to screw the worm gear on :)
        # gripper range is 318(open) 210(close)
        dev.set_pwm(7, request.angle)
        if request.angle == 1:
            dev.set_pwm(7, 380)
        elif request.angle == 2:
            dev.set_pwm(7, 240)
        # else:
        #     dev.set_pwm(7, request.angle)
        time.sleep(0.5)

        return response

    def torpedo_callback(self, request: Servo, response):
        dev.set_pwm(4, request.angle)
        if request.angle == 1:
            dev.set_pwm(4, 270)
            time.sleep(1)
            dev.set_pwm(4, 300)
        elif request.angle == 2:
            dev.set_pwm(4, 350)
            time.sleep(1)
            dev.set_pwm(4, 300)
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
