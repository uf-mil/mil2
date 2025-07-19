import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from subjugator_msgs.srv import Servo

Dropper_Pin = 15
Gripper_Pin = 33
Torpedo_Pin = 32

GPIO.setmode(GPIO.BOARD)

# setup dropper pwm
GPIO.setup(Dropper_Pin, GPIO.OUT, initial=GPIO.HIGH)
Dropper_PWM = GPIO.PWM(Dropper_Pin, 50)
Dropper_PWM.start(12)

# setup gripper pwm
GPIO.setup(Gripper_Pin, GPIO.OUT, initial=GPIO.HIGH)
Gripper_PWM = GPIO.PWM(Gripper_Pin, 50)
Gripper_PWM.start(0)

# setup torpedo pwm
GPIO.setup(Torpedo_Pin, GPIO.OUT, initial=GPIO.HIGH)
Torpedo_PWM = GPIO.PWM(Torpedo_Pin, 50)
Torpedo_PWM.start(0)

# duty needs to = 12 to allow for load
# duty needs to = 7 for first drop and 4 for second drop


class Servo_Controller(Node):
    def __init__(self):
        super().__init__("servo_controller")
        self.srv1 = self.create_service(Servo, "dropper", self.dropper_callback)
        self.srv2 = self.create_service(Servo, "gripper", self.gripper_callback)
        self.srv3 = self.create_service(Servo, "torpedo", self.torpedo_callback)

    def change_angle(self, x, angle):
        duty = angle / 10
        if x == 1:
            Dropper_PWM.ChangeDutyCycle(duty)
        elif x == 2:
            Gripper_PWM.ChangeDutyCycle(duty)
        elif x == 3:
            Torpedo_PWM.ChangeDutyCycle(duty)

    def dropper_callback(self, request: Servo, response):
        self.change_angle(1, request.angle)
        return response

    def gripper_callback(self, request: Servo, response):
        self.change_angle(2, request.angle)
        return response

    def torpedo_callback(self, request: Servo, response):
        self.change_angle(3, request.angle)
        return response


def main():
    rclpy.init()
    node = Servo_Controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
