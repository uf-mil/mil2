from movement_publisher import SubjugatorControl
import rclpy

rclpy.init()
control = SubjugatorControl()

action = [10, 4]

while True:
    control.force_action(action)
