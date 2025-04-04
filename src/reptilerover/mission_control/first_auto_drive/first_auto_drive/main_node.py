# take the 
# imu
# sonar
# controller and do this
# 1. if the controller gives any about ignore everything else and do that
# 2. if the sonar says 20cm or less, start_turn
# 3. start_turn uses the imu to turn 90 degress and then we go back to step 1

import sys
import datetime

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Float64MultiArray, String
from first_auto_drive.wheels.tank_drive_train import TankDriveTrain

class MainNode(Node):
    def __init__(self):
        # topic data stored here
        self.joy_data = [0, 0] # gonna be [x, y] on the joystick
        self.imu_data = 0 # this will be a float that is like compass angle
        self.sonar_data = [] # distances from each sonar [10, 12, 2] means sonar1 gives 10 cm sonar 2 reads 12 cm etc.
        # ros stuff
        super().__init__('main_node')

        self.publisher = self.create_publisher(String, 'gpio/write', 20)

        self.subscription_1 = self.create_subscription( Joy, 'joy', self.joy_callback, 10)
        self.subscription_2 = self.create_subscription( Imu, 'imu/data', self.imu_callback, 10)
        self.subscription_3 = self.create_subscription( Float64MultiArray, 'sonar/data', self.sonar_callback, 10)
        self.subscription_1
        self.subscription_2
        self.subscription_3

        self.timer = self.create_timer(0.005, self.timer_callback)

        # motors for later
        self.tank_drive_train = TankDriveTrain(lambda s: self.forward_gpio(s))

        # state ish vars
        self.turning_rn = False
        self.turn_started = datetime.datetime.now()
        self.start_angle = -1 # not used :(

    def forward_gpio(self, s):
        msg = String()
        msg.data = s
        # self.get_logger().warn(f"publishing this msg: {s}")
        self.publisher.publish(msg)

    def joy_callback(self, msg):
        # The Joy message contains axes and buttons info from the joystick
        self.joy_data[0] = msg.axes[0]
        self.joy_data[1] = msg.axes[1]

    def sonar_callback(self, msg):
        # just a list of distances detected by sonar
        self.sonar_data = list(msg.data)

    def imu_callback(self, msg):
        ori = msg.orientation
        roll, pitch, yaw = quaternion_to_euler(ori)
        # self.get_logger().warn(str(roll))
        self.imu_data = roll
        

    def controller_to_motors(self):
        x_axis = self.joy_data[0]
        y_axis = self.joy_data[1]

        # TODO del me
        '''
        if abs(x_axis) == 1 or abs(y_axis) == 1:
            self.tank_drive_train.stop()
            sys.exit(1)
            return
        '''

        if abs(x_axis) > abs(y_axis):
            if x_axis > 0:
                # self.get_logger().warn("left")
                self.tank_drive_train.left(x_axis)
            else:
                # self.get_logger().warn("right")
                self.tank_drive_train.right(x_axis)
        else:
            # move forwards with y_axis speed
            if y_axis > 0:
                # self.get_logger().warn("forward")
                self.tank_drive_train.forward(y_axis)
            else:
                # self.get_logger().warn("backward")
                self.tank_drive_train.backward(y_axis)

    '''
    def imu_turn(self):
        if len(self.sonar_data) == 1:
            self.tank_drive_train.left(0.5)

        elif len(self.sonar_data) == 2:
            # if 2 smaller turn left
            if  self.sonar_data[1]< self.sonar_data[0]:
                self.tank_drive_train.left(0.5)
            else:
                self.tank_drive_train.right(0.5)
        else:
            print("bad things")

        start_angle = self.imu_data
        end_angle = self.imu_data
        while abs(end_angle- start_angle) < 90:
            self.get_logger().warn("b4s")
            rclpy.spin_once(self)
            self.get_logger().warn("after spin")
            end_angle = self.imu_data

            self.get_logger().warn("=========================")
            self.get_logger().warn(str(end_angle))
            self.get_logger().warn(str(abs(end_angle-start_angle)))
            self.get_logger().warn("=========================")
    '''

    def timer_callback(self):
        # first, should we just use controller
        controller_being_used = not (self.joy_data[0] == 0 and self.joy_data[1] == 0)
        if controller_being_used:
            # self.get_logger().warn("cont")

            self.controller_to_motors()
            return

        # second, are we turning?
        # if so, just keep track of delta angle
        # delta_angle = abs(self.start_angle - self.imu_data)
        if self.turning_rn:
            time_spent_turning = (datetime.datetime.now() - self.turn_started).total_seconds()
            if time_spent_turning > 1:
                self.turning_rn = False
                # self.get_logger().warn("leaving turn")
            return

        # final check, is there something in front of us
        sonar_thats_too_close = -1 # could use this to determine which sonar is firing
        if not self.turning_rn:
            for i, reading in enumerate(self.sonar_data):
                # self.get_logger().warn(str(reading))
                if reading>16 and reading <65 and reading != -1.0:
                    self.tank_drive_train.left(0.4)

                    self.turning_rn = True
                    self.start_angle = self.imu_data
                    self.turn_started = datetime.datetime.now()
                    return

        # just keep going forwards
        # self.get_logger().warn("forward")
        # self.tank_drive_train.stop()
        self.tank_drive_train.forward(0.4)

def quaternion_to_euler(quaternion):
    # Convert quaternion to Euler angles (roll, pitch, yaw) in radians

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
    cosr_cosp = 1 - 2 * (quaternion.x ** 2 + quaternion.y ** 2)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
    pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1 - 2 * (quaternion.y ** 2 + quaternion.z ** 2)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)

    main_node = MainNode()

    rclpy.spin(main_node)

    main_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

