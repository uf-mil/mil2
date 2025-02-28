import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from my_joy_listener.dc_motor_driver import DCMotor

class JoySubscriber(Node):
    def __init__(self, L1, L2, L3, R1, R2, R3):
        # motors for later
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.R1 = R1
        self.R2 = R2
        self.R3 = R3

        # ros stuff
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.subscription

    def joy_callback(self, msg):
        # The Joy message contains axes and buttons info from the joystick
        x_axis = msg.axes[0]
        y_axis = msg.axes[1]
        # self.get_logger().info(f"x axis: {x_axis}")
        # self.get_logger().info(f"y axis: {y_axis}")

        # if both are zero stay breaking
        if x_axis == 0.0 and y_axis == 0.0:
            print("break")
            self.L1.force_break()
            self.L2.force_break()
            self.L3.force_break()
            self.R1.force_break()
            self.R2.force_break()
            self.R3.force_break()
            return

        # this code is untested and lowk depends on my memory lol gl
        # assume x is left and right
        # assume y is up and dowm
        # assume +y is up
        # assume +x is left
        #
        # THE IF STATEMENTS TO FOLLOW ARE STRAIGHT ASS
        # ill fix them later with a list or something

        # turn left or right
        if abs(x_axis) > abs(y_axis):
            # turn left with x_axis speed
            if x_axis > 0:
                print("turn left")
                self.L1.set_speed(x_axis)
                self.L2.set_speed(x_axis)
                self.L3.set_speed(x_axis)
                self.R1.set_speed(x_axis)
                self.R2.set_speed(x_axis)
                self.R3.set_speed(x_axis)
                self.L1.go_forward()
                self.L2.go_forward()
                self.L3.go_forward()
                self.R1.go_backwards()
                self.R2.go_backwards()
                self.R3.go_backwards()
            # turn left with x_axis speed
            else:
                print("turn right")
                self.L1.set_speed(-x_axis)
                self.L2.set_speed(-x_axis)
                self.L3.set_speed(-x_axis)
                self.R1.set_speed(-x_axis)
                self.R2.set_speed(-x_axis)
                self.R3.set_speed(-x_axis)
                self.L1.go_backwards()
                self.L2.go_backwards()
                self.L3.go_backwards()
                self.R1.go_forward()
                self.R2.go_forward()
                self.R3.go_forward()
        # go forwards or backards
        else:
            # move forwards with y_axis speed
            if y_axis > 0:
                print("go fowards")
                self.L1.set_speed(y_axis)
                self.L2.set_speed(y_axis)
                self.L3.set_speed(y_axis)
                self.R1.set_speed(y_axis)
                self.R2.set_speed(y_axis)
                self.R3.set_speed(y_axis)
                self.L1.go_forward()
                self.L2.go_forward()
                self.L3.go_forward()
                self.R1.go_forward()
                self.R2.go_forward()
                self.R3.go_forward()
            # move backwards with -y_axis speed (speed must be positve)
            else:
                print("go back")
                self.L1.set_speed(-y_axis)
                self.L2.set_speed(-y_axis)
                self.L3.set_speed(-y_axis)
                self.R1.set_speed(-y_axis)
                self.R2.set_speed(-y_axis)
                self.R3.set_speed(-y_axis)
                self.L1.go_backwards()
                self.L2.go_backwards()
                self.L3.go_backwards()
                self.R1.go_backwards()
                self.R2.go_backwards()
                self.R3.go_backwards()

def main(args=None):
    # which GPIO pins and PWM channels correspond to each wheel
    '''
    Legend: (channel number, gpio_one, gpio_two)
        TOP-DOWN VIEW OF ROVER
        +-------------------+
        |                   |
     L1 | (0, 5, 6)         | R1 (3, 4, 17)
        |                   |
     L2 | (1, 23, 24)       | R2 (4, 27, 22)
        |                   |
     L3 | (2, 12, 13)       | R3 (5, 25, 16)
        +-------------------+
        HINGES ON THIS SIDE
    '''

    L1 = DCMotor(0, 5, 6)
    L2 = DCMotor(1, 23, 24)
    L3 = DCMotor(2, 12, 13)

    R1 = DCMotor(3, 4, 17)
    R2 = DCMotor(4, 27, 22)
    R3 = DCMotor(5, 25, 16)

    # ros2 stuff (setup node start node clenup node)
    rclpy.init(args=args)
    joy_subscriber = JoySubscriber(L1, L2, L3, R1, R2, R3)
    rclpy.spin(joy_subscriber)
    joy_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

