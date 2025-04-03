import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu

import sys
from time import sleep


class IMUSubscriber(Node):

    def __init__(self):

        super().__init__('imu_subscriber')

        # Reference values for each test

        # Orientation test reference values
        self.x_orientation_test = [9.8, 0, 0]
        self.y_orientation_test = [0, 9.8, 0]
        self.z_orientation_test = [0, 0, 9.8]

        # Index of which IMU test we're on
        self.test_counter = 0

        # Averages of the linear acceleration components
        self.x_average = 0
        self.y_average = 0
        self.z_average = 0

        # Threshold for deviance to pass a test
        self.ORIENTATION_DEVIANCE_THRESHOLD = 0.1

        # Subscribes to the IMU topic
        self.subscription = self.create_subscription(
            Imu,
            "/imu/data_raw",
            self.listener_callback,
            10)
        
        self.subscription

    def listener_callback(self, msg):

        # Rounded linear acceleration values
        x = round(msg.orientation.x, 2)
        y = round(msg.orientation.y, 2)
        z = round(msg.orientation.z, 2)

        # If we just started, we'll set the first values to the first IMU readings we get
        if (self.x_average == 0 and self.y_average == 0 and self.z_average == 0):
            self.x_average = x
            self.y_average = y
            self.z_average = z
        # Update the averages (not the best technique since they're both weight equally, but it's the simplest)
        else:
            self.x_average = (self.x_average + x)/2
            self.y_average = (self.y_average + y)/2
            self.z_average = (self.z_average + z)/2

        match (self.test_counter):

            # (Test) Point X-axis up
            case 0:
            
                # Overwrite the same line instead of stacking a bunch of messages in the terminal
                sys.stdout.write(f"\033[2J\033[H")
                sys.stdout.write("Orient X-Axis of IMU Up, achieve +/- 0.1 deviance to continue tests\n")
                sys.stdout.write(f"Expected: \t{self.x_orientation_test[0]} \t{self.x_orientation_test[1]} \t{self.x_orientation_test[2]}\n")
                sys.stdout.write(f"\rActual: \t{x} \t{y} \t{z}")
                sys.stdout.flush()

                if (abs(self.x_average - self.x_orientation_test[0]) <= self.ORIENTATION_DEVIANCE_THRESHOLD and abs(self.y_average - self.x_orientation_test[1]) <= self.ORIENTATION_DEVIANCE_THRESHOLD and abs(self.z_average - self.x_orientation_test[2]) <= self.ORIENTATION_DEVIANCE_THRESHOLD):
                    print("Sufficient deviance met, continuing to next test...\n")

                    sleep(2)
                    self.test_counter += 1


            # (Test) Point Y-axis up
            case 1:

                # Overwrite the same line instead of stacking a bunch of messages in the terminal
                sys.stdout.write(f"\033[2J\033[H")
                sys.stdout.write("Orient Y-Axis of IMU Up, achieve +/- 0.1 deviance to continue tests\n")
                sys.stdout.write(f"Expected: \t{self.y_orientation_test[0]} \t{self.y_orientation_test[1]} \t{self.y_orientation_test[2]}\n")
                sys.stdout.write(f"\rActual: \t{x} \t{y} \t{z}")
                sys.stdout.flush()

                if (abs(self.x_average - self.y_orientation_test[0]) <= self.ORIENTATION_DEVIANCE_THRESHOLD and abs(self.y_average - self.y_orientation_test[1]) <= self.ORIENTATION_DEVIANCE_THRESHOLD and abs(self.z_average - self.y_orientation_test[2]) <= self.ORIENTATION_DEVIANCE_THRESHOLD):
                    print("Sufficient deviance met, continuing to next test...\n")

                    sleep(2)
                    self.test_counter += 1


            # (Test) Point Z-axis up
            case 1:

                # Overwrite the same line instead of stacking a bunch of messages in the terminal
                sys.stdout.write(f"\033[2J\033[H")
                sys.stdout.write("Orient Z-Axis of IMU Up, achieve +/- 0.1 deviance to continue tests\n")
                sys.stdout.write(f"Expected: \t{self.y_orientation_test[0]} \t{self.y_orientation_test[1]} \t{self.y_orientation_test[2]}\n")
                sys.stdout.write(f"\rActual: \t{x} \t{y} \t{z}")
                sys.stdout.flush()

                if (abs(self.x_average - self.z_orientation_test[0]) <= self.ORIENTATION_DEVIANCE_THRESHOLD and abs(self.y_average - self.z_orientation_test[1]) <= self.ORIENTATION_DEVIANCE_THRESHOLD and abs(self.z_average - self.z_orientation_test[2]) <= self.ORIENTATION_DEVIANCE_THRESHOLD):
                    print("Sufficient deviance met, continuing to next test...\n")

                    sleep(2)
                    self.test_counter += 1


        # Different pieces of data we can access from the topic
        # .orientation (Quaternion)
        # .oritentation_covariance (array)
        # angular_velocity (Vector3)
        # angular_velocity_covariance (array)
        # linear_acceleration (Vector3)
        # linear_acceleration_covariance (array)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = IMUSubscriber()

    # Keep the node alive/listening
    rclpy.spin(minimal_subscriber)



if __name__ == '__main__':
    main()