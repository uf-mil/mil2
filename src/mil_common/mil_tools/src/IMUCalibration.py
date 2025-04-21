import datetime
import sys
from time import sleep

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

# ANSI codes for colored output
RED_START = "\033[1;91m"
GREEN_START = "\033[1;32m"
COLOR_END = "\033[0m"


class IMUSubscriber(Node):

    def __init__(self):

        super().__init__("imu_subscriber")

        # Linear acceleration reference values
        self.x_linear_test = [9.8, 0, 0]
        self.y_linear_test = [0, 9.8, 0]
        self.z_linear_test = [0, 0, 9.8]

        # Magnetometer reference values
        self.x_mag_test = [4.5e-5, 0, 0]
        self.y_mag_test = [0, 4.5e-5, 0]
        self.z_mag_test = [0, 0, 4.5e-5]

        # Time that the robot has to stay in one position before the test is successful/passes
        self.timer_threshold = datetime.timedelta(seconds=5)
        self.test_countdown = False
        self.start_time = 0

        # Index of which IMU test we're on
        self.test_counter = 0

        # Averages of the linear acceleration components
        self.averages = [0.0, 0.0, 0.0]

        # Deviances
        self.deviances = [0.0, 0.0, 0.0]

        # Threshold for deviance to pass a test. Making this smaller means
        # you'll have to meet tighter tolerances to pass these tests.
        self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD = 1

        self.MAGNETOMETER_DEVIANCE_THRESHOLD = 1e-5

        if self.test_counter < 3:
            # Subscribes to the IMU topic for accelerometer data
            self.subscription = self.create_subscription(
                Imu,
                "/imu/data",
                self.listener_callback,
                10,
            )

        else:
            self.subscription = self.create_subscription(
                MagneticField,
                "/imu/mag",
                self.listener_callback,
                10,
            )

    # Looks at the deviances and suggests the next action a user should take to calibrate
    def print_suggestion(self):

        # If the test_counter is between 0 and 2, that means we're running the linear acceleration based tests
        if self.test_counter <= 2:

            suggestion = ""

            # If any of the deviances are in the 18-20 range, that means the user is on the right track, except they've
            # flipped the sub 180 degrees from what it should be
            for deviance in self.deviances:

                if deviance >= 18:
                    suggestion = "\n\nTry rotating the sub 180 degrees in the "

                elif deviance >= 8:
                    suggestion = "\n\nTry rotating the sub 90 degrees in the "

                if deviance >= 8:
                    match (self.test_counter):

                        case 0:
                            suggestion += "x-axis\n"

                        case 1:
                            suggestion += "y-axis\n"

                        case 2:
                            suggestion += "z-axis\n"

                        case _:
                            pass

            sys.stdout.write(suggestion)

    def print_deviances(self):

        # Since I'm incorporating color codes to indicate if a deviance is within the threshold or not, I'll make it a string
        # that I can append text to so that I can reduce the number of if-statements I need

        formatted_deviance_string = ""
        deviance_to_compare = 0

        # For the linear acceleration tests
        if self.test_counter < 3:
            deviance_to_compare = self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD

        else:
            deviance_to_compare = self.MAGNETOMETER_DEVIANCE_THRESHOLD

        if all(deviance <= deviance_to_compare for deviance in self.deviances):
            sys.stdout.write(
                f"{GREEN_START}Deviance:{COLOR_END}\t{self.deviances[0]} \t{self.deviances[1]} \t{self.deviances[2]}\n",
            )

        else:

            formatted_deviance_string += f"{RED_START}Deviance:{COLOR_END}\t"

            if self.deviances[0] > deviance_to_compare:
                formatted_deviance_string += (
                    f"{RED_START}{self.deviances[0]}{COLOR_END} \t"
                )
            else:
                formatted_deviance_string += f"{self.deviances[0]} \t"

            if self.deviances[1] > deviance_to_compare:
                formatted_deviance_string += (
                    f"{RED_START}{self.deviances[1]}{COLOR_END} \t"
                )
            else:
                formatted_deviance_string += f"{self.deviances[1]} \t"

            if self.deviances[2] > deviance_to_compare:
                formatted_deviance_string += (
                    f"{RED_START}{self.deviances[2]}{COLOR_END} \t"
                )
            else:
                formatted_deviance_string += f"{self.deviances[2]} \t"

            sys.stdout.write(formatted_deviance_string)

    def listener_callback(self, msg):

        # Rounded linear acceleration values
        if self.test_counter < 3:
            x = round(msg.linear_acceleration.x, 2)
            y = round(msg.linear_acceleration.y, 2)
            z = round(msg.linear_acceleration.z, 2)

        else:
            x = round(msg.magnetic_field.x, 6)
            y = round(msg.magnetic_field.y, 6)
            z = round(msg.magnetic_field.z, 6)

        # # Update the averages (not the best technique since they're both weight equally, but it's the simplest)
        self.averages[0] = (self.averages[0] + x) / 2
        self.averages[1] = (self.averages[1] + y) / 2
        self.averages[2] = (self.averages[2] + z) / 2

        match (self.test_counter):

            # (Linear Acceleration Test) Point X-axis up
            case 0:
                self.deviances[0] = round(
                    abs(self.averages[0] - self.x_linear_test[0]),
                    2,
                )
                self.deviances[1] = round(
                    abs(self.averages[1] - self.x_linear_test[1]),
                    2,
                )
                self.deviances[2] = round(
                    abs(self.averages[2] - self.x_linear_test[2]),
                    2,
                )

                # Overwrite the same line instead of stacking a bunch of messages in the terminal
                sys.stdout.write("\033[2J\033[H")
                sys.stdout.write(
                    f"(Test 1) Orient X-Axis of IMU Up, achieve +/- {self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD} deviance to continue tests\n",
                )
                sys.stdout.write(
                    f"Expected: \t{self.x_linear_test[0]} \t{self.x_linear_test[1]} \t{self.x_linear_test[2]}\n",
                )
                sys.stdout.write(f"\rActual: \t{x} \t{y} \t{z}\n")

                self.print_deviances()

                self.print_suggestion()

                # If all of the deviances are within the threshold, then we'll start/update the timer telling the user
                # how much longer to maintain the position
                if all(
                    deviance <= self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD
                    for deviance in self.deviances
                ):

                    # If the countdown is running, we'll check if we've met the time threshold
                    if self.test_countdown:
                        current_time = datetime.datetime.now()

                        if current_time - self.start_time >= self.timer_threshold:

                            print(
                                "\nSufficient deviance met, continuing to next test...\n",
                            )

                            sleep(2)
                            self.test_counter += 1

                        else:
                            sys.stdout.write(
                                f"\nMaintain position for {round((self.timer_threshold - (current_time - self.start_time)).total_seconds(), 1)} more seconds...",
                            )

                    else:
                        self.test_countdown = True

                        # Record the starting time so we know if the timer threshold is met next time we check
                        self.start_time = datetime.datetime.now()

                # If the current linear acceleration values are not within the deviance, then we need to set test_countdown to false
                else:
                    self.test_countdown = False

                sys.stdout.flush()

            # (Linear Acceleration Test) Point Y-axis up
            case 1:
                self.deviances[0] = round(
                    abs(self.averages[0] - self.y_linear_test[0]),
                    2,
                )
                self.deviances[1] = round(
                    abs(self.averages[1] - self.y_linear_test[1]),
                    2,
                )
                self.deviances[2] = round(
                    abs(self.averages[2] - self.y_linear_test[2]),
                    2,
                )

                # Overwrite the same line instead of stacking a bunch of messages in the terminal
                sys.stdout.write("\033[2J\033[H")
                sys.stdout.write(
                    f"(Test 2) Orient Y-Axis of IMU Up, achieve +/- {self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD} deviance to continue tests\n",
                )
                sys.stdout.write(
                    f"Expected: \t{self.y_linear_test[0]} \t{self.y_linear_test[1]} \t{self.y_linear_test[2]}\n",
                )
                sys.stdout.write(f"\rActual: \t{x} \t{y} \t{z}\n")

                self.print_deviances()

                self.print_suggestion()

                sys.stdout.flush()

                if all(
                    deviance <= self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD
                    for deviance in self.deviances
                ):

                    # If the countdown is running, we'll check if we've met the time threshold
                    if self.test_countdown:
                        current_time = datetime.datetime.now()

                        if current_time - self.start_time >= self.timer_threshold:

                            print(
                                "\nSufficient deviance met, continuing to next test...\n",
                            )

                            sleep(2)
                            self.test_counter += 1

                        else:
                            sys.stdout.write(
                                f"\nMaintain position for {round((self.timer_threshold - (current_time - self.start_time)).total_seconds(), 1)} more seconds...",
                            )

                    else:
                        self.test_countdown = True

                        # Record the starting time so we know if the timer threshold is met next time we check
                        self.start_time = datetime.datetime.now()

                # If the current linear acceleration values are not within the deviance, then we need to set test_countdown to false
                else:

                    self.test_countdown = False

                sys.stdout.flush()

            # (Linear Acceleration Test) Point Z-axis up
            case 2:

                self.deviances[0] = round(
                    abs(self.averages[0] - self.z_linear_test[0]),
                    2,
                )
                self.deviances[1] = round(
                    abs(self.averages[1] - self.z_linear_test[1]),
                    2,
                )
                self.deviances[2] = round(
                    abs(self.averages[2] - self.z_linear_test[2]),
                    2,
                )

                # Overwrite the same line instead of stacking a bunch of messages in the terminal
                sys.stdout.write("\033[2J\033[H")
                sys.stdout.write(
                    f"(Test 3) Orient Z-Axis of IMU Up, achieve +/- {self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD} deviance to continue tests\n",
                )
                sys.stdout.write(
                    f"Expected: \t{self.z_linear_test[0]} \t{self.z_linear_test[1]} \t{self.z_linear_test[2]}\n",
                )
                sys.stdout.write(f"\rActual: \t{x} \t{y} \t{z}\n")

                self.print_deviances()

                self.print_suggestion()

                sys.stdout.flush()

                if all(
                    deviance <= self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD
                    for deviance in self.deviances
                ):

                    # If the countdown is running, we'll check if we've met the time threshold
                    if self.test_countdown:
                        current_time = datetime.datetime.now()

                        if (current_time - self.start_time) >= self.timer_threshold:

                            print(
                                "\nSufficient deviance met, continuing to next test...\n",
                            )

                            # Kill the accelerometer subscriber so we can create a new subscriber for magnetometer
                            self.destroy_subscription(self.subscription)

                            self.subscription = self.create_subscription(
                                MagneticField,
                                "/imu/mag",
                                self.listener_callback,
                                10,
                            )

                            sleep(2)
                            self.test_counter += 1

                        else:
                            sys.stdout.write(
                                f"\nMaintain position for {round((self.timer_threshold - (current_time - self.start_time)).total_seconds(), 1)} more seconds...",
                            )

                    else:
                        self.test_countdown = True

                        # Record the starting time so we know if the timer threshold is met next time we check
                        self.start_time = datetime.datetime.now()

                # If the current linear acceleration values are not within the deviance, then we need to set test_countdown to false
                else:

                    self.test_countdown = False

                sys.stdout.flush()

            # (Magnetometer Test) x-axis
            case 3:

                self.deviances[0] = round(
                    abs(self.averages[0] - self.x_mag_test[0]),
                    6,
                )
                self.deviances[1] = round(
                    abs(self.averages[1] - self.x_mag_test[1]),
                    6,
                )
                self.deviances[2] = round(
                    abs(self.averages[2] - self.x_mag_test[2]),
                    6,
                )

                # Overwrite the same line instead of stacking a bunch of messages in the terminal
                sys.stdout.write("\033[2J\033[H")
                sys.stdout.write(
                    f"(Test 4) Orient X-Axis of IMU towards Magnetic North, achieve +/- {self.MAGNETOMETER_DEVIANCE_THRESHOLD} deviance to continue tests\n",
                )
                sys.stdout.write(
                    f"Expected: \t{self.x_mag_test[0]:.1e} \t{self.x_mag_test[1]:.1e} \t{self.x_mag_test[2]:.1e}\n",
                )

                sys.stdout.write(f"\rActual: \t{x:.1e} \t{y:.1e} \t{z:.1e}\n")

                self.print_deviances()
                self.print_suggestion()

                sys.stdout.flush()

                if all(
                    deviance <= self.MAGNETOMETER_DEVIANCE_THRESHOLD
                    for deviance in self.deviances
                ):

                    # If the countdown is running, we'll check if we've met the time threshold
                    if self.test_countdown:
                        current_time = datetime.datetime.now()

                        if current_time - self.start_time >= self.timer_threshold:

                            print(
                                "\nSufficient deviance met, continuing to next test...\n",
                            )

                            sleep(2)
                            self.test_counter += 1

                        else:
                            sys.stdout.write(
                                f"\nMaintain position for {round((self.timer_threshold - (current_time - self.start_time)).total_seconds(), 1)} more seconds...",
                            )

                    else:
                        self.test_countdown = True

                        # Record the starting time so we know if the timer threshold is met next time we check
                        self.start_time = datetime.datetime.now()

                # If the current magnetometer values are not within the deviance, then we need to set test_countdown to false
                else:

                    self.test_countdown = False

                sys.stdout.flush()

            # (Magnetometer Test) y-axis
            case 4:

                self.deviances[0] = round(
                    abs(self.averages[0] - self.y_mag_test[0]),
                    6,
                )
                self.deviances[1] = round(
                    abs(self.averages[1] - self.y_mag_test[1]),
                    6,
                )
                self.deviances[2] = round(
                    abs(self.averages[2] - self.y_mag_test[2]),
                    6,
                )

                # Overwrite the same line instead of stacking a bunch of messages in the terminal
                sys.stdout.write("\033[2J\033[H")
                sys.stdout.write(
                    f"(Test 5) Orient Y-Axis of IMU towards Magnetic North, achieve +/- {self.MAGNETOMETER_DEVIANCE_THRESHOLD} deviance to continue tests\n",
                )
                sys.stdout.write(
                    f"Expected: \t{self.y_mag_test[0]:.1e} \t{self.y_mag_test[1]:.1e} \t{self.y_mag_test[2]:.1e}\n",
                )

                sys.stdout.write(f"\rActual: \t{x:.1e} \t{y:.1e} \t{z:.1e}\n")

                self.print_deviances()
                self.print_suggestion()

                sys.stdout.flush()

                if all(
                    deviance <= self.MAGNETOMETER_DEVIANCE_THRESHOLD
                    for deviance in self.deviances
                ):

                    # If the countdown is running, we'll check if we've met the time threshold
                    if self.test_countdown:
                        current_time = datetime.datetime.now()

                        if current_time - self.start_time >= self.timer_threshold:

                            print(
                                "\nSufficient deviance met, continuing to next test...\n",
                            )

                            sleep(2)
                            self.test_counter += 1

                        else:
                            sys.stdout.write(
                                f"\nMaintain position for {round((self.timer_threshold - (current_time - self.start_time)).total_seconds(), 1)} more seconds...",
                            )

                    else:
                        self.test_countdown = True

                        # Record the starting time so we know if the timer threshold is met next time we check
                        self.start_time = datetime.datetime.now()

                # If the current magnetometer values are not within the deviance, then we need to set test_countdown to false
                else:

                    self.test_countdown = False

                sys.stdout.flush()

            # # (Magnetometer Test) z-axis
            case 5:

                self.deviances[0] = round(
                    abs(self.averages[0] - self.z_mag_test[0]),
                    6,
                )
                self.deviances[1] = round(
                    abs(self.averages[1] - self.z_mag_test[1]),
                    6,
                )
                self.deviances[2] = round(
                    abs(self.averages[2] - self.z_mag_test[2]),
                    6,
                )

                # Overwrite the same line instead of stacking a bunch of messages in the terminal
                sys.stdout.write("\033[2J\033[H")
                sys.stdout.write(
                    f"(Test 6) Orient Z-Axis of IMU towards Magnetic North, achieve +/- {self.MAGNETOMETER_DEVIANCE_THRESHOLD} deviance to continue tests\n",
                )
                sys.stdout.write(
                    f"Expected: \t{self.z_mag_test[0]:.1e} \t{self.z_mag_test[1]:.1e} \t{self.z_mag_test[2]:.1e}\n",
                )

                sys.stdout.write(f"\rActual: \t{x:.1e} \t{y:.1e} \t{z:.1e}\n")

                self.print_deviances()
                self.print_suggestion()

                sys.stdout.flush()

                if all(
                    deviance <= self.MAGNETOMETER_DEVIANCE_THRESHOLD
                    for deviance in self.deviances
                ):

                    # If the countdown is running, we'll check if we've met the time threshold
                    if self.test_countdown:
                        current_time = datetime.datetime.now()

                        if current_time - self.start_time >= self.timer_threshold:

                            print(
                                "\nSufficient deviance met, all done (:\n",
                            )

                            sleep(2)
                            self.test_counter += 1

                        else:
                            sys.stdout.write(
                                f"\nMaintain position for {round((self.timer_threshold - (current_time - self.start_time)).total_seconds(), 1)} more seconds...",
                            )

                    else:
                        self.test_countdown = True

                        # Record the starting time so we know if the timer threshold is met next time we check
                        self.start_time = datetime.datetime.now()

                # If the current magnetometer values are not within the deviance, then we need to set test_countdown to false
                else:

                    self.test_countdown = False

                sys.stdout.flush()

            case _:
                sys.exit(0)

        # Different pieces of data we can access from the topic
        # .orientation (Quaternion)
        # .oritentation_covariance (array)
        # angular_velocity (Vector3)
        # angular_velocity_covariance (array)
        # linear_acceleration (Vector3)
        # linear_acceleration_covariance (array)

        sleep(0.05)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = IMUSubscriber()

    # Keep the node alive/listening
    rclpy.spin(minimal_subscriber)

    print("Reaches this point")


if __name__ == "__main__":
    main()
