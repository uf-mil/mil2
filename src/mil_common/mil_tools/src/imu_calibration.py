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
CLEAR_CONSOLE = "\033[2J\033[H"


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

        # The first 3 tests are for linear acceleration, so if test_counter is < 3, then we'll subscribe
        # to the /imu/data topic
        if self.test_counter < 3:
            self.subscription = self.create_subscription(
                Imu,
                "/imu/data",
                self.listener_callback,
                10,
            )

        # Otherwise if test counter is > 3, then we're running the magnetometer tests and can subscribe to /imu/mag
        else:
            self.subscription = self.create_subscription(
                MagneticField,
                "/imu/mag",
                self.listener_callback,
                10,
            )

    # Looks at the deviances and suggests the next action a user should take to calibrate
    def print_suggestion(self):
        """
        For the linear acceleration/acceleromter tests, we can give a general suggestion to the user on which way to
        turn or reorient the sub to meet the threshold requirements.

        We can generate that suggestion by looking at the deviance values and what ranges they fall in.
        """

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
        """
        Every test has to print the deviance values, so it's in a function to remove duplicate code. It also contains
        some checks to help with formatting since magnetometer data is printed differently than accelerometer data.
        """

        # Since I'm incorporating color codes to indicate if a deviance is within the threshold or not, I'll make it a string
        # that I can append text to so that I can reduce the number of if-statements I need

        formatted_deviance_string = ""
        deviance_to_compare = 0

        # For the linear acceleration tests
        if self.test_counter < 3:
            deviance_to_compare = self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD

        # For the magnetometer tests
        else:
            deviance_to_compare = self.MAGNETOMETER_DEVIANCE_THRESHOLD

        # The magnetometer values can make the formatting look messy if they're not all extended to the correct
        # decimal place (in the case of whole number values), so I need to do some checks to reinforce formatting
        # at the cost of some efficiency and copying
        deviances_to_print = self.deviances

        if self.test_counter > 2:

            # Enforces the decimal to be to one place (EX: 1e-5 becomes 1.0e-5)
            deviances_to_print = [f"{x:.1e}" for x in self.deviances]

        # If all of the IMU deviances are within the thresholds, then we'll use a green text to indicate we're good
        if all(deviance <= deviance_to_compare for deviance in self.deviances):
            sys.stdout.write(
                f"{GREEN_START}Deviance:{COLOR_END}\t{self.deviances[0]} \t{self.deviances[1]} \t{self.deviances[2]}\n",
            )

        # Otherwise, we'll use red text to indicate some or all of the deviances are not within the thresholds
        else:

            formatted_deviance_string += f"{RED_START}Deviance:{COLOR_END}\t"

            if self.deviances[0] > deviance_to_compare:
                formatted_deviance_string += (
                    f"{RED_START}{deviances_to_print[0]}{COLOR_END} \t"
                )
            else:
                formatted_deviance_string += f"{deviances_to_print[0]} \t"

            if self.deviances[1] > deviance_to_compare:
                formatted_deviance_string += (
                    f"{RED_START}{deviances_to_print[1]}{COLOR_END} \t"
                )
            else:
                formatted_deviance_string += f"{deviances_to_print[1]} \t"

            if self.deviances[2] > deviance_to_compare:
                formatted_deviance_string += (
                    f"{RED_START}{deviances_to_print[2]}{COLOR_END} \t"
                )
            else:
                formatted_deviance_string += f"{deviances_to_print[2]} \t"

            sys.stdout.write(formatted_deviance_string)

    def update_timer_threshold(self):
        """
        Checks if the current IMU readings are within the deviance threshold. If so, then the program lets the timer
        run down. If the readings stay within the deviance threshold by the time the counter reaches 0, then we proceed
        to the next test, otherwise we reset the timer.

        Returns a bool indicating whether a test succeeded (True, meaning the timer went all the way to 0 without deviances going
        out of bounds), or of the test is still in progress (False).
        """

        deviance_threshold = self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD

        # If test counter is above 2, that means we're on the magnetometer tests, so we'll need to update the
        # deviance_threshold value
        if self.test_counter > 2:
            deviance_threshold = self.MAGNETOMETER_DEVIANCE_THRESHOLD

        # If all of the deviances have been met, then we can check if we've passed the threshold timer yet
        if all(deviance <= deviance_threshold for deviance in self.deviances):

            # If the countdown timer is running for this test, we'll check if we've met the time threshold
            if self.test_countdown:
                current_time = datetime.datetime.now()

                if current_time - self.start_time >= self.timer_threshold:

                    print(
                        "\nSufficient deviance met, continuing to next test...\n",
                    )

                    sleep(2)
                    self.test_counter += 1

                    # Return True to indicate that the test passed (in case I need to update the subscriber to a new topic)
                    return True

                else:
                    sys.stdout.write(
                        f"\nMaintain position for {round((self.timer_threshold - (current_time - self.start_time)).total_seconds(), 1)} more seconds...",
                    )

            else:

                # Record the starting time so we know if the timer threshold is met next time we check
                self.start_time = datetime.datetime.now()

                self.test_countdown = True

        # If the current IMU values are not within the deviance, then we need to set test_countdown to false
        else:
            self.test_countdown = False

        # Indicates test hasn't passed (yet)
        return False

    def listener_callback(self, msg):
        """
        Callback function that executes every time a message is received from the ROS 2 topic subscriber.

        It checks if the average of the past + current data is within the deviance thresholds for the IMU test
        that's being ran and checks if the threshold timer is met for each test.

        If so, then the program moves onto the next text.

        Currently the function has implementations for the linear acceleration and magnetometer data.
        """

        # Rounded linear acceleration values
        if self.test_counter < 3:
            x = round(msg.linear_acceleration.x, 2)
            y = round(msg.linear_acceleration.y, 2)
            z = round(msg.linear_acceleration.z, 2)

        # We'll use higher precision for the magnetometer values since they're so small
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

                # Update deviance values
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
                sys.stdout.write(f"{CLEAR_CONSOLE}")

                sys.stdout.write(
                    f"(Test 1) Orient X-Axis of IMU Up, achieve +/- {self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD} deviance to continue tests\n",
                )

                sys.stdout.write(
                    f"Expected: \t{self.x_linear_test[0]} \t{self.x_linear_test[1]} \t{self.x_linear_test[2]}\n",
                )

                sys.stdout.write(f"\rActual: \t{x} \t{y} \t{z}\n")

                self.print_deviances()
                self.print_suggestion()

                self.update_timer_threshold()

                sys.stdout.flush()

            # (Linear Acceleration Test) Point Y-axis up
            case 1:

                # Update deviance values
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
                sys.stdout.write(f"{CLEAR_CONSOLE}")

                sys.stdout.write(
                    f"(Test 2) Orient Y-Axis of IMU Up, achieve +/- {self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD} deviance to continue tests\n",
                )

                sys.stdout.write(
                    f"Expected: \t{self.y_linear_test[0]} \t{self.y_linear_test[1]} \t{self.y_linear_test[2]}\n",
                )

                sys.stdout.write(f"\rActual: \t{x} \t{y} \t{z}\n")

                self.print_deviances()
                self.print_suggestion()

                self.update_timer_threshold()

                sys.stdout.flush()

            # (Linear Acceleration Test) Point Z-axis up
            case 2:

                # Update deviance values
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
                sys.stdout.write(f"{CLEAR_CONSOLE}")

                sys.stdout.write(
                    f"(Test 3) Orient Z-Axis of IMU Up, achieve +/- {self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD} deviance to continue tests\n",
                )

                sys.stdout.write(
                    f"Expected: \t{self.z_linear_test[0]} \t{self.z_linear_test[1]} \t{self.z_linear_test[2]}\n",
                )

                sys.stdout.write(f"\rActual: \t{x} \t{y} \t{z}\n")

                self.print_deviances()
                self.print_suggestion()

                if self.update_timer_threshold():

                    # If the test passes, then we need to kill the accelerometer subscriber so
                    # we can create a new subscriber for magnetometer
                    self.destroy_subscription(self.subscription)

                    self.subscription = self.create_subscription(
                        MagneticField,
                        "/imu/mag",
                        self.listener_callback,
                        10,
                    )

                sys.stdout.flush()

            # (Magnetometer Test) x-axis
            case 3:

                # Update deviance values
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
                sys.stdout.write(f"{CLEAR_CONSOLE}")

                sys.stdout.write(
                    f"(Test 4) Orient X-Axis of IMU towards Magnetic North, achieve +/- {self.MAGNETOMETER_DEVIANCE_THRESHOLD} deviance to continue tests\n",
                )
                sys.stdout.write(
                    f"Expected: \t{self.x_mag_test[0]:.1e} \t{self.x_mag_test[1]:.1e} \t{self.x_mag_test[2]:.1e}\n",
                )

                sys.stdout.write(f"\rActual: \t{x:.1e} \t{y:.1e} \t{z:.1e}\n")

                self.print_deviances()
                # self.print_suggestion()

                self.update_timer_threshold()

                sys.stdout.flush()

            # (Magnetometer Test) y-axis
            case 4:

                # Update our deviance values
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
                sys.stdout.write(f"{CLEAR_CONSOLE}")

                sys.stdout.write(
                    f"(Test 5) Orient Y-Axis of IMU towards Magnetic North, achieve +/- {self.MAGNETOMETER_DEVIANCE_THRESHOLD} deviance to continue tests\n",
                )

                sys.stdout.write(
                    f"Expected: \t{self.y_mag_test[0]:.1e} \t{self.y_mag_test[1]:.1e} \t{self.y_mag_test[2]:.1e}\n",
                )

                sys.stdout.write(f"\rActual: \t{x:.1e} \t{y:.1e} \t{z:.1e}\n")

                self.print_deviances()
                # self.print_suggestion()

                self.update_timer_threshold()

                sys.stdout.flush()

            # # (Magnetometer Test) z-axis
            case 5:

                # Update deviance values
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
                sys.stdout.write(f"{CLEAR_CONSOLE}")

                sys.stdout.write(
                    f"(Test 6) Orient Z-Axis of IMU towards Magnetic North, achieve +/- {self.MAGNETOMETER_DEVIANCE_THRESHOLD} deviance to continue tests\n",
                )
                sys.stdout.write(
                    f"Expected: \t{self.z_mag_test[0]:.1e} \t{self.z_mag_test[1]:.1e} \t{self.z_mag_test[2]:.1e}\n",
                )

                sys.stdout.write(f"\rActual: \t{x:.1e} \t{y:.1e} \t{z:.1e}\n")

                self.print_deviances()
                # self.print_suggestion()

                self.update_timer_threshold()

                sys.stdout.flush()

            case _:
                print("\nTesting complete\n")
                sys.exit(0)

        # Small sleep to limit the number of data objects we process
        sleep(0.05)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = IMUSubscriber()

    # Keep the node alive/listening
    rclpy.spin(minimal_subscriber)

    print("Reaches this point")


if __name__ == "__main__":
    main()
