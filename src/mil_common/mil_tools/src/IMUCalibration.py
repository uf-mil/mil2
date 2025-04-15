import datetime
import sys
from time import sleep

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class IMUSubscriber(Node):

    def __init__(self):

        super().__init__("imu_subscriber")

        # Orientation test reference values
        self.x_axis_test = [9.8, 0, 0]
        self.y_axis_test = [0, 9.8, 0]
        self.z_axis_test = [0, 0, 9.8]

        # Time that the robot has to stay in one position before the test is successful/passes
        self.timer_threshold = datetime.timedelta(seconds=5)
        self.test_countdown = False
        self.start_time = 0

        # Index of which IMU test we're on
        self.test_counter = 0

        # Averages of the linear acceleration components
        self.linear_acceleration_averages = [0.0, 0.0, 0.0]

        # Deviances
        self.linear_acceleration_deviances = [0.0, 0.0, 0.0]

        # Threshold for deviance to pass a test. Making this smaller means
        # you'll have to meet tighter tolerances to pass these tests.
        self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD = 1

        # Subscribes to the IMU topic
        self.subscription = self.create_subscription(
            Imu,
            "/imu/data",
            self.listener_callback,
            10,
        )

        self.subscription

    # Looks at the deviances and suggests the next action a user should take to calibrate
    def generate_suggestion(self):

        # If the test_counter is between 0 and 2, that means we're running the linear acceleration based tests
        if self.test_counter <= 2:

            # If any of the deviances are in the 18-20 range, that means the user is on the right track, except they've
            # flipped the sub 180 degrees from what it should be
            for deviance in self.linear_acceleration_deviances:

                if deviance >= 18:
                    suggestion = "Try rotating the sub 180 degrees in the "

                elif deviance >= 8:
                    suggestion = "Try rotation the sub 90 degrees in the "

                if deviance >= 8:
                    match (self.test_counter):

                        case 0:
                            suggestion += "x-axis\n"

                        case 1:
                            suggestion += "y-axis\n"

                        case 2:
                            suggestion += "z-axis\n"

                        case _:
                            return ""

                    return suggestion

            return ""

        return ""

    def listener_callback(self, msg):

        # Rounded linear acceleration values
        x = round(msg.linear_acceleration.x, 2)
        y = round(msg.linear_acceleration.y, 2)
        z = round(msg.linear_acceleration.z, 2)

        # # Update the averages (not the best technique since they're both weight equally, but it's the simplest)
        self.linear_acceleration_averages[0] = (
            self.linear_acceleration_averages[0] + x
        ) / 2
        self.linear_acceleration_averages[1] = (
            self.linear_acceleration_averages[1] + y
        ) / 2
        self.linear_acceleration_averages[2] = (
            self.linear_acceleration_averages[2] + z
        ) / 2

        suggestion = self.generate_suggestion()

        match (self.test_counter):

            # (Test) Point X-axis up
            case 0:
                self.linear_acceleration_deviances[0] = round(
                    abs(self.linear_acceleration_averages[0] - self.x_axis_test[0]),
                    2,
                )
                self.linear_acceleration_deviances[1] = round(
                    abs(self.linear_acceleration_averages[1] - self.x_axis_test[1]),
                    2,
                )
                self.linear_acceleration_deviances[2] = round(
                    abs(self.linear_acceleration_averages[2] - self.x_axis_test[2]),
                    2,
                )

                # Overwrite the same line instead of stacking a bunch of messages in the terminal
                sys.stdout.write("\033[2J\033[H")
                sys.stdout.write(
                    f"Orient X-Axis of IMU Up, achieve +/- {self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD} deviance to continue tests\n",
                )
                sys.stdout.write(
                    f"Expected: \t{self.x_axis_test[0]} \t{self.x_axis_test[1]} \t{self.x_axis_test[2]}\n",
                )
                sys.stdout.write(f"\rActual: \t{x} \t{y} \t{z}\n")

                # If all of the deviances are within the threshold, then we'll start/update the timer telling the user
                # how much longer to maintain the position
                if all(
                    deviance <= self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD
                    for deviance in self.linear_acceleration_deviances
                ):
                    sys.stdout.write(
                        f"\033[1;32mDeviance:\033[0m \t{self.linear_acceleration_deviances[0]} \t{self.linear_acceleration_deviances[1]} \t{self.linear_acceleration_deviances[2]}\n",
                    )

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
                    sys.stdout.write(
                        f"\033[1;31mDeviance:\033[0m \t{self.linear_acceleration_deviances[0]} \t{self.linear_acceleration_deviances[1]} \t{self.linear_acceleration_deviances[2]}\n",
                    )

                    if len(suggestion) != 0:
                        sys.stdout.write(
                            f"\nSuggestion: {self.generate_suggestion()}\n",
                        )

                    self.test_countdown = False

                sys.stdout.flush()

            # (Test) Point Y-axis up
            case 1:
                self.linear_acceleration_deviances[0] = round(
                    abs(self.linear_acceleration_averages[0] - self.y_axis_test[0]),
                    2,
                )
                self.linear_acceleration_deviances[1] = round(
                    abs(self.linear_acceleration_averages[1] - self.y_axis_test[1]),
                    2,
                )
                self.linear_acceleration_deviances[2] = round(
                    abs(self.linear_acceleration_averages[2] - self.y_axis_test[2]),
                    2,
                )

                # Overwrite the same line instead of stacking a bunch of messages in the terminal
                sys.stdout.write("\033[2J\033[H")
                sys.stdout.write(
                    f"Orient Y-Axis of IMU Up, achieve +/- {self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD} deviance to continue tests\n",
                )
                sys.stdout.write(
                    f"Expected: \t{self.y_axis_test[0]} \t{self.y_axis_test[1]} \t{self.y_axis_test[2]}\n",
                )
                sys.stdout.write(f"\rActual: \t{x} \t{y} \t{z}\n")
                sys.stdout.flush()

                if all(
                    deviance <= self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD
                    for deviance in self.linear_acceleration_deviances
                ):
                    sys.stdout.write(
                        f"\033[1;32mDeviance:\033[0m \t{self.linear_acceleration_deviances[0]} \t{self.linear_acceleration_deviances[1]} \t{self.linear_acceleration_deviances[2]}\n",
                    )

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
                    sys.stdout.write(
                        f"\033[1;31mDeviance:\033[0m \t{self.linear_acceleration_deviances[0]} \t{self.linear_acceleration_deviances[1]} \t{self.linear_acceleration_deviances[2]}\n",
                    )

                    if len(suggestion) != 0:
                        sys.stdout.write(
                            f"\nSuggestion: {self.generate_suggestion()}\n",
                        )

                    self.test_countdown = False

                sys.stdout.flush()

            # (Test) Point Z-axis up
            case 2:

                self.linear_acceleration_deviances[0] = round(
                    abs(self.linear_acceleration_averages[0] - self.z_axis_test[0]),
                    2,
                )
                self.linear_acceleration_deviances[1] = round(
                    abs(self.linear_acceleration_averages[1] - self.z_axis_test[1]),
                    2,
                )
                self.linear_acceleration_deviances[2] = round(
                    abs(self.linear_acceleration_averages[2] - self.z_axis_test[2]),
                    2,
                )

                # Overwrite the same line instead of stacking a bunch of messages in the terminal
                sys.stdout.write("\033[2J\033[H")
                sys.stdout.write(
                    f"Orient Z-Axis of IMU Up, achieve +/- {self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD} deviance to continue tests\n",
                )
                sys.stdout.write(
                    f"Expected: \t{self.z_axis_test[0]} \t{self.z_axis_test[1]} \t{self.z_axis_test[2]}\n",
                )
                sys.stdout.write(f"\rActual: \t{x} \t{y} \t{z}\n")
                sys.stdout.flush()

                if all(
                    deviance <= self.LINEAR_ACCELERATION_DEVIANCE_THRESHOLD
                    for deviance in self.linear_acceleration_deviances
                ):
                    sys.stdout.write(
                        f"\033[1;32mDeviance:\033[0m \t{self.linear_acceleration_deviances[0]} \t{self.linear_acceleration_deviances[1]} \t{self.linear_acceleration_deviances[2]}\n",
                    )

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
                    sys.stdout.write(
                        f"\033[1;31mDeviance:\033[0m \t{self.linear_acceleration_deviances[0]} \t{self.linear_acceleration_deviances[1]} \t{self.linear_acceleration_deviances[2]}\n",
                    )

                    if len(suggestion) != 0:
                        sys.stdout.write(
                            f"\nSuggestion: {self.generate_suggestion()}\n",
                        )

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


if __name__ == "__main__":
    main()
