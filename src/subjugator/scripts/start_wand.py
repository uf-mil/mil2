#!/usr/bin/env python3

"""
Checks for the start wand, starts a certain task
"""


#**********<Dependencies>**********#
import argparse

import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Node
#**********</Dependencies>**********#


#**********<Definitions>**********#
GPIO_PIN = 7 # GPIO9 = pin 7
#**********</Definitions>**********#



#**********<Topic Definition>**********#
class GPIOMonitor(Node):
    def __init__(self, poll_hz: float, topic: str, timeout: float | None = None):
        super().__init__("gpio_monitor")
        
        self.pub = self.create_publisher(Bool, topic, 10)
        self.poll_hz = poll_hz
        self.timeout = timeout
        self._last_logical_state: bool | None = None  # None = not yet read
 
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(GPIO_PIN, GPIO.IN)
 
        self.poll_timer = self.create_timer(1.0 / poll_hz, self._poll)
        self.shutdown_timer = None
        if timeout is not None:
            self.shutdown_timer = self.create_timer(timeout, self._on_timeout)
 
        self.get_logger().info(
            f"Monitoring GPIO9 (board pin {GPIO_PIN}) at {poll_hz} Hz "
            f"on topic '{topic}'"
            + (f" for {timeout} seconds" if timeout is not None else ""),
        )
#**********</Topic Definition>**********#



#**********<Helper Functions>**********#
    def _read_logical(self) -> bool:
        """
        Return the logical state of the active-low device.
        GPIO reads LOW (0V)  -> device ACTIVE  -> logical HIGH (True)
        GPIO reads HIGH (3.3V) -> device INACTIVE -> logical LOW (False)
        """
        raw = GPIO.input(GPIO_PIN)
        return raw == GPIO.LOW  # invert: active-low
 
    def _poll(self):
        logical = self._read_logical()
 
        if self._last_logical_state is None:
            # First read — just record state, no transition to report yet.
            self._last_logical_state = logical
            state_str = "HIGH (device active)" if logical else "LOW (device inactive)"
            self.get_logger().info(f"Initial GPIO9 state: {state_str}")
            self._publish(logical)
            return
 
        if logical != self._last_logical_state:
            direction = "LOW → HIGH" if logical else "HIGH → LOW"
            meaning = (
                "device became ACTIVE"
                if logical
                else "device became INACTIVE"
            )
            self.get_logger().info(
                f"GPIO9 state change: {direction}  ({meaning})",
            )
            self._last_logical_state = logical
            self._publish(logical)
 
    def _publish(self, logical: bool):
        msg = Bool()
        msg.data = logical
        self.pub.publish(msg)
#**********</Helper Functions>**********#



#**********<Init and Deinit>**********#
    def _on_timeout(self):
        self.get_logger().info("Timeout reached. Shutting down.")
        self._cleanup()
        rclpy.shutdown()
 
    def _cleanup(self):
        GPIO.cleanup()
#**********</Init and Deinit>**********#



#**********<Main>**********#
def main():
    parser = argparse.ArgumentParser(
        description="Monitor GPIO9 on the Jetson Orin Nano and report state changes.",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=10.0,
        help="Polling rate in Hz (default: 10)",
    )
    parser.add_argument(
        "--topic",
        type=str,
        default="/gpio9_state",
        help="ROS2 topic to publish Bool state on (default: /gpio9_state)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=None,
        help="Stop monitoring after this many seconds (omit for indefinite)",
    )
    args = parser.parse_args()
 
    if args.hz <= 0:
        raise ValueError("--hz must be a positive number.")
 
    rclpy.init()
    node = GPIOMonitor(poll_hz=args.hz, topic=args.topic, timeout=args.timeout)
 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt. Stopping...")
    finally:
        node._cleanup()
        rclpy.shutdown()
        
        
if __name__ == "__main__":
    main()
#**********</Main>**********#
