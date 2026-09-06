#!/usr/bin/env python3

"""
Start-wand bootstrap script.

Monitors the start wand on GPIO9. When the wand becomes ACTIVE it brings the
whole sub9 stack online, in this exact order:

    1. ros2 launch subjugator_bringup sub9.launch.py   (long-running)
    2. start-localization                              (service call)
    3. reset-localization                              (service call)
    4. start-controller                                (service call)
    5. unkill                                          (service call)
    6. ros2 run subjugator_mission_planner mission_planner   (long-running)

The bring-up runs exactly once, on a background thread, so the GPIO monitor
keeps polling and publishing while the stack comes up.
"""


# **********<Dependencies>**********#
import argparse
import subprocess
import threading
import time

import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# **********</Dependencies>**********#


# **********<Definitions>**********#
GPIO_PIN = 7  # GPIO9 = pin 7

# Seconds to wait after launching the stack before issuing service calls, giving
# the localization/controller/kill nodes time to register their services.
STACK_STARTUP_DELAY = 5.0

# Long-running processes. These are launched in the background and kept alive
# for the lifetime of the run.
LAUNCH_CMD = ["ros2", "launch", "subjugator_bringup", "sub9.launch.py"]
MISSION_CMD = ["ros2", "run", "mission_planner", "mission_planner_node"]


# One-shot service calls, run in order once the stack is up. These mirror the
# start-localization -> reset-localization -> start-controller -> unkill aliases
# defined in scripts/setup.bash.
SERVICE_CALLS = [
    (
        "start-localization",
        [
            "ros2",
            "service",
            "call",
            "/subjugator_localization/enable",
            "std_srvs/srv/Empty",
        ],
    ),
    (
        "reset-localization",
        [
            "ros2",
            "service",
            "call",
            "/subjugator_localization/reset",
            "std_srvs/srv/Empty",
        ],
    ),
    (
        "start-controller",
        [
            "ros2",
            "service",
            "call",
            "/pid_controller/enable",
            "std_srvs/srv/SetBool",
            "{data: true}",
        ],
    ),
    (
        "unkill",
        ["ros2", "service", "call", "/unkill", "std_srvs/srv/Empty"],
    ),
]
# **********</Definitions>**********#


# **********<Startup Sequence>**********#
class StartupSequencer:
    """Runs the sub9 bring-up sequence exactly once, on a background thread."""

    def __init__(self, logger):
        self._logger = logger
        self._started = False
        self._lock = threading.Lock()
        self._processes: list[subprocess.Popen] = []

    def trigger(self):
        """Kick off the bring-up sequence. Safe to call repeatedly; runs once."""
        with self._lock:
            if self._started:
                return
            self._started = True

        thread = threading.Thread(target=self._run_sequence, daemon=True)
        thread.start()

    def _run_sequence(self):
        # 1. Launch the full sub9 stack in the background.
        self._logger.info("Start wand active — bringing up sub9 stack.")
        self._start_background("sub9.launch", LAUNCH_CMD)

        self._logger.info(f"Waiting {STACK_STARTUP_DELAY}s for the stack to come up.")
        time.sleep(STACK_STARTUP_DELAY)

        # 2-5. Enable localization, reset it, enable the controller, then unkill.
        for name, cmd in SERVICE_CALLS:
            self._run_blocking(name, cmd)

        # 6. Run the mission planner.
        self._logger.info("Starting mission planner.")
        self._start_background("mission_planner", MISSION_CMD)

        self._logger.info("Bring-up sequence complete.")

    def _start_background(self, name: str, cmd: list[str]):
        """Launch a long-running process and keep a handle to it."""
        proc = subprocess.Popen(cmd)
        self._processes.append(proc)
        self._logger.info(f"Launched '{name}' (pid {proc.pid}).")

    def _run_blocking(self, name: str, cmd: list[str]):
        """Run a one-shot command and wait for it to finish."""
        self._logger.info(f"Running '{name}'.")
        result = subprocess.run(cmd)
        if result.returncode != 0:
            self._logger.error(
                f"'{name}' exited with code {result.returncode}.",
            )

    def shutdown(self):
        """Terminate any background processes we started."""
        for proc in self._processes:
            if proc.poll() is None:
                proc.terminate()


# **********</Startup Sequence>**********#


# **********<Topic Definition>**********#
class GPIOMonitor(Node):
    def __init__(
        self,
        poll_hz: float,
        topic: str,
        timeout: float | None = None,
        run_startup: bool = True,
    ):
        super().__init__("gpio_monitor")

        self.pub = self.create_publisher(Bool, topic, 10)
        self.poll_hz = poll_hz
        self.timeout = timeout
        self._last_logical_state: bool | None = None  # None means not read yet

        # Bring-up is triggered on the wand's rising edge (device becomes active).
        self.sequencer = StartupSequencer(self.get_logger()) if run_startup else None

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

    # **********</Topic Definition>**********#

    # **********<Helper Functions>**********#
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
            # First read, just record state, no transition to report yet
            self._last_logical_state = logical
            state_str = "HIGH (device active)" if logical else "LOW (device inactive)"
            self.get_logger().info(f"Initial GPIO9 state: {state_str}")
            self._publish(logical)
            self._maybe_trigger_startup(logical)
            return

        if logical != self._last_logical_state:
            direction = "LOW → HIGH" if logical else "HIGH → LOW"
            meaning = "device became ACTIVE" if logical else "device became INACTIVE"
            self.get_logger().info(
                f"GPIO9 state change: {direction}  ({meaning})",
            )
            self._last_logical_state = logical
            self._publish(logical)
            self._maybe_trigger_startup(logical)

    def _maybe_trigger_startup(self, logical: bool):
        """Run the bring-up sequence when the wand becomes active (once)."""
        if logical and self.sequencer is not None:
            self.sequencer.trigger()

    def _publish(self, logical: bool):
        msg = Bool()
        msg.data = logical
        self.pub.publish(msg)

    # **********</Helper Functions>**********#

    # **********<Init and Deinit>**********#
    def _on_timeout(self):
        self.get_logger().info("Timeout reached. Shutting down.")
        self._cleanup()
        rclpy.shutdown()

    def _cleanup(self):
        if self.sequencer is not None:
            self.sequencer.shutdown()
        GPIO.cleanup()


# **********</Init and Deinit>**********#


# **********<Main>**********#
def main():
    parser = argparse.ArgumentParser(
        description="Monitor GPIO9 on the Jetson Orin Nano and, on the start "
        "wand signal, bring up the sub9 stack.",
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
    parser.add_argument(
        "--monitor-only",
        action="store_true",
        help="Only publish GPIO state; do not run the sub9 bring-up sequence.",
    )
    args = parser.parse_args()

    if args.hz <= 0:
        raise ValueError("--hz must be a positive number.")

    rclpy.init()
    node = GPIOMonitor(
        poll_hz=args.hz,
        topic=args.topic,
        timeout=args.timeout,
        run_startup=not args.monitor_only,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt. Stopping...")
    finally:
        node._cleanup()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
# **********</Main>**********#
