#!/usr/bin/env python3

"""
Auto driver script.

Monitors the start wand on a GPIO pin. When the wand becomes ACTIVE it brings
the whole sub9 stack online, in this order:

    1. ros2 launch subjugator_bringup sub9.launch.py   (background)
    2. ros2 launch yolo_bringup yolov26.launch.py      (background)
    3. start-localization                              (service call)
    4. reset-localization                              (service call)
    5. start-controller                                (service call)
    6. unkill                                          (service call)
    7. ros2 run mission_planner mission_planner_node   (background)

A hall-effect sensor detects a magnet on the computer box exterior and restarts
the mission planner without a tether:

    1. Kill mission_planner
    2. reset-localization  (service call)
    3. Relaunch mission_planner
"""


# **********<Dependencies>**********#
import subprocess
import threading
import time

import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# **********</Dependencies>**********#


# **********<Definitions>**********#
LAUNCH_CMD = ["ros2", "launch", "subjugator_bringup", "sub9.launch.py"]
YOLO_LAUNCH_CMD = ["ros2", "launch", "yolo_bringup", "yolov26.launch.py"]
MISSION_CMD = ["ros2", "run", "mission_planner", "mission_planner_node"]

# One-shot service calls, run in order once the stack is up
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

RESET_LOCALIZATION_CMD = [
    "ros2",
    "service",
    "call",
    "/subjugator_localization/reset",
    "std_srvs/srv/Empty",
]
# **********</Definitions>**********#


# **********<Startup Sequencer>**********#
class StartupSequencer:
    """Manages sub9 bring-up and named process lifecycle."""

    def __init__(self, logger, startup_delay: float, mission: str):
        self._logger = logger
        self._startup_delay = startup_delay
        self._mission = mission
        self._started = False
        self._start_lock = threading.Lock()
        self._restart_lock = threading.Lock()
        # name -> (cmd, process)  — keyed so processes can be restarted by name
        self._processes: dict[str, tuple[list[str], subprocess.Popen]] = {}

    def trigger(self):
        """Kick off the bring-up sequence. Safe to call repeatedly; runs once."""
        with self._start_lock:
            if self._started:
                return
            self._started = True

        thread = threading.Thread(target=self._run_sequence, daemon=True)
        thread.start()

    def is_running(self) -> bool:
        return self._started

    def _run_sequence(self):
        # 1. Launch the full sub9 stack and the YOLO vision stack in the background.
        self._logger.info("Start wand active — bringing up DA SUB")
        self._start_background("sub9.launch", LAUNCH_CMD)
        self._start_background("yolo.launch", YOLO_LAUNCH_CMD)

        self._logger.info(f"Waiting {self._startup_delay}s for the stack to come up.")
        time.sleep(self._startup_delay)

        # 2-5. Enable localization, reset it, enable the controller, then unkill.
        for name, cmd in SERVICE_CALLS:
            self._run_blocking(name, cmd)

        # 6. Run the mission planner with the chosen mission.
        self._logger.info(f"Starting mission planner with mission '{self._mission}'.")
        mission_cmd = [*MISSION_CMD, "--ros-args", "-p", f"mission:={self._mission}"]
        self._start_background("mission_planner", mission_cmd)

        self._logger.info("Bring-up sequence complete.")

    def restart_mission(self):
        """
        Kill the mission planner, reset localization, then relaunch.

        Thread-safe: concurrent calls are dropped. Ignored if the stack has not
        yet been brought up or the mission planner has not launched.
        """
        if not self._started:
            self._logger.warning("Stack not yet started; ignoring magnet trigger.")
            return
        if "mission_planner" not in self._processes:
            self._logger.warning("Mission planner not yet launched; ignoring magnet trigger.")
            return
        if not self._restart_lock.acquire(blocking=False):
            self._logger.warning("Restart already in progress; ignoring magnet trigger.")
            return
        thread = threading.Thread(target=self._do_restart_mission, daemon=True)
        thread.start()

    def _do_restart_mission(self):
        try:
            self._logger.info("Magnet trigger: restarting mission.")

            # 1. Kill mission planner.
            name = "mission_planner"
            cmd, proc = self._processes[name]
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()

            # 2. Reset localization so the next run starts from a clean state.
            self._run_blocking("reset-localization", RESET_LOCALIZATION_CMD)

            # 3. Relaunch mission planner.
            new_proc = subprocess.Popen(cmd)
            self._processes[name] = (cmd, new_proc)
            self._logger.info(f"Restarted '{name}' (pid {new_proc.pid}).")
        finally:
            self._restart_lock.release()

        self._logger.info("Mission restart complete.")

    def _start_background(self, name: str, cmd: list[str]):
        """Launch a long-running process and track it by name."""
        proc = subprocess.Popen(cmd)
        self._processes[name] = (cmd, proc)
        self._logger.info(f"Launched '{name}' (pid {proc.pid}).")

    def _run_blocking(self, name: str, cmd: list[str]):
        """Run a one-shot command and wait for it to finish."""
        self._logger.info(f"Running '{name}'.")
        result = subprocess.run(cmd)
        if result.returncode != 0:
            self._logger.error(f"'{name}' exited with code {result.returncode}.")

    def shutdown(self):
        """Terminate all tracked background processes."""
        for name, (cmd, proc) in self._processes.items():
            if proc.poll() is None:
                proc.terminate()


# **********</Startup Sequencer>**********#


# **********<Node>**********#
class StartWand(Node):
    """ROS 2 node that watches the start wand and hall-effect magnet sensor."""

    def __init__(self):
        super().__init__("start_wand")

        # --- Parameters ---
        self.gpio_pin = self.declare_parameter("gpio_pin", 7).value
        self.poll_hz = self.declare_parameter("poll_hz", 10.0).value
        self.topic = self.declare_parameter("topic", "/gpio9_state").value
        self.startup_delay = self.declare_parameter("startup_delay", 15.0).value
        self.mission = self.declare_parameter("mission", "SonarFollowerTest").value

        # TODO: set magnet_gpio_pin to the actual hall-effect sensor board pin
        self.magnet_gpio_pin = self.declare_parameter("magnet_gpio_pin", 11).value
        self.magnet_topic = self.declare_parameter("magnet_topic", "/magnet_state").value
        # Seconds to ignore further magnet triggers after a restart fires.
        self.magnet_debounce_s = self.declare_parameter("magnet_debounce_s", 3.0).value

        if self.poll_hz <= 0:
            raise ValueError("poll_hz must be a positive number.")
        if self.magnet_gpio_pin == self.gpio_pin:
            raise ValueError("magnet_gpio_pin and gpio_pin must be different pins.")

        # --- Publishers ---
        self.pub = self.create_publisher(Bool, self.topic, 10)
        self.magnet_pub = self.create_publisher(Bool, self.magnet_topic, 10)

        # --- Sequencer (shared by wand and magnet monitors) ---
        self.sequencer = StartupSequencer(
            self.get_logger(),
            self.startup_delay,
            self.mission,
        )

        # --- GPIO setup ---
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.gpio_pin, GPIO.IN)
        GPIO.setup(self.magnet_gpio_pin, GPIO.IN)

        # --- Start-wand polling state ---
        self._last_wand_state: bool | None = None
        self.poll_timer = self.create_timer(1.0 / self.poll_hz, self._poll_wand)

        # --- Magnet polling state ---
        self._last_magnet_state: bool | None = None
        self._last_restart_time: float = 0.0
        self.magnet_timer = self.create_timer(1.0 / self.poll_hz, self._poll_magnet)

        self.get_logger().info(
            f"Start wand:    GPIO board pin {self.gpio_pin} → topic '{self.topic}'\n"
            f"Magnet sensor: GPIO board pin {self.magnet_gpio_pin} → topic '{self.magnet_topic}' "
            f"(debounce {self.magnet_debounce_s}s)",
        )

    # **********</Node>**********#

    # **********<GPIO Helpers>**********#
    def _read_logical(self, pin: int) -> bool:
        """
        Return the logical state of an active-low device.
        GPIO reads LOW (0V)    -> device ACTIVE   -> logical True
        GPIO reads HIGH (3.3V) -> device INACTIVE -> logical False
        """
        return GPIO.input(pin) == GPIO.LOW

    def _publish(self, publisher, logical: bool):
        msg = Bool()
        msg.data = logical
        publisher.publish(msg)

    # **********</GPIO Helpers>**********#

    # **********<Wand Polling>**********#
    def _poll_wand(self):
        logical = self._read_logical(self.gpio_pin)

        if self._last_wand_state is None:
            self._last_wand_state = logical
            state_str = "HIGH (device active)" if logical else "LOW (device inactive)"
            self.get_logger().info(f"Initial wand state: {state_str}")
            self._publish(self.pub, logical)
            if logical:
                self.sequencer.trigger()
            return

        if logical != self._last_wand_state:
            direction = "LOW → HIGH" if logical else "HIGH → LOW"
            meaning = "device became ACTIVE" if logical else "device became INACTIVE"
            self.get_logger().info(f"Wand state change: {direction}  ({meaning})")
            self._last_wand_state = logical
            self._publish(self.pub, logical)
            if logical:
                self.sequencer.trigger()

    # **********</Wand Polling>**********#

    # **********<Magnet Polling>**********#
    def _poll_magnet(self):
        logical = self._read_logical(self.magnet_gpio_pin)

        if self._last_magnet_state is None:
            self._last_magnet_state = logical
            return

        if logical != self._last_magnet_state:
            self._last_magnet_state = logical
            self._publish(self.magnet_pub, logical)

            if logical:  # rising edge: magnet applied
                now = time.time()
                elapsed = now - self._last_restart_time
                if elapsed >= self.magnet_debounce_s:
                    self._last_restart_time = now
                    self.sequencer.restart_mission()
                else:
                    remaining = self.magnet_debounce_s - elapsed
                    self.get_logger().info(
                        f"Magnet trigger ignored (debounce: {remaining:.1f}s remaining).",
                    )

    # **********</Magnet Polling>**********#

    # **********<Cleanup>**********#
    def cleanup(self):
        """Stop background processes and release GPIO pins."""
        self.sequencer.shutdown()
        GPIO.cleanup()

    # **********</Cleanup>**********#


# **********<Main>**********#
def main():
    rclpy.init()
    node = StartWand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
# **********</Main>**********#
