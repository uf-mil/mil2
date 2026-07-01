import subprocess
import threading
import time

import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# Long-running processes. Launched in the background and kept alive for the run.
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


class StartupSequencer:
    # Run sub9 bring-up sequence exactly once, on a background thread

    def __init__(self, logger, startup_delay: float, mission: str):
        self._logger = logger
        self._startup_delay = startup_delay
        self._mission = mission
        self._started = False
        self._lock = threading.Lock()
        self._processes: list[subprocess.Popen] = []

    def trigger(self):
        # Starts the bring-up sequence. Safe to call repeatedly.. runs once.
        with self._lock:
            if self._started:
                return
            self._started = True

        thread = threading.Thread(target=self._run_sequence, daemon=True)
        thread.start()

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

    def _start_background(self, name: str, cmd: list[str]):
        # Launch a long-running process and keep a handle to it
        proc = subprocess.Popen(cmd)
        self._processes.append(proc)
        self._logger.info(f"Launched '{name}' (pid {proc.pid}).")

    def _run_blocking(self, name: str, cmd: list[str]):
        # Run a one-shot command and wait for it to finish
        self._logger.info(f"Running '{name}'.")
        result = subprocess.run(cmd)
        if result.returncode != 0:
            self._logger.error(f"'{name}' exited with code {result.returncode}.")

    def shutdown(self):
        """Terminate any background processes we started."""
        for proc in self._processes:
            if proc.poll() is None:
                proc.terminate()


class StartWand(Node):
    """ROS 2 node that watches the start wand and boots the sub9 stack."""

    def __init__(self):
        super().__init__("start_wand")

        # Parameters (override with --ros-args -p name:=value)
        self.gpio_pin = self.declare_parameter("gpio_pin", 7).value
        self.poll_hz = self.declare_parameter("poll_hz", 10.0).value
        self.topic = self.declare_parameter("topic", "/gpio9_state").value
        self.startup_delay = self.declare_parameter("startup_delay", 15.0).value
        # Mission passed down to the mission planner node (its "mission" parameter).
        self.mission = self.declare_parameter("mission", "SonarFollowerTest").value

        if self.poll_hz <= 0:
            raise ValueError("poll_hz must be a positive number.")

        self.pub = self.create_publisher(Bool, self.topic, 10)

        # Bring-up is triggered on the wand's rising edge (device becomes active).
        self.sequencer = StartupSequencer(
            self.get_logger(),
            self.startup_delay,
            self.mission,
        )

        # ---- GPIO SETUP ----
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.gpio_pin, GPIO.IN)

        self._last_logical_state: bool | None = None  # None means not read yet
        self.poll_timer = self.create_timer(1.0 / self.poll_hz, self._poll)

        self.get_logger().info(
            f"Monitoring GPIO (board pin {self.gpio_pin}) at {self.poll_hz} Hz "
            f"on topic '{self.topic}'.",
        )

    def _read_logical(self) -> bool:
        """
        Return the logical state of the active-low device.
        GPIO reads LOW (0V)   -> device ACTIVE   -> logical HIGH (True)
        GPIO reads HIGH (3.3V) -> device INACTIVE -> logical LOW (False)
        """
        raw = GPIO.input(self.gpio_pin)
        return raw == GPIO.LOW  # invert: active-low

    def _poll(self):
        logical = self._read_logical()

        if self._last_logical_state is None:
            # First read, just record state, no transition to report yet.
            self._last_logical_state = logical
            state_str = "HIGH (device active)" if logical else "LOW (device inactive)"
            self.get_logger().info(f"Initial GPIO state: {state_str}")
            self._publish(logical)
            self._maybe_trigger_startup(logical)
            return

        if logical != self._last_logical_state:
            direction = "LOW → HIGH" if logical else "HIGH → LOW"
            meaning = "device became ACTIVE" if logical else "device became INACTIVE"
            self.get_logger().info(f"GPIO state change: {direction}  ({meaning})")
            self._last_logical_state = logical
            self._publish(logical)
            self._maybe_trigger_startup(logical)

    def _maybe_trigger_startup(self, logical: bool):
        # Run the bring-up sequence when the wand becomes active (once)
        if logical:
            self.sequencer.trigger()

    def _publish(self, logical: bool):
        msg = Bool()
        msg.data = logical
        self.pub.publish(msg)

    def cleanup(self):
        # Stop background processes and release the GPIO pins
        self.sequencer.shutdown()
        GPIO.cleanup()


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
