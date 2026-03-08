from __future__ import annotations

import argparse
import io
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from .filesystem import write_demo_topic_png
from .get_ros2_topics import get_ros2_topics
from .sample_input_topics import (
    collect_numeric_values_from_topic_subtopics,
    collect_topic_messages_once,
    numerical_headers_from_topic_subtopics,
)

_STOP_PUBLISHER = False


@dataclass
class LiveTestConfig:
    timeout_s: float
    graph_timeout_s: float
    publish_rate_hz: float
    publisher_duration_s: float
    output_dir: str


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Live ROS 2 smoke test for subscriber-based sampling helpers in "
            "mil_robogym.data_collection.sample_input_topics."
        ),
    )
    parser.add_argument(
        "--publisher-mode",
        action="store_true",
        help="Internal mode: run the temporary test publisher.",
    )
    parser.add_argument(
        "--numeric-topic",
        default="",
        help="Topic name for std_msgs/msg/Float64 publisher (publisher mode only).",
    )
    parser.add_argument(
        "--status-topic",
        default="",
        help="Topic name for std_msgs/msg/String publisher (publisher mode only).",
    )
    parser.add_argument(
        "--image-topic",
        default="",
        help="Topic name for sensor_msgs/msg/Image publisher (publisher mode only).",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=3.0,
        help="Sampling timeout in seconds for collector functions.",
    )
    parser.add_argument(
        "--graph-timeout-s",
        type=float,
        default=5.0,
        help="Seconds to wait until temporary test topics appear in ROS graph.",
    )
    parser.add_argument(
        "--publish-rate-hz",
        type=float,
        default=20.0,
        help="Rate for temporary publishers in Hz.",
    )
    parser.add_argument(
        "--publisher-duration-s",
        type=float,
        default=60.0,
        help="Max lifetime of temporary publisher process in seconds.",
    )
    parser.add_argument(
        "--output-dir",
        default="/tmp/mil_robogym_live_sampling_output",
        help=(
            "Directory where smoke-test demo artifacts are written. "
            "A run-specific subdirectory is created automatically."
        ),
    )
    return parser


def _validate_positive(value: float, *, name: str) -> None:
    if value <= 0:
        raise ValueError(f"{name} must be positive.")


def _preflight_ros_environment() -> None:
    try:
        import rclpy  # noqa: F401
    except ImportError as e:
        raise RuntimeError(
            "Missing ROS Python dependency 'rclpy'. Source your ROS 2 environment "
            "before running this smoke test.",
        ) from e

    try:
        get_ros2_topics()
    except FileNotFoundError as e:
        raise RuntimeError(
            "No ROS 2 CLI found ('ros2' not in PATH). Source your ROS 2 environment first.",
        ) from e
    except RuntimeError:
        # Topic listing can fail transiently while discovery starts. The live
        # wait phase handles retries.
        pass


def _wait_for_topics(
    topics: Sequence[str],
    *,
    timeout_s: float,
    publisher: subprocess.Popen[bytes] | None = None,
) -> None:
    deadline = time.monotonic() + timeout_s
    expected = set(topics)
    while time.monotonic() < deadline:
        if publisher is not None and publisher.poll() is not None:
            raise RuntimeError(
                "Temporary publisher process exited before topics appeared "
                f"(exit_code={publisher.returncode}).",
            )
        try:
            available = set(get_ros2_topics())
        except (RuntimeError, FileNotFoundError):
            time.sleep(0.1)
            continue
        if expected.issubset(available):
            return
        time.sleep(0.1)

    raise RuntimeError(
        "Timed out waiting for test topics to appear in ROS graph: "
        f"{sorted(expected)}",
    )


def _signal_stop_publisher(_sig: int, _frame: object) -> None:
    global _STOP_PUBLISHER
    _STOP_PUBLISHER = True


def _run_publisher_mode(
    *,
    numeric_topic: str,
    status_topic: str,
    image_topic: str,
    publish_rate_hz: float,
    publisher_duration_s: float,
) -> int:
    if not numeric_topic or not status_topic or not image_topic:
        raise ValueError(
            "--numeric-topic, --status-topic, and --image-topic are required in --publisher-mode.",
        )
    _validate_positive(publish_rate_hz, name="publish_rate_hz")
    _validate_positive(publisher_duration_s, name="publisher_duration_s")

    signal.signal(signal.SIGTERM, _signal_stop_publisher)
    signal.signal(signal.SIGINT, _signal_stop_publisher)

    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float64, String

    context = Context()
    rclpy.init(context=context)
    node = Node("robogym_live_sampling_test_pub", context=context)

    numeric_pub = node.create_publisher(Float64, numeric_topic, 10)
    status_pub = node.create_publisher(String, status_topic, 10)
    image_pub = node.create_publisher(Image, image_topic, 10)

    period_s = 1.0 / publish_rate_hz
    start = time.monotonic()
    count = 0
    width = 96
    height = 60
    step = width * 3

    try:
        while (
            not _STOP_PUBLISHER
            and context.ok()
            and (time.monotonic() - start) < publisher_duration_s
        ):
            numeric_msg = Float64()
            numeric_msg.data = 100.0 + float(count)
            numeric_pub.publish(numeric_msg)

            status_msg = String()
            status_msg.data = f"tick-{count}"
            status_pub.publish(status_msg)

            image_msg = Image()
            image_msg.header.stamp = node.get_clock().now().to_msg()
            image_msg.header.frame_id = "robogym_live_test/frontcam"
            image_msg.height = height
            image_msg.width = width
            image_msg.encoding = "rgb8"
            image_msg.is_bigendian = 0
            image_msg.step = step

            image_data = bytearray(height * step)
            for y in range(height):
                for x in range(width):
                    idx = (y * step) + (x * 3)
                    image_data[idx + 0] = (x * 2 + count) % 256
                    image_data[idx + 1] = (y * 4) % 256
                    image_data[idx + 2] = ((x + y) * 3) % 256
            image_msg.data = bytes(image_data)
            image_pub.publish(image_msg)

            time.sleep(period_s)
            count += 1
    finally:
        node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)
    return 0


def _build_test_topics() -> tuple[str, str, str]:
    # ROS topic tokens cannot start with digits, so keep this token alpha-prefixed.
    suffix = f"run_{os.getpid()}_{int(time.time() * 1000)}"
    base = f"/robogym_live_sampling_test/{suffix}"
    return (
        f"{base}/numeric",
        f"{base}/status",
        f"{base}/frontcam_image_raw",
    )


def _launch_publisher_subprocess(
    *,
    numeric_topic: str,
    status_topic: str,
    image_topic: str,
    cfg: LiveTestConfig,
) -> subprocess.Popen[bytes]:
    command = [
        sys.executable,
        "-m",
        "mil_robogym.data_collection.live_ros_sampling_test",
        "--publisher-mode",
        "--numeric-topic",
        numeric_topic,
        "--status-topic",
        status_topic,
        "--image-topic",
        image_topic,
        "--publish-rate-hz",
        str(cfg.publish_rate_hz),
        "--publisher-duration-s",
        str(cfg.publisher_duration_s),
    ]
    return subprocess.Popen(command)


def _stop_publisher_subprocess(proc: subprocess.Popen[bytes]) -> None:
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=3.0)


def _image_message_dict_to_png_bytes(image_message: dict[str, object]) -> bytes:
    try:
        from PIL import Image
    except ImportError as e:
        raise RuntimeError(
            "Pillow is required to convert sampled image messages to PNG bytes.",
        ) from e

    encoding = image_message.get("encoding")
    if encoding != "rgb8":
        raise RuntimeError(
            f"Only 'rgb8' image encoding is supported in this smoke test, got: {encoding!r}",
        )

    width = image_message.get("width")
    height = image_message.get("height")
    step = image_message.get("step")
    data = image_message.get("data")
    if not isinstance(width, int) or not isinstance(height, int):
        raise RuntimeError("Image message is missing valid width/height.")
    if width <= 0 or height <= 0:
        raise RuntimeError("Image message width/height must be positive.")
    if not isinstance(step, int) or step < width * 3:
        raise RuntimeError("Image message has invalid step size for rgb8.")
    if not isinstance(data, list):
        raise RuntimeError("Image message data field must be a list.")

    expected = height * step
    if len(data) < expected:
        raise RuntimeError(
            "Image message data is shorter than expected by height*step: "
            f"{len(data)} < {expected}.",
        )

    raw = bytes((int(value) & 0xFF) for value in data[:expected])
    if step == width * 3:
        rgb_packed = raw
    else:
        rows = [raw[(row * step) : (row * step + width * 3)] for row in range(height)]
        rgb_packed = b"".join(rows)

    image = Image.frombytes("RGB", (width, height), rgb_packed)
    output = io.BytesIO()
    image.save(output, format="PNG")
    return output.getvalue()


def _run_live_sampling_smoke_test(cfg: LiveTestConfig) -> int:
    _preflight_ros_environment()

    numeric_topic, status_topic, image_topic = _build_test_topics()
    run_token = image_topic.split("/")[-2]
    publisher = _launch_publisher_subprocess(
        numeric_topic=numeric_topic,
        status_topic=status_topic,
        image_topic=image_topic,
        cfg=cfg,
    )

    try:
        _wait_for_topics(
            [numeric_topic, status_topic, image_topic],
            timeout_s=cfg.graph_timeout_s,
            publisher=publisher,
        )

        messages = collect_topic_messages_once(
            [numeric_topic, status_topic, image_topic],
            timeout_s=cfg.timeout_s,
        )
        topic_subtopics = {numeric_topic: ["data"]}
        headers = numerical_headers_from_topic_subtopics(topic_subtopics)
        numeric_values = collect_numeric_values_from_topic_subtopics(
            topic_subtopics,
            timeout_s=cfg.timeout_s,
        )

        if len(messages) != 3:
            raise RuntimeError(f"Expected 3 messages, got {len(messages)}")
        numeric_message = messages[0]
        status_message = messages[1]
        image_message = messages[2]
        if not isinstance(numeric_message.get("data"), (bool, int, float)):
            raise RuntimeError(
                "Numeric message did not contain scalar numeric field 'data'.",
            )
        if not isinstance(status_message.get("data"), str):
            raise RuntimeError(
                "Status message did not contain string field 'data'.",
            )
        if image_message.get("encoding") != "rgb8":
            raise RuntimeError("Sampled image message did not contain rgb8 encoding.")
        if headers != [f"{numeric_topic}:data"]:
            raise RuntimeError(f"Unexpected numerical headers: {headers}")
        if len(numeric_values) != 1:
            raise RuntimeError(
                f"Expected one numeric sampled value, got {len(numeric_values)}",
            )
        if not isinstance(numeric_values[0], (bool, int, float)):
            raise RuntimeError(
                "Sampled numeric value is missing or non-numeric.",
            )

        png_data = _image_message_dict_to_png_bytes(image_message)
        demo_dir = Path(cfg.output_dir).expanduser().resolve() / run_token
        image_path = write_demo_topic_png(
            demo_dir,
            topic=image_topic,
            file_name="image0.png",
            png_data=png_data,
        )

        print("Live ROS sampling smoke test passed.")
        print(f"numeric_topic={numeric_topic}")
        print(f"status_topic={status_topic}")
        print(f"image_topic={image_topic}")
        print(f"sampled_headers={headers}")
        print(f"sampled_numeric_values={numeric_values}")
        print(f"saved_image_path={image_path}")
        return 0
    finally:
        _stop_publisher_subprocess(publisher)


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.publisher_mode:
        return _run_publisher_mode(
            numeric_topic=args.numeric_topic,
            status_topic=args.status_topic,
            image_topic=args.image_topic,
            publish_rate_hz=args.publish_rate_hz,
            publisher_duration_s=args.publisher_duration_s,
        )

    cfg = LiveTestConfig(
        timeout_s=args.timeout_s,
        graph_timeout_s=args.graph_timeout_s,
        publish_rate_hz=args.publish_rate_hz,
        publisher_duration_s=args.publisher_duration_s,
        output_dir=args.output_dir,
    )
    _validate_positive(cfg.timeout_s, name="timeout_s")
    _validate_positive(cfg.graph_timeout_s, name="graph_timeout_s")
    _validate_positive(cfg.publish_rate_hz, name="publish_rate_hz")
    _validate_positive(cfg.publisher_duration_s, name="publisher_duration_s")
    return _run_live_sampling_smoke_test(cfg)


if __name__ == "__main__":
    raise SystemExit(main())
