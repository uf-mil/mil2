"""Unit tests for live ROS sampling smoke-test helpers."""

import pytest

from mil_robogym.data_collection.live_ros_sampling_test import (
    _build_test_topics,
    _image_message_dict_to_png_bytes,
)


def test_build_test_topics_returns_expected_structure():
    """Builds deterministic test topic structure with a valid alpha token."""
    numeric_topic, status_topic, image_topic = _build_test_topics()

    assert numeric_topic.startswith("/robogym_live_sampling_test/run_")
    assert status_topic.startswith("/robogym_live_sampling_test/run_")
    assert image_topic.startswith("/robogym_live_sampling_test/run_")
    assert numeric_topic.endswith("/numeric")
    assert status_topic.endswith("/status")
    assert image_topic.endswith("/frontcam_image_raw")


def test_image_message_dict_to_png_bytes_rgb8():
    """Converts an rgb8 image dict payload into PNG bytes."""
    pytest.importorskip("PIL")

    image_message = {
        "encoding": "rgb8",
        "width": 2,
        "height": 2,
        "step": 6,
        "data": [
            255,
            0,
            0,
            0,
            255,
            0,
            0,
            0,
            255,
            255,
            255,
            255,
        ],
    }

    png_data = _image_message_dict_to_png_bytes(image_message)
    assert png_data.startswith(b"\x89PNG\r\n\x1a\n")


def test_image_message_dict_to_png_bytes_rejects_non_rgb8():
    """Rejects unsupported image encodings with a clear error."""
    with pytest.raises(RuntimeError, match="Only 'rgb8' image encoding"):
        _image_message_dict_to_png_bytes(
            {
                "encoding": "bgr8",
                "width": 2,
                "height": 2,
                "step": 6,
                "data": [0] * 12,
            },
        )
