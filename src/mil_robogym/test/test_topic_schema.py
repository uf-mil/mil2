"""Tests for supported non-numeric topic field classification."""

from mil_robogym.data_collection.topic_schema import (
    supported_non_numeric_fields_for_message_type,
)


class _WrapperMessage:
    @classmethod
    def get_fields_and_field_types(cls) -> dict[str, str]:
        return {
            "detections": "sequence<vision_msgs/msg/Detection2D>",
            "camera": "sensor_msgs/msg/Image",
            "label": "string",
            "count": "uint32",
        }


class _NestedMessage:
    @classmethod
    def get_fields_and_field_types(cls) -> dict[str, str]:
        return {
            "wrapper": "example_msgs/msg/Wrapper",
            "scores": "sequence<float32>",
            "status": "string",
        }


def test_supported_non_numeric_fields_include_images_and_unordered_sets(monkeypatch):
    """Classifies image fields and variable-length arrays as supported special data."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.topic_schema._load_message_class",
        lambda message_type: {
            "example_msgs/msg/Wrapper": _WrapperMessage,
        }[message_type],
    )

    fields = supported_non_numeric_fields_for_message_type("example_msgs/msg/Wrapper")

    assert [
        (field.field_path, field.data_type, field.ros_type) for field in fields
    ] == [
        (
            "detections",
            "unordered_set",
            "sequence<vision_msgs/msg/Detection2D>",
        ),
        ("camera.data", "image", "sensor_msgs/msg/Image"),
    ]


def test_supported_non_numeric_fields_recurse_into_nested_messages(monkeypatch):
    """Finds supported nested fields without surfacing plain strings or scalars."""
    monkeypatch.setattr(
        "mil_robogym.data_collection.topic_schema._load_message_class",
        lambda message_type: {
            "example_msgs/msg/Nested": _NestedMessage,
            "example_msgs/msg/Wrapper": _WrapperMessage,
        }[message_type],
    )

    fields = supported_non_numeric_fields_for_message_type("example_msgs/msg/Nested")

    assert [(field.field_path, field.data_type) for field in fields] == [
        ("wrapper.detections", "unordered_set"),
        ("wrapper.camera.data", "image"),
        ("scores", "unordered_set"),
    ]
