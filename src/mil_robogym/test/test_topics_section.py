from mil_robogym.ui.pages.create_project_page.topics_section import (
    _format_topic_warning_message,
)


def test_format_topic_warning_message_with_matches() -> None:
    warning = {
        "topic": "/camera/image_raw",
        "reason": "raw_sensor_stream",
        "matched": ["image_raw", "camera"],
        "category": "raw_sensor_stream",
    }

    message = _format_topic_warning_message(warning)

    assert "Raw sensor stream" in message
    assert "'image_raw'" in message
    assert "'camera'" in message


def test_format_topic_warning_message_without_matches() -> None:
    warning = {
        "topic": "/foo",
        "reason": "unknown",
        "matched": [],
        "category": "unknown_bucket",
    }

    message = _format_topic_warning_message(warning)

    assert message == "Unknown Bucket: this topic is likely unhelpful for training."
