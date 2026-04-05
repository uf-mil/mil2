from mil_robogym.data_collection.non_numeric_topic_fields import (
    filter_populated_non_numeric_topic_fields,
)


def test_filter_populated_non_numeric_topic_fields_drops_empty_topics() -> None:
    result = filter_populated_non_numeric_topic_fields(
        {
            "/ping": [],
            "/camera/image_raw": [
                {
                    "field_path": "data",
                    "data_type": "image",
                    "ros_type": "sensor_msgs/msg/Image",
                },
            ],
        },
    )

    assert result == {
        "/camera/image_raw": [
            {
                "field_path": "data",
                "data_type": "image",
                "ros_type": "sensor_msgs/msg/Image",
            },
        ],
    }


def test_filter_populated_non_numeric_topic_fields_copies_selected_fields() -> None:
    selected_field = {
        "field_path": "detections",
        "data_type": "unordered_set",
        "ros_type": "sequence<vision_msgs/msg/Detection2D>",
    }

    result = filter_populated_non_numeric_topic_fields(
        {"/detections": [selected_field]},
    )
    selected_field["field_path"] = "mutated"

    assert result == {
        "/detections": [
            {
                "field_path": "detections",
                "data_type": "unordered_set",
                "ros_type": "sequence<vision_msgs/msg/Detection2D>",
            },
        ],
    }
