from __future__ import annotations

from typing import TypedDict


class TopicWarningConfig(TypedDict):
    always_warn: set[str]
    safe_keywords: set[str]
    warn_rules: dict[str, list[str]]
    warn_order: tuple[str, ...]


_DEFAULT_TOPIC_WARNING_CONFIG: TopicWarningConfig = {
    "always_warn": {
        "clock",
        "tf",
        "tf_static",
    },
    "safe_keywords": {
        "processed",
        "filtered",
        "fused",
        "state",
        "estimate",
        "est",
        "odom",
        "odometry",
        "pose",
        "cmd",
        "command",
        "control",
        "setpoint",
        "target",
        "ref",
        "trajectory",
    },
    "warn_rules": {
        "raw_sensor_stream": [
            "image_raw",
            "camera",
            "depth",
            "rgb",
            "point_cloud",
            "points",
            "lidar",
            "laser",
            "scan",
            "raw",
            "sensor",
            "imu",
            "gps",
            "mag",
        ],
        "sim_internal": [
            "ground_truth",
            "truth",
            "model",
            "link_states",
            "joint_states",
            "pose/info",
            "world",
            "gz",
            "gazebo",
        ],
        "debug_or_stats": [
            "debug",
            "statistics",
            "stats",
            "diagnostics",
            "perf",
            "profile",
        ],
        "system_topic": [
            "time",
            "clock",
            "event",
        ],
    },
    "warn_order": (
        "system_topic",
        "sim_internal",
        "debug_or_stats",
        "raw_sensor_stream",
    ),
}


def load_topic_warning_config() -> TopicWarningConfig:
    """
    Return a mutable copy of the topic warning rules.
    """
    return {
        "always_warn": set(_DEFAULT_TOPIC_WARNING_CONFIG["always_warn"]),
        "safe_keywords": set(_DEFAULT_TOPIC_WARNING_CONFIG["safe_keywords"]),
        "warn_rules": {
            category: list(keywords)
            for category, keywords in _DEFAULT_TOPIC_WARNING_CONFIG[
                "warn_rules"
            ].items()
        },
        "warn_order": tuple(_DEFAULT_TOPIC_WARNING_CONFIG["warn_order"]),
    }
