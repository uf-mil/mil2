"""Tests for topic warning categorization and suppression rules."""

import logging

from mil_robogym.data_collection.topic_warnings import warn_for_unhelpful_topics


def test_warns_on_always_warn_topics():
    """Flags clock and tf topics as system topics."""
    warnings = warn_for_unhelpful_topics(["/clock", "tf", "/tf_static"])
    categories = {w["category"] for w in warnings}
    topics = {w["topic"] for w in warnings}
    assert categories == {"system_topic"}
    assert topics == {"/clock", "tf", "/tf_static"}


def test_warns_on_raw_sensor_stream():
    """Flags camera raw streams as raw sensor inputs."""
    warnings = warn_for_unhelpful_topics(["/camera/image_raw"])
    assert len(warnings) == 1
    assert warnings[0]["category"] == "raw_sensor_stream"


def test_warns_on_sim_internal():
    """Flags gazebo internals as simulation-only topics."""
    warnings = warn_for_unhelpful_topics(["/gazebo/world/pose/info"])
    assert len(warnings) == 1
    assert warnings[0]["category"] == "sim_internal"


def test_warns_on_debug_or_stats():
    """Flags statistics-style topics as debug or stats."""
    warnings = warn_for_unhelpful_topics(["/statistics"])
    assert len(warnings) == 1
    assert warnings[0]["category"] == "debug_or_stats"


def test_safe_keywords_suppress_warning():
    """Suppresses raw-sensor warnings when a safe keyword is present."""
    warnings = warn_for_unhelpful_topics(["/camera/processed"])
    assert warnings == []


def test_logging_option_emits_warning(caplog):
    """Emits log records when warning logging is enabled."""
    caplog.set_level(logging.WARNING)
    warn_for_unhelpful_topics(["/statistics"], log=True)
    assert any("Topic warning" in record.message for record in caplog.records)
