import logging

from mil_robogym.data_collection.topic_warnings import warn_for_unhelpful_topics


def test_warns_on_always_warn_topics():
    warnings = warn_for_unhelpful_topics(["/clock", "tf", "/tf_static"])
    categories = {w["category"] for w in warnings}
    topics = {w["topic"] for w in warnings}
    assert categories == {"system_topic"}
    assert topics == {"/clock", "tf", "/tf_static"}


def test_warns_on_raw_sensor_stream():
    warnings = warn_for_unhelpful_topics(["/camera/image_raw"])
    assert len(warnings) == 1
    assert warnings[0]["category"] == "raw_sensor_stream"


def test_warns_on_sim_internal():
    warnings = warn_for_unhelpful_topics(["/gazebo/world/pose/info"])
    assert len(warnings) == 1
    assert warnings[0]["category"] == "sim_internal"


def test_warns_on_debug_or_stats():
    warnings = warn_for_unhelpful_topics(["/statistics"])
    assert len(warnings) == 1
    assert warnings[0]["category"] == "debug_or_stats"


def test_safe_keywords_suppress_warning():
    warnings = warn_for_unhelpful_topics(["/camera/processed"])
    assert warnings == []


def test_logging_option_emits_warning(caplog):
    caplog.set_level(logging.WARNING)
    warn_for_unhelpful_topics(["/statistics"], log=True)
    assert any("Topic warning" in record.message for record in caplog.records)
