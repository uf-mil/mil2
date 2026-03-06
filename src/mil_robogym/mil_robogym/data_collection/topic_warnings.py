from __future__ import annotations

import logging
import re

from .topic_warning_config import load_topic_warning_config
from .types import TopicWarning

_WARN_CONFIG = load_topic_warning_config()
ALWAYS_WARN = _WARN_CONFIG["always_warn"]
SAFE_KEYWORDS = _WARN_CONFIG["safe_keywords"]
WARN_RULES = _WARN_CONFIG["warn_rules"]
WARN_ORDER = _WARN_CONFIG["warn_order"]

TOKEN_SPLIT_RE = re.compile(r"[\/._-]+")


def _tokenize(topic: str) -> list[str]:
    return [t for t in TOKEN_SPLIT_RE.split(topic) if t]


def _keyword_present(normalized: str, tokens: list[str], keyword: str) -> bool:
    if any(sep in keyword for sep in ("_", "/", "-", ".")):
        return keyword in normalized
    return keyword in tokens


def _matching_keywords(
    normalized: str,
    tokens: list[str],
    keywords: list[str] | set[str],
) -> list[str]:
    matches = []
    for keyword in keywords:
        if _keyword_present(normalized, tokens, keyword):
            matches.append(keyword)
    return matches


def warn_for_unhelpful_topics(
    topics: list[str],
    *,
    log: bool = False,
) -> list[TopicWarning]:
    """
    Identify topics that likely represent raw sensor streams or other
    sim-only/noisy data sources that are not ideal for training.

    :param topics: Topic names to evaluate.
    :param log: If True, log warnings via logging.warning.
    :return: List of warning records for flagged topics.
    """
    warnings: list[TopicWarning] = []

    for topic in topics:
        if not topic:
            continue
        normalized = topic.strip().lower()
        tokens = _tokenize(normalized)
        safe_matches = _matching_keywords(normalized, tokens, SAFE_KEYWORDS)

        # Always-warn exact topic names (ignores leading slash).
        if normalized.lstrip("/") in ALWAYS_WARN:
            warning: TopicWarning = {
                "topic": topic,
                "reason": "system_topic",
                "matched": [normalized.lstrip("/")],
                "category": "system_topic",
            }
            warnings.append(warning)
            if log:
                logging.warning("Topic warning: %s", warning)
            continue

        for category in WARN_ORDER:
            keywords = WARN_RULES[category]
            matches = _matching_keywords(normalized, tokens, keywords)
            if matches:
                # Safe keywords intentionally suppress only raw sensor streams.
                if category == "raw_sensor_stream" and safe_matches:
                    break
                warning = {
                    "topic": topic,
                    "reason": category,
                    "matched": matches,
                    "category": category,
                }
                warnings.append(warning)
                if log:
                    logging.warning("Topic warning: %s", warning)
                break

    return warnings
