from __future__ import annotations

from collections.abc import Mapping, Sequence

from .types import NonNumericTopicFieldSelection


def filter_populated_non_numeric_topic_fields(
    topic_fields: Mapping[str, Sequence[NonNumericTopicFieldSelection]],
) -> dict[str, list[NonNumericTopicFieldSelection]]:
    """Return only topic entries that still have at least one selected field."""

    filtered: dict[str, list[NonNumericTopicFieldSelection]] = {}
    for topic, fields in topic_fields.items():
        normalized_fields = [dict(field) for field in fields]
        if normalized_fields:
            filtered[topic] = normalized_fields
    return filtered
