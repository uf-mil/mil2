from __future__ import annotations

from collections import Counter

from .sample_input_topics import sample_topics
from .types import RoboGymProject, RoboGymTensorSpec


def _canonical_topic_name(topic: str) -> str:
    """Normalize a topic name by trimming whitespace and leading slashes."""
    return topic.strip().lstrip("/")


def _validate_unique_topics(topics: list[str], *, side: str) -> None:
    """
    Validate that a topic list has no duplicates after normalization.

    Normalizes each topic by trimming whitespace and a leading slash so
    '/foo' and 'foo' are treated as the same topic.

    Raises:
      ValueError if duplicate normalized topic names are found.
    """
    normalized = [_canonical_topic_name(topic) for topic in topics]
    counts = Counter(name for name in normalized if name)
    duplicates = sorted(name for name, count in counts.items() if count > 1)
    if duplicates:
        raise ValueError(
            f"Duplicate normalized {side} topics are not allowed: {duplicates}",
        )


def _is_numeric_tensor_value(value: object) -> bool:
    """
    Check whether a flattened field can be represented as a scalar tensor.

    Returns:
      True for `bool`, `int`, and `float`; otherwise False.
    """
    return isinstance(value, (bool, int, float))


def _collect_features(
    sampled_topics: dict[str, dict[str, object]],
    *,
    side: str,
    strict_numeric: bool,
) -> tuple[list[str], dict[str, list[str]]]:
    """
    Convert sampled topic fields into stable feature names.

    Builds features in deterministic order:
      - topic order from the sampled topic mapping
      - field order from each flattened topic mapping

    Non-numeric leaves are ignored by default and tracked per topic.

    Raises:
      RuntimeError if `strict_numeric` is True and a non-numeric field appears.

    Returns:
      A tuple of `(features, ignored_fields_by_topic)`.
    """
    features: list[str] = []
    ignored: dict[str, list[str]] = {}

    for topic, flattened in sampled_topics.items():
        for field, value in flattened.items():
            if _is_numeric_tensor_value(value):
                features.append(f"{topic}:{field}")
                continue

            if strict_numeric:
                raise RuntimeError(
                    "Found non-numeric field while building tensor spec for "
                    f"{side} topics: topic='{topic}', field='{field}', "
                    f"value_type='{type(value).__name__}'.",
                )

            ignored.setdefault(topic, []).append(field)

    return features, ignored


def _dedupe_preserve_order(values: list[str]) -> list[str]:
    """
    Remove duplicate strings while preserving first-seen order.

    Returns:
      A list with duplicate string values removed.
    """
    deduped: list[str] = []
    seen: set[str] = set()
    for value in values:
        if value in seen:
            continue
        seen.add(value)
        deduped.append(value)
    return deduped


def build_tensor_spec(
    project: RoboGymProject,
    *,
    timeout_s: float = 2.0,
    strict_numeric: bool = False,
) -> RoboGymTensorSpec:
    """
    Build a deterministic tensor specification for selected input/output topics.

    Reads:
      - Topic schemas for selected input/output topics using `sample_topics`.

    Builds:
      - `input_features` and `output_features` in `<topic>:<field>` format.
      - `input_dim` and `output_dim` from numeric feature counts.
      - ignored non-numeric fields by topic when `strict_numeric` is False.

    Raises:
      ValueError if `timeout_s` is not positive.
      ValueError if normalized duplicate topic names are selected.
      RuntimeError if topic sampling fails.
      RuntimeError if `strict_numeric` is True and non-numeric fields are found.
      RuntimeError if selected inputs or outputs yield zero numeric features.

    Returns:
      A tensor specification dictionary for PyTorch input/output sizing.
    """
    if timeout_s <= 0:
        raise ValueError("timeout_s must be positive.")

    input_topics = list(project["input_topics"])
    output_topics = list(project["output_topics"])

    _validate_unique_topics(input_topics, side="input")
    _validate_unique_topics(output_topics, side="output")

    combined_topics = _dedupe_preserve_order([*input_topics, *output_topics])
    sampled_all = sample_topics(combined_topics, timeout_s=timeout_s)

    sampled_inputs = {topic: sampled_all[topic] for topic in input_topics}
    sampled_outputs = {topic: sampled_all[topic] for topic in output_topics}

    input_features, ignored_inputs = _collect_features(
        sampled_inputs,
        side="input",
        strict_numeric=strict_numeric,
    )
    output_features, ignored_outputs = _collect_features(
        sampled_outputs,
        side="output",
        strict_numeric=strict_numeric,
    )

    if input_topics and not input_features:
        raise RuntimeError(
            "Input topics were selected, but no numeric tensor features were "
            "derived. Choose topics/fields with numeric values.",
        )

    if output_topics and not output_features:
        raise RuntimeError(
            "Output topics were selected, but no numeric tensor features were "
            "derived. Choose topics/fields with numeric values.",
        )

    return {
        "input_features": input_features,
        "output_features": output_features,
        "input_dim": len(input_features),
        "output_dim": len(output_features),
        "ignored_input_features": ignored_inputs,
        "ignored_output_features": ignored_outputs,
    }
