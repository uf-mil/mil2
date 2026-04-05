from __future__ import annotations

import re
from dataclasses import dataclass

from .sample_input_topics import resolve_topic_message_types
from .types import SupportedNonNumericDataType

_SEQUENCE_TYPE_RE = re.compile(
    r"^(?:bounded_)?sequence<\s*(.+?)(?:\s*,\s*\d+)?\s*>$",
)
_FIXED_ARRAY_TYPE_RE = re.compile(r"^(.+)\[(\d+)\]$")

_NUMERIC_SCALAR_TYPES = {
    "bool",
    "byte",
    "char",
    "float",
    "float32",
    "float64",
    "double",
    "int8",
    "uint8",
    "int16",
    "uint16",
    "int32",
    "uint32",
    "int64",
    "uint64",
    "long",
    "ulong",
    "long long",
    "unsigned long long",
    "short",
    "unsigned short",
    "octet",
}
_STRING_SCALAR_TYPES = {"string", "wstring"}
_IMAGE_MESSAGE_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/Image"}


@dataclass(frozen=True)
class SupportedNonNumericFieldOption:
    field_path: str
    label: str
    data_type: SupportedNonNumericDataType
    ros_type: str


def _normalize_message_type(type_name: str) -> str:
    normalized = type_name.strip()
    if "/" not in normalized:
        return normalized
    if "/msg/" in normalized:
        return normalized
    package, name = normalized.split("/", 1)
    return f"{package}/msg/{name}"


def _normalize_type_expr(type_expr: str) -> str:
    stripped = type_expr.strip()

    fixed_array_match = _FIXED_ARRAY_TYPE_RE.fullmatch(stripped)
    if fixed_array_match is not None:
        element_type, count = fixed_array_match.groups()
        return f"{_normalize_type_expr(element_type)}[{count}]"

    sequence_match = _SEQUENCE_TYPE_RE.fullmatch(stripped)
    if sequence_match is not None:
        return f"sequence<{_normalize_type_expr(sequence_match.group(1))}>"

    if "<=" in stripped:
        stripped = stripped.split("<=", 1)[0].strip()

    return _normalize_message_type(stripped)


def _is_image_message_type(type_expr: str) -> bool:
    return _normalize_type_expr(type_expr) in _IMAGE_MESSAGE_TYPES


def is_image_message_type(type_expr: str) -> bool:
    return _is_image_message_type(type_expr)


def _sequence_element_type(type_expr: str) -> str | None:
    match = _SEQUENCE_TYPE_RE.fullmatch(type_expr.strip())
    if match is None:
        return None
    return _normalize_type_expr(match.group(1))


def _fixed_array_element_type(type_expr: str) -> str | None:
    match = _FIXED_ARRAY_TYPE_RE.fullmatch(type_expr.strip())
    if match is None:
        return None
    return _normalize_type_expr(match.group(1))


def _is_numeric_scalar_type(type_expr: str) -> bool:
    return _normalize_type_expr(type_expr) in _NUMERIC_SCALAR_TYPES


def _is_string_scalar_type(type_expr: str) -> bool:
    return _normalize_type_expr(type_expr) in _STRING_SCALAR_TYPES


def _load_message_class(message_type: str):
    try:
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "rosidl_runtime_py is required to inspect ROS 2 topic schemas.",
        ) from exc

    try:
        return get_message(_normalize_message_type(message_type))
    except Exception as exc:
        raise RuntimeError(
            f"Failed to resolve ROS 2 message class for type '{message_type}'.",
        ) from exc


def _image_field_path(prefix: str) -> str:
    return f"{prefix}.data" if prefix else "data"


def _image_label(prefix: str) -> str:
    return prefix if prefix else "data"


def _describe_supported_non_numeric_fields(
    message_type: str,
    *,
    prefix: str,
    active_types: set[str],
) -> list[SupportedNonNumericFieldOption]:
    normalized_type = _normalize_message_type(message_type)
    if normalized_type in active_types:
        return []

    if normalized_type in _IMAGE_MESSAGE_TYPES:
        return [
            SupportedNonNumericFieldOption(
                field_path=_image_field_path(prefix),
                label=_image_label(prefix),
                data_type="image",
                ros_type=normalized_type,
            ),
        ]

    active_types.add(normalized_type)
    try:
        message_class = _load_message_class(normalized_type)
        field_types = message_class.get_fields_and_field_types()

        options: list[SupportedNonNumericFieldOption] = []
        for field_name, field_type in field_types.items():
            field_path = f"{prefix}.{field_name}" if prefix else field_name
            normalized_field_type = _normalize_type_expr(field_type)

            if _is_image_message_type(normalized_field_type):
                options.append(
                    SupportedNonNumericFieldOption(
                        field_path=_image_field_path(field_path),
                        label=field_path,
                        data_type="image",
                        ros_type=_normalize_message_type(normalized_field_type),
                    ),
                )
                continue

            sequence_element_type = _sequence_element_type(normalized_field_type)
            if sequence_element_type is not None:
                if _is_string_scalar_type(sequence_element_type):
                    continue
                options.append(
                    SupportedNonNumericFieldOption(
                        field_path=field_path,
                        label=field_path,
                        data_type="unordered_set",
                        ros_type=normalized_field_type,
                    ),
                )
                continue

            if (
                _is_numeric_scalar_type(normalized_field_type)
                or _is_string_scalar_type(normalized_field_type)
                or _fixed_array_element_type(normalized_field_type) is not None
            ):
                continue

            options.extend(
                _describe_supported_non_numeric_fields(
                    normalized_field_type,
                    prefix=field_path,
                    active_types=active_types,
                ),
            )

        deduped: list[SupportedNonNumericFieldOption] = []
        seen: set[tuple[str, SupportedNonNumericDataType]] = set()
        for option in options:
            key = (option.field_path, option.data_type)
            if key in seen:
                continue
            seen.add(key)
            deduped.append(option)
        return deduped
    finally:
        active_types.discard(normalized_type)


def supported_non_numeric_fields_for_message_type(
    message_type: str,
) -> list[SupportedNonNumericFieldOption]:
    return _describe_supported_non_numeric_fields(
        message_type,
        prefix="",
        active_types=set(),
    )


def supported_non_numeric_fields_for_topic(
    topic: str,
    *,
    timeout_s: float = 2.0,
) -> list[SupportedNonNumericFieldOption]:
    if timeout_s <= 0:
        raise ValueError("timeout_s must be positive.")

    topic_types = resolve_topic_message_types([topic], timeout_s=timeout_s)
    topic_type = topic_types.get(topic)
    if topic_type is None:
        raise RuntimeError(f"Could not resolve ROS 2 message type for topic '{topic}'.")

    return supported_non_numeric_fields_for_message_type(topic_type)
