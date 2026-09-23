from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import numpy as np
import torch
import torch.nn as nn

from mil_robogym.data_collection.types import (
    RoboGymProjectYaml,
    SupportedNonNumericDataType,
)

from .deep_set import DeepSet
from .image_encoder import CNNEncoder
from .utils import load_file

_PREPROCESSOR_BUNDLE_VERSION = 1
_DEFAULT_IMAGE_OUTPUT_DIM = 128
_DEFAULT_DEEP_SET_OUTPUT_DIM = 32
_DEFAULT_IMAGE_RESIZE_SHAPE = (224, 224)


@dataclass(frozen=True, slots=True)
class ExternalEncoderSpec:
    feature_name: str
    data_type: SupportedNonNumericDataType
    ros_type: str
    output_dim: int
    resize_shape: tuple[int, int] | None = None

    def to_dict(self) -> dict[str, object]:
        payload: dict[str, object] = {
            "feature_name": self.feature_name,
            "data_type": self.data_type,
            "ros_type": self.ros_type,
            "output_dim": self.output_dim,
        }
        if self.resize_shape is not None:
            payload["resize_shape"] = list(self.resize_shape)
        return payload

    @classmethod
    def from_dict(cls, payload: dict[str, object]) -> ExternalEncoderSpec:
        feature_name = payload.get("feature_name")
        data_type = payload.get("data_type")
        ros_type = payload.get("ros_type")
        output_dim = payload.get("output_dim")

        if not isinstance(feature_name, str) or not feature_name.strip():
            raise ValueError(
                "External encoder feature_name must be a non-empty string.",
            )
        if data_type not in {"image", "unordered_set"}:
            raise ValueError("External encoder data_type is invalid.")
        if not isinstance(ros_type, str) or not ros_type.strip():
            raise ValueError("External encoder ros_type must be a non-empty string.")
        if not isinstance(output_dim, int) or output_dim <= 0:
            raise ValueError("External encoder output_dim must be a positive int.")

        resize_shape_raw = payload.get("resize_shape")
        resize_shape = _normalize_resize_shape(resize_shape_raw)

        return cls(
            feature_name=feature_name.strip(),
            data_type=cast(SupportedNonNumericDataType, data_type),
            ros_type=ros_type.strip(),
            output_dim=output_dim,
            resize_shape=resize_shape,
        )


class ObservationPreprocessor(nn.Module):
    """Encode raw abstract observation fields into policy-ready float tensors."""

    def __init__(
        self,
        *,
        numeric_feature_names: list[str],
        encoder_specs: list[ExternalEncoderSpec],
        encoders: list[nn.Module] | None = None,
    ) -> None:
        super().__init__()

        if encoders is not None and len(encoders) != len(encoder_specs):
            raise ValueError("encoders and encoder_specs must have matching lengths.")

        self.numeric_feature_names = tuple(str(name) for name in numeric_feature_names)
        self.encoder_specs = tuple(encoder_specs)
        self.encoders = nn.ModuleList(
            (
                encoders
                if encoders is not None
                else [_create_encoder_from_spec(spec) for spec in encoder_specs]
            ),
        )

    @classmethod
    def from_project(
        cls,
        project: RoboGymProjectYaml,
    ) -> ObservationPreprocessor | None:
        encoder_specs = build_external_encoder_specs(project)
        if not encoder_specs:
            return None

        tensor_spec = project.get("tensor_spec")
        if not isinstance(tensor_spec, dict):
            raise ValueError("Project tensor_spec is required to build a preprocessor.")

        input_features = tensor_spec.get("input_features")
        if not isinstance(input_features, list):
            raise ValueError("Project tensor_spec.input_features must be a list.")

        return cls(
            numeric_feature_names=[str(feature) for feature in input_features],
            encoder_specs=encoder_specs,
        )

    @property
    def numeric_feature_count(self) -> int:
        return len(self.numeric_feature_names)

    @property
    def has_external_encoders(self) -> bool:
        return bool(self.encoder_specs)

    @property
    def raw_input_size(self) -> int:
        return self.numeric_feature_count + len(self.encoder_specs)

    @property
    def encoded_input_size(self) -> int:
        return self.numeric_feature_count + sum(
            spec.output_dim for spec in self.encoder_specs
        )

    def save(self, path: str) -> None:
        torch.save(self._build_bundle(), path)

    @classmethod
    def load(cls, path: Path | str) -> ObservationPreprocessor:
        bundle = torch.load(Path(path), map_location="cpu")
        if not isinstance(bundle, dict):
            raise ValueError("Observation preprocessor bundle must be a mapping.")

        version = bundle.get("version")
        if version != _PREPROCESSOR_BUNDLE_VERSION:
            raise ValueError(
                "Observation preprocessor bundle version is unsupported: "
                f"{version!r}",
            )

        numeric_feature_names = bundle.get("numeric_feature_names")
        encoder_payloads = bundle.get("encoders")
        encoder_state_dicts = bundle.get("encoder_state_dicts")

        if not isinstance(numeric_feature_names, list):
            raise ValueError(
                "Observation preprocessor bundle is missing numeric_feature_names.",
            )
        if not isinstance(encoder_payloads, list) or not isinstance(
            encoder_state_dicts,
            list,
        ):
            raise ValueError(
                "Observation preprocessor bundle is missing encoder metadata.",
            )
        if len(encoder_payloads) != len(encoder_state_dicts):
            raise ValueError(
                "Observation preprocessor bundle encoder metadata is inconsistent.",
            )

        encoder_specs = [
            ExternalEncoderSpec.from_dict(dict(payload))
            for payload in encoder_payloads
            if isinstance(payload, dict)
        ]
        if len(encoder_specs) != len(encoder_payloads):
            raise ValueError(
                "Observation preprocessor bundle contains invalid encoder entries.",
            )

        preprocessor = cls(
            numeric_feature_names=[str(name) for name in numeric_feature_names],
            encoder_specs=encoder_specs,
        )

        for encoder, state_dict in zip(
            preprocessor.encoders,
            encoder_state_dicts,
            strict=False,
        ):
            if not isinstance(state_dict, dict):
                raise ValueError(
                    "Observation preprocessor bundle contains an invalid state_dict.",
                )
            encoder.load_state_dict(state_dict)

        return preprocessor

    def encode_state(
        self,
        state: list[object] | tuple[object, ...] | np.ndarray,
    ) -> np.ndarray:
        with torch.no_grad():
            encoded = self.encode_batch([state])
        return encoded[0].detach().cpu().numpy()

    def encode_batch(
        self,
        states: list[list[object] | tuple[object, ...] | np.ndarray],
        *,
        device: torch.device | str | None = None,
    ) -> torch.Tensor:
        rows = [self._coerce_state_items(state) for state in states]
        target_device = self._resolve_device(device)

        if not rows:
            return torch.zeros(
                (0, self.encoded_input_size),
                dtype=torch.float32,
                device=target_device,
            )

        if self.has_external_encoders:
            self.to(target_device)

        if self.numeric_feature_count > 0:
            numeric_array = np.asarray(
                [row[: self.numeric_feature_count] for row in rows],
                dtype=np.float32,
            )
            numeric_tensor = torch.as_tensor(
                numeric_array,
                dtype=torch.float32,
                device=target_device,
            )
        else:
            numeric_tensor = torch.zeros(
                (len(rows), 0),
                dtype=torch.float32,
                device=target_device,
            )

        if not self.has_external_encoders:
            return numeric_tensor

        encoded_parts = [numeric_tensor]
        abstract_offset = self.numeric_feature_count

        for encoder_index, encoder in enumerate(self.encoders):
            raw_values = [
                self._load_raw_value(row[abstract_offset + encoder_index])
                for row in rows
            ]
            embeddings = encoder(raw_values)
            if not isinstance(embeddings, torch.Tensor):
                embeddings = torch.as_tensor(embeddings)
            embeddings = embeddings.to(device=target_device, dtype=torch.float32)
            if embeddings.ndim == 1:
                embeddings = embeddings.unsqueeze(0)
            if embeddings.shape[0] != len(rows):
                raise ValueError(
                    "Encoded batch size does not match the raw observation batch size.",
                )
            encoded_parts.append(embeddings)

        return torch.cat(encoded_parts, dim=1)

    def _build_bundle(self) -> dict[str, object]:
        return {
            "version": _PREPROCESSOR_BUNDLE_VERSION,
            "numeric_feature_names": list(self.numeric_feature_names),
            "encoders": [spec.to_dict() for spec in self.encoder_specs],
            "encoder_state_dicts": [encoder.state_dict() for encoder in self.encoders],
        }

    def _coerce_state_items(
        self,
        state: list[object] | tuple[object, ...] | np.ndarray,
    ) -> list[object]:
        if isinstance(state, np.ndarray):
            items = state.tolist() if state.ndim > 1 else list(state)
        else:
            items = list(state)

        if len(items) != self.raw_input_size:
            raise ValueError(
                "Raw observation does not match the saved preprocessor input size: "
                f"{len(items)} != {self.raw_input_size}",
            )

        return items

    def _load_raw_value(self, value: object) -> object:
        if isinstance(value, Path):
            return load_file(str(value))
        if isinstance(value, str) and value.endswith((".jpg", ".json")):
            return load_file(value)
        return value

    def _resolve_device(self, device: torch.device | str | None) -> torch.device:
        if device is not None:
            return torch.device(device)

        for parameter in self.parameters():
            return parameter.device
        for buffer in self.buffers():
            return buffer.device
        return torch.device("cpu")


def build_external_encoder_specs(
    project: RoboGymProjectYaml,
) -> list[ExternalEncoderSpec]:
    encoder_specs: list[ExternalEncoderSpec] = []
    input_non_numeric_topics = project.get("input_non_numeric_topics", {})

    for topic, data_list in input_non_numeric_topics.items():
        for data in data_list:
            data_type = data["data_type"]
            field_path = data["field_path"]

            if data_type == "image":
                feature_name = topic
                output_dim = int(data.get("output_dim", _DEFAULT_IMAGE_OUTPUT_DIM))
                resize_shape = _normalize_resize_shape(
                    data.get("resize_shape", _DEFAULT_IMAGE_RESIZE_SHAPE),
                )
            elif data_type == "unordered_set":
                feature_name = f"{topic}:{field_path}"
                output_dim = int(data.get("output_dim", _DEFAULT_DEEP_SET_OUTPUT_DIM))
                resize_shape = None
            else:
                raise ValueError(f"Unsupported abstract data_type: {data_type}")

            if output_dim <= 0:
                raise ValueError("External encoder output_dim must be positive.")

            encoder_specs.append(
                ExternalEncoderSpec(
                    feature_name=feature_name,
                    data_type=data_type,
                    ros_type=data["ros_type"],
                    output_dim=output_dim,
                    resize_shape=resize_shape,
                ),
            )

    return encoder_specs


def _normalize_resize_shape(
    value: object,
) -> tuple[int, int] | None:
    if value is None:
        return None
    if (
        isinstance(value, tuple)
        and len(value) == 2
        and all(isinstance(size, int) and size > 0 for size in value)
    ):
        return value
    if (
        isinstance(value, list)
        and len(value) == 2
        and all(isinstance(size, int) and size > 0 for size in value)
    ):
        return (value[0], value[1])
    raise ValueError("resize_shape must contain exactly two positive ints.")


def _create_encoder_from_spec(spec: ExternalEncoderSpec) -> nn.Module:
    if spec.data_type == "image":
        return CNNEncoder(
            latent_dim=spec.output_dim,
            resize_shape=spec.resize_shape or _DEFAULT_IMAGE_RESIZE_SHAPE,
        )

    if spec.data_type == "unordered_set":
        return DeepSet(
            obj=_create_deepset_reference_object(spec.ros_type),
            output_shape=spec.output_dim,
        )

    raise ValueError(f"Unsupported external encoder type: {spec.data_type}")


def _create_deepset_reference_object(ros_type: str) -> Any:
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    inner_type = ros_type
    if ros_type.startswith("sequence<") and ros_type.endswith(">"):
        inner_type = ros_type[len("sequence<") : -1]

    msg_class = get_message(inner_type)
    return message_to_ordereddict(msg_class())
