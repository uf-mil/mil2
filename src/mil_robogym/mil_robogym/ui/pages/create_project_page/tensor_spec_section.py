from __future__ import annotations

import tkinter as tk
from typing import Callable, Literal

from mil_robogym.data_collection.build_tensor_spec import build_tensor_spec
from mil_robogym.data_collection.types import RoboGymProjectYaml, RoboGymTensorSpec

TopicListType = Literal["input", "output"]
TensorSpecTopicSignature = tuple[
    tuple[tuple[str, tuple[str, ...]], ...],
    tuple[tuple[str, tuple[str, ...]], ...],
]


class TensorSpecSection:
    """Render tensor-spec controls and handle tensor-spec computation logic."""

    def __init__(
        self,
        parent: tk.Widget,
        get_project_config: Callable[[], RoboGymProjectYaml],
        get_selected_input_topics: Callable[[], list[str]],
        get_selected_output_topics: Callable[[], list[str]],
        get_selected_input_topic_subtopics: Callable[[], dict[str, list[str]]],
        get_selected_output_topic_subtopics: Callable[[], dict[str, list[str]]],
        show_error: Callable[[str], None],
        hide_error: Callable[[], None],
    ) -> None:
        self._get_project_config = get_project_config
        self._get_selected_input_topics = get_selected_input_topics
        self._get_selected_output_topics = get_selected_output_topics
        self._get_selected_input_topic_subtopics = get_selected_input_topic_subtopics
        self._get_selected_output_topic_subtopics = get_selected_output_topic_subtopics
        self._show_error = show_error
        self._hide_error = hide_error

        self._tensor_spec: RoboGymTensorSpec | None = None
        self._tensor_spec_topic_signature: TensorSpecTopicSignature | None = None

        self.compute_tensor_spec_button = tk.Button(
            parent,
            text="Compute Tensor Spec",
            command=self.compute_tensor_spec,
            bg="#ECECEC",
            activebackground="#DFDFDF",
            fg="black",
            relief="solid",
            bd=1,
            font=("Arial", 14),
            padx=8,
            pady=5,
        )
        self.compute_tensor_spec_button.grid(
            row=10,
            column=0,
            columnspan=2,
            sticky="nsew",
            padx=(14, 8),
            pady=(2, 6),
        )

        self.tensor_spec_status_var = tk.StringVar(value="Tensor Spec: Not computed")
        self.tensor_spec_status_label = tk.Label(
            parent,
            textvariable=self.tensor_spec_status_var,
            bg="#DADADA",
            fg="#444444",
            font=("Arial", 12),
            anchor="w",
        )
        self.tensor_spec_status_label.grid(
            row=10,
            column=2,
            columnspan=4,
            sticky="nsew",
            padx=(8, 14),
            pady=(2, 6),
        )

    def _current_tensor_spec_signature(self) -> TensorSpecTopicSignature:
        selected_input_topic_subtopics = self._get_selected_input_topic_subtopics()
        selected_output_topic_subtopics = self._get_selected_output_topic_subtopics()
        return (
            tuple(
                (topic, tuple(fields))
                for topic, fields in selected_input_topic_subtopics.items()
            ),
            tuple(
                (topic, tuple(fields))
                for topic, fields in selected_output_topic_subtopics.items()
            ),
        )

    def _selected_subtopic_fields(
        self,
        list_type: TopicListType,
    ) -> dict[str, set[str]]:
        topic_subtopics = (
            self._get_selected_input_topic_subtopics()
            if list_type == "input"
            else self._get_selected_output_topic_subtopics()
        )
        return {topic: set(subtopics) for topic, subtopics in topic_subtopics.items()}

    def _apply_subtopic_filters_to_tensor_spec(
        self,
        tensor_spec: RoboGymTensorSpec,
    ) -> RoboGymTensorSpec:
        filtered: RoboGymTensorSpec = {
            "input_features": list(tensor_spec["input_features"]),
            "output_features": list(tensor_spec["output_features"]),
            "input_dim": tensor_spec["input_dim"],
            "output_dim": tensor_spec["output_dim"],
        }

        input_filters = self._selected_subtopic_fields("input")
        output_filters = self._selected_subtopic_fields("output")

        def _feature_kept(feature: str, allowed_fields: dict[str, set[str]]) -> bool:
            if ":" not in feature:
                return True
            topic, field = feature.split(":", 1)
            allowed = allowed_fields.get(topic)
            if allowed is None:
                return True
            return field in allowed

        if input_filters:
            filtered["input_features"] = [
                feature
                for feature in filtered["input_features"]
                if _feature_kept(feature, input_filters)
            ]
            filtered["input_dim"] = len(filtered["input_features"])

        if output_filters:
            filtered["output_features"] = [
                feature
                for feature in filtered["output_features"]
                if _feature_kept(feature, output_filters)
            ]
            filtered["output_dim"] = len(filtered["output_features"])

        return filtered

    def _set_tensor_spec_status(self, text: str, *, fg: str) -> None:
        self.tensor_spec_status_var.set(text)
        self.tensor_spec_status_label.configure(fg=fg)

    def invalidate_tensor_spec(self) -> None:
        self._tensor_spec = None
        self._tensor_spec_topic_signature = None
        self._set_tensor_spec_status("Tensor Spec: Not computed", fg="#444444")

    def compute_tensor_spec(self) -> RoboGymTensorSpec | None:
        selected_input_topics = self._get_selected_input_topics()
        selected_output_topics = self._get_selected_output_topics()
        if not selected_input_topics or not selected_output_topics:
            self.invalidate_tensor_spec()
            self._show_error(
                "Select at least one input and output topic before computing tensor spec.",
            )
            return None

        project_cfg = self._get_project_config()

        try:
            tensor_spec = build_tensor_spec(project_cfg)
        except (RuntimeError, ValueError, KeyError) as exc:
            self.invalidate_tensor_spec()
            self._set_tensor_spec_status("Tensor Spec: Failed", fg="#AA2222")
            self._show_error(str(exc) or type(exc).__name__)
            return None

        tensor_spec = self._apply_subtopic_filters_to_tensor_spec(tensor_spec)
        if selected_input_topics and tensor_spec["input_dim"] == 0:
            self.invalidate_tensor_spec()
            self._set_tensor_spec_status("Tensor Spec: Failed", fg="#AA2222")
            self._show_error(
                "No numeric input subtopics selected. Expand input topics and select at least one numeric subtopic.",
            )
            return None

        if selected_output_topics and tensor_spec["output_dim"] == 0:
            self.invalidate_tensor_spec()
            self._set_tensor_spec_status("Tensor Spec: Failed", fg="#AA2222")
            self._show_error(
                "No numeric output subtopics selected. Expand output topics and select at least one numeric subtopic.",
            )
            return None

        self._tensor_spec = tensor_spec
        self._tensor_spec_topic_signature = self._current_tensor_spec_signature()
        self._set_tensor_spec_status(
            (
                "Tensor Spec: "
                f"in={tensor_spec['input_dim']} "
                f"out={tensor_spec['output_dim']}"
            ),
            fg="#222222",
        )
        self._hide_error()
        return tensor_spec

    def get_tensor_spec_for_current_selection(self) -> RoboGymTensorSpec | None:
        if (
            self._tensor_spec is not None
            and self._tensor_spec_topic_signature
            == self._current_tensor_spec_signature()
        ):
            return self._tensor_spec

        return None
