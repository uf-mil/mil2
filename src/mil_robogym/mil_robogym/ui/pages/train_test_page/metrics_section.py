from __future__ import annotations

from mil_robogym.ui.components.series_plots_section import SeriesPlotsSection

DEFAULT_METRIC_ORDER = (
    "reward_mean",
    "reward_std",
    "disc_loss",
    "disc_kl",
    "disc_beta",
)


class MetricsSection(SeriesPlotsSection):
    """Train/test metrics panel built on the shared multi-plot section."""

    def __init__(self, parent) -> None:
        super().__init__(
            parent,
            title="Metrics",
            preferred_metric_order=DEFAULT_METRIC_ORDER,
            section_bg="#DADADA",
            content_bg="#F2F2F2",
            empty_state_message="No training metrics available.",
            pending_data_message="Waiting for training metrics...",
            missing_file_message="No metrics file found.",
            empty_file_message="Metrics file is empty.",
        )
        self.container.grid(
            row=1,
            column=2,
            columnspan=4,
            sticky="nsew",
            padx=(8, 14),
            pady=(0, 8),
        )
