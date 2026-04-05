from __future__ import annotations

from collections.abc import Sequence

from mil_robogym.ui.components.series_plots_section import SeriesPlotsSection

DEFAULT_METRIC_ORDER = (
    "reward_mean",
    "reward_std",
    "disc_loss",
    "disc_kl",
    "disc_beta",
    "gen_rollout_ep_len_mean",
    "gen_rollout_ep_rew_mean",
    "gen_rollout_ep_rew_wrapped_mean",
    "gen_time_fps",
    "gen_time_iterations",
    "gen_time_time_elapsed",
    "gen_time_total_timesteps",
    "gen_train_explained_variance",
    "gen_train_is_line_search_success",
    "gen_train_kl_divergence_loss",
    "gen_train_learning_rate",
    "gen_train_n_updates",
    "gen_train_policy_objective",
    "gen_train_std",
    "gen_train_value_loss",
)
DEFAULT_VISIBLE_METRICS = DEFAULT_METRIC_ORDER[:5]


class MetricsSection(SeriesPlotsSection):
    """Train/Test metrics panel backed by the shared multi-plot widget."""

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

    def _default_selected_metrics(self, available_metrics: Sequence[str]) -> list[str]:
        preferred_metrics = [
            metric_name
            for metric_name in DEFAULT_VISIBLE_METRICS
            if metric_name in available_metrics
        ]
        return preferred_metrics if preferred_metrics else list(available_metrics)
