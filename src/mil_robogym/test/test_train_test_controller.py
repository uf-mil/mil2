"""Tests for the Train/Test controller wiring."""

from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path


class DummyTrainer:
    """Minimal trainer stub for controller tests."""

    raise_on_train = False

    def __init__(self, project, **kwargs):
        self.project = project
        self.settings = kwargs
        self.progress_callback = kwargs.get("progress_callback")
        self.train_calls = 0

    def train(self):
        self.train_calls += 1
        if self.progress_callback is not None:
            self.progress_callback(
                {
                    "type": "metrics_updated",
                    "episode": 1,
                    "num_episodes": 7,
                    "metrics": {
                        "episode": [1.0],
                        "reward_mean": [0.5],
                        "disc_loss": [1.2],
                    },
                },
            )
            self.progress_callback(
                {
                    "type": "agent_saved",
                    "agent_name": "2026_03_26_03_15_pm_ep_0003",
                    "checkpoint_episode": 3,
                    "is_final": False,
                },
            )
        if type(self).raise_on_train:
            raise RuntimeError("training exploded")
        return [Path("/tmp/2026_03_26_03_15_pm_final")]


class DummyView:
    """Records controller interactions without a real Tk widget tree."""

    def __init__(self):
        self.selected_agent_name: str | None = None
        self.terminal_messages: list[str] = []
        self.training_enabled_states: list[bool] = []
        self.refreshed_agent_names: list[str | None] = []
        self.history_refreshes: list[str | None] = []
        self.live_metrics_payloads: list[dict[str, list[float]]] = []
        self.flush_calls = 0
        self.after_calls: list[int] = []
        self._after_id = 0
        self.scheduled_callbacks: dict[str, object] = {}

    def set_terminal_text(self, text: str) -> None:
        self.terminal_messages.append(text)

    def set_training_enabled(self, enabled: bool) -> None:
        self.training_enabled_states.append(enabled)

    def refresh_project_artifacts(
        self,
        preferred_agent_name: str | None = None,
    ) -> None:
        self.refreshed_agent_names.append(preferred_agent_name)
        self.selected_agent_name = preferred_agent_name

    def refresh_history(self, preferred_agent_name: str | None = None) -> None:
        self.history_refreshes.append(preferred_agent_name)

    def show_live_metrics(self, metrics: dict[str, list[float]]) -> None:
        self.live_metrics_payloads.append(metrics)

    def flush_ui_updates(self) -> None:
        self.flush_calls += 1

    def after(self, delay_ms: int, callback) -> str:
        self.after_calls.append(delay_ms)
        self._after_id += 1
        after_id = f"after-{self._after_id}"
        self.scheduled_callbacks[after_id] = callback
        return after_id

    def after_cancel(self, after_id: str) -> None:
        self.scheduled_callbacks.pop(after_id, None)


class DummyApp:
    def __init__(self):
        self.calls: list[tuple[tuple[object, ...], dict[str, object]]] = []

    def show_page(self, *args, **kwargs) -> None:
        self.calls.append((args, kwargs))


def _load_controller_module(monkeypatch):
    trainer_module = types.ModuleType("mil_robogym.vairl.trainer")
    trainer_module.Trainer = DummyTrainer
    monkeypatch.setitem(sys.modules, "mil_robogym.vairl.trainer", trainer_module)
    sys.modules.pop("mil_robogym.ui.pages.train_test_page.train_test_controller", None)
    return importlib.import_module(
        "mil_robogym.ui.pages.train_test_page.train_test_controller",
    )


def test_start_training_refreshes_history_with_saved_agent(monkeypatch):
    """The controller refreshes the UI using the agent saved by training."""
    module = _load_controller_module(monkeypatch)
    view = DummyView()
    app = DummyApp()
    controller = module.TrainTestViewController(view, app)

    controller.set_context(
        {
            "robogym_project": {"name": "Demo Project"},
            "robogym_training": {
                "num_episodes": 7,
                "rollout_steps": 512,
                "save_every": 3,
                "seed": 9,
            },
        },
    )
    controller.start_training()
    controller.wait_for_training_completion(timeout=2)
    controller.process_pending_training_events()

    assert controller.trainer is not None
    assert controller.trainer.settings["num_episodes"] == 7
    assert controller.trainer.settings["rollout_steps"] == 512
    assert controller.trainer.settings["save_every"] == 3
    assert controller.trainer.settings["seed"] == 9
    assert controller.trainer.progress_callback is not None
    assert controller.trainer.train_calls == 1
    assert view.training_enabled_states == [False, True]
    assert view.history_refreshes == ["2026_03_26_03_15_pm_ep_0003"]
    assert view.refreshed_agent_names == ["2026_03_26_03_15_pm_final"]
    assert view.selected_agent_name == "2026_03_26_03_15_pm_final"
    assert "Training agent with saved settings" in view.terminal_messages[0]
    assert "Latest saved agent: 2026_03_26_03_15_pm_final" in view.terminal_messages[-1]
    assert view.live_metrics_payloads[0] == {}
    assert view.live_metrics_payloads[-1]["reward_mean"] == [0.5]


def test_start_training_reports_failure_without_crashing(monkeypatch):
    """The controller keeps the UI responsive if training raises an exception."""
    module = _load_controller_module(monkeypatch)
    view = DummyView()
    controller = module.TrainTestViewController(view, DummyApp())

    controller.set_context({"robogym_project": {"name": "Demo Project"}})
    DummyTrainer.raise_on_train = True

    try:
        controller.start_training()
        controller.wait_for_training_completion(timeout=2)
        controller.process_pending_training_events()
    finally:
        DummyTrainer.raise_on_train = False

    assert controller.trainer is not None
    assert controller.trainer.train_calls == 1
    assert view.training_enabled_states == [False, True]
    assert view.refreshed_agent_names == []
    assert view.history_refreshes == ["2026_03_26_03_15_pm_ep_0003"]
    assert (
        view.terminal_messages[-1]
        == "Training failed.\nRuntimeError: training exploded"
    )


def test_navigate_to_settings_opens_training_settings_page(monkeypatch):
    """The controller opens the training settings page with the current project."""
    module = _load_controller_module(monkeypatch)
    view = DummyView()
    app = DummyApp()
    controller = module.TrainTestViewController(view, app)
    raw_project = {"robogym_project": {"name": "Demo Project"}}

    controller.set_context(raw_project)
    controller.navigate_to_settings()

    assert app.calls == [(("training_settings",), {"project": raw_project})]
