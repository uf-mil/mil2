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
        self.train_calls = 0

    def train(self):
        self.train_calls += 1
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
        self.flush_calls = 0

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

    def flush_ui_updates(self) -> None:
        self.flush_calls += 1


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

    assert controller.trainer is not None
    assert controller.trainer.settings["num_episodes"] == 7
    assert controller.trainer.settings["rollout_steps"] == 512
    assert controller.trainer.settings["save_every"] == 3
    assert controller.trainer.settings["seed"] == 9
    assert controller.trainer.train_calls == 1
    assert view.training_enabled_states == [False, True]
    assert view.refreshed_agent_names == ["2026_03_26_03_15_pm_final"]
    assert view.selected_agent_name == "2026_03_26_03_15_pm_final"
    assert "Training agent with saved settings" in view.terminal_messages[0]
    assert "Latest saved agent: 2026_03_26_03_15_pm_final" in view.terminal_messages[-1]


def test_start_training_reports_failure_without_crashing(monkeypatch):
    """The controller keeps the UI responsive if training raises an exception."""
    module = _load_controller_module(monkeypatch)
    view = DummyView()
    controller = module.TrainTestViewController(view, DummyApp())

    controller.set_context({"robogym_project": {"name": "Demo Project"}})
    DummyTrainer.raise_on_train = True

    try:
        controller.start_training()
    finally:
        DummyTrainer.raise_on_train = False

    assert controller.trainer is not None
    assert controller.trainer.train_calls == 1
    assert view.training_enabled_states == [False, True]
    assert view.refreshed_agent_names == []
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
