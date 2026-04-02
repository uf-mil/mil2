"""Tests for the Train/Test controller wiring."""

from __future__ import annotations

import importlib
import sys
import threading
import types
from pathlib import Path


class DummyTrainer:
    """Minimal trainer stub for controller tests."""

    raise_on_train = False
    block_on_train = False
    train_started = threading.Event()
    allow_train_finish = threading.Event()

    def __init__(self, project, **kwargs):
        self.project = project
        self.settings = kwargs
        self.progress_callback = kwargs.get("progress_callback")
        self.train_calls = 0
        self.stop_requested = False
        self.abort_requested = False

    def train(self):
        self.train_calls += 1
        type(self).train_started.set()
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
        if type(self).block_on_train:
            type(self).allow_train_finish.wait(timeout=2)
        if type(self).raise_on_train:
            raise RuntimeError("training exploded")
        if self.abort_requested:
            return []
        return [Path("/tmp/2026_03_26_03_15_pm_final")]

    def request_stop(self):
        self.stop_requested = True
        type(self).allow_train_finish.set()

    def request_abort(self):
        self.abort_requested = True
        self.stop_requested = True
        type(self).allow_train_finish.set()

    def was_stopped(self):
        return self.stop_requested

    def was_aborted(self):
        return self.abort_requested


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


def _reset_dummy_trainer_state():
    DummyTrainer.raise_on_train = False
    DummyTrainer.block_on_train = False
    DummyTrainer.train_started = threading.Event()
    DummyTrainer.allow_train_finish = threading.Event()


def test_start_training_refreshes_history_with_saved_agent(monkeypatch):
    """The controller refreshes the UI using the agent saved by training."""
    _reset_dummy_trainer_state()
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
    _reset_dummy_trainer_state()
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


def test_stop_training_requests_worker_shutdown(monkeypatch):
    """The controller can stop a background training run without freezing the UI."""
    _reset_dummy_trainer_state()
    DummyTrainer.block_on_train = True
    module = _load_controller_module(monkeypatch)
    view = DummyView()
    controller = module.TrainTestViewController(view, DummyApp())

    controller.set_context({"robogym_project": {"name": "Demo Project"}})
    controller.start_training()
    assert DummyTrainer.train_started.wait(timeout=2)

    controller.stop_training()
    controller.wait_for_training_completion(timeout=2)
    controller.process_pending_training_events()

    assert controller.trainer is not None
    assert controller.trainer.stop_requested is True
    assert view.training_enabled_states == [False, True]
    assert view.refreshed_agent_names == ["2026_03_26_03_15_pm_final"]
    assert view.history_refreshes == ["2026_03_26_03_15_pm_ep_0003"]
    assert view.terminal_messages[-2] == (
        "Stopping training...\nWaiting for the current episode to finish."
    )
    assert view.terminal_messages[-1] == (
        "Training stopped.\nLatest saved agent: 2026_03_26_03_15_pm_final"
    )


def test_load_selected_agent_reports_success(monkeypatch):
    """Loads the selected saved model through the dedicated helper."""
    _reset_dummy_trainer_state()
    module = _load_controller_module(monkeypatch)
    view = DummyView()
    controller = module.TrainTestViewController(view, DummyApp())
    view.selected_agent_name = "2026_03_30_11_15_am_final"
    loaded_agent = types.SimpleNamespace(
        handle=types.SimpleNamespace(
            agent_name="2026_03_30_11_15_am_final",
            checkpoint_episode=None,
        ),
        input_size=12,
        output_size=4,
    )

    monkeypatch.setattr(
        module,
        "load_saved_agent_model",
        lambda project, agent_name: loaded_agent,
    )

    controller.set_context({"robogym_project": {"name": "Demo Project"}})
    resolved_agent = controller.load_selected_agent()

    assert resolved_agent is loaded_agent
    assert controller.loaded_agent is loaded_agent
    assert view.terminal_messages[-1] == (
        "Loaded saved model.\n"
        "2026_03_30_11_15_am_final | final model | in 12 -> out 4"
    )


def test_load_selected_agent_reports_failure(monkeypatch):
    """Surfaces saved-model load failures in the terminal panel."""
    _reset_dummy_trainer_state()
    module = _load_controller_module(monkeypatch)
    view = DummyView()
    controller = module.TrainTestViewController(view, DummyApp())
    view.selected_agent_name = "missing_agent"

    def _raise(_project, _agent_name):
        raise FileNotFoundError("missing saved model")

    monkeypatch.setattr(module, "load_saved_agent_model", _raise)

    controller.set_context({"robogym_project": {"name": "Demo Project"}})
    resolved_agent = controller.load_selected_agent()

    assert resolved_agent is None
    assert controller.loaded_agent is None
    assert view.terminal_messages[-1] == (
        "Failed to load saved model.\n" "FileNotFoundError: missing saved model"
    )


def test_delete_saved_agent_reports_success_and_clears_loaded_agent(monkeypatch):
    """Deleting a loaded saved model clears the in-memory handle."""
    _reset_dummy_trainer_state()
    module = _load_controller_module(monkeypatch)
    view = DummyView()
    controller = module.TrainTestViewController(view, DummyApp())
    controller.loaded_agent = types.SimpleNamespace(
        handle=types.SimpleNamespace(agent_name="saved_final"),
    )

    monkeypatch.setattr(
        module,
        "delete_saved_agent_artifacts",
        lambda project, agent_name: [Path("/tmp/source/agents/saved_final")],
    )

    controller.set_context({"robogym_project": {"name": "Demo Project"}})
    controller.loaded_agent = types.SimpleNamespace(
        handle=types.SimpleNamespace(agent_name="saved_final"),
    )
    deleted = controller.delete_saved_agent("saved_final")

    assert deleted is True
    assert controller.loaded_agent is None
    assert view.terminal_messages[-1] == (
        "Deleted saved model.\nsaved_final | removed 1 copy"
    )


def test_delete_saved_agent_is_blocked_while_training_runs(monkeypatch):
    """The controller refuses deletes while the training worker is active."""
    _reset_dummy_trainer_state()
    DummyTrainer.block_on_train = True
    module = _load_controller_module(monkeypatch)
    view = DummyView()
    controller = module.TrainTestViewController(view, DummyApp())

    controller.set_context({"robogym_project": {"name": "Demo Project"}})
    controller.start_training()
    assert DummyTrainer.train_started.wait(timeout=2)

    try:
        deleted = controller.delete_saved_agent("saved_final")
    finally:
        controller.abort_training()
        controller.wait_for_training_completion(timeout=2)
        controller.process_pending_training_events()

    assert deleted is False
    assert view.terminal_messages[-3] == (
        "Cannot delete saved models while training runs."
    )


def test_abort_training_uses_latest_completed_checkpoint(monkeypatch):
    """Abort skips the final save and falls back to the last completed checkpoint."""
    _reset_dummy_trainer_state()
    DummyTrainer.block_on_train = True
    module = _load_controller_module(monkeypatch)
    view = DummyView()
    controller = module.TrainTestViewController(view, DummyApp())

    controller.set_context({"robogym_project": {"name": "Demo Project"}})
    controller.start_training()
    assert DummyTrainer.train_started.wait(timeout=2)

    controller.abort_training()
    controller.wait_for_training_completion(timeout=2)
    controller.process_pending_training_events()

    assert controller.trainer is not None
    assert controller.trainer.abort_requested is True
    assert controller.trainer.stop_requested is True
    assert view.training_enabled_states == [False, True]
    assert view.history_refreshes == ["2026_03_26_03_15_pm_ep_0003"]
    assert view.refreshed_agent_names == ["2026_03_26_03_15_pm_ep_0003"]
    assert view.terminal_messages[-2] == (
        "Aborting training...\n"
        "Unsaved progress from the current episode will be discarded."
    )
    assert view.terminal_messages[-1] == (
        "Training aborted.\nLatest saved agent: 2026_03_26_03_15_pm_ep_0003"
    )


def test_navigate_to_settings_opens_training_settings_page(monkeypatch):
    """The controller opens the training settings page with the current project."""
    _reset_dummy_trainer_state()
    module = _load_controller_module(monkeypatch)
    view = DummyView()
    app = DummyApp()
    controller = module.TrainTestViewController(view, app)
    raw_project = {"robogym_project": {"name": "Demo Project"}}

    controller.set_context(raw_project)
    controller.navigate_to_settings()

    assert app.calls == [(("training_settings",), {"project": raw_project})]
