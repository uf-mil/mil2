"""Tests for the reusable create-demo popup."""

from __future__ import annotations

from types import SimpleNamespace

import mil_robogym.ui.components.create_demo_popup as create_demo_popup_module
from mil_robogym.ui.components.create_demo_popup import (
    CreateDemoPopup,
    _default_demo_name,
)


def test_create_demo_popup_creates_demo_and_calls_on_created(monkeypatch):
    popup = CreateDemoPopup.__new__(CreateDemoPopup)
    popup.project = {"name": "Demo Project"}
    popup.default_start_position = (1.0, 2.0, 3.0, 4.0)
    popup.name_var = SimpleNamespace(get=lambda: "Demo 2")
    popup.rate_var = SimpleNamespace(get=lambda: "5")

    warning_actions: list[str] = []
    popup.warning_label = SimpleNamespace(
        grid_remove=lambda: warning_actions.append("hide"),
        grid=lambda: warning_actions.append("show"),
    )

    destroyed: list[bool] = []
    popup.win = SimpleNamespace(destroy=lambda: destroyed.append(True))

    created_calls: list[tuple[str, dict]] = []
    popup.on_created = lambda name, cfg: created_calls.append((name, cfg))

    folder_calls: list[tuple[object, str, float, tuple[float, float, float, float]]] = (
        []
    )

    def fake_create_demo_folder(project, *, name, sampling_rate, start_position):
        folder_calls.append((project, name, sampling_rate, start_position))
        return "/tmp/demo", {"robogym_demo": {"name": name}}

    monkeypatch.setattr(
        create_demo_popup_module,
        "create_demo_folder",
        fake_create_demo_folder,
    )

    popup._create()

    assert warning_actions == ["hide"]
    assert folder_calls == [
        ({"name": "Demo Project"}, "Demo 2", 5.0, (1.0, 2.0, 3.0, 4.0)),
    ]
    assert destroyed == [True]
    assert created_calls == [("Demo 2", {"robogym_demo": {"name": "Demo 2"}})]


def test_create_demo_popup_shows_warning_when_demo_exists(monkeypatch):
    popup = CreateDemoPopup.__new__(CreateDemoPopup)
    popup.project = {"name": "Demo Project"}
    popup.default_start_position = None
    popup.name_var = SimpleNamespace(get=lambda: "Demo 1")
    popup.rate_var = SimpleNamespace(get=lambda: "10")

    warning_actions: list[str] = []
    popup.warning_label = SimpleNamespace(
        grid_remove=lambda: warning_actions.append("hide"),
        grid=lambda: warning_actions.append("show"),
    )

    destroyed: list[bool] = []
    popup.win = SimpleNamespace(destroy=lambda: destroyed.append(True))

    created_calls: list[tuple[str, dict]] = []
    popup.on_created = lambda name, cfg: created_calls.append((name, cfg))

    def fake_create_demo_folder(*_args, **_kwargs):
        raise FileExistsError

    monkeypatch.setattr(
        create_demo_popup_module,
        "create_demo_folder",
        fake_create_demo_folder,
    )

    popup._create()

    assert warning_actions == ["hide", "show"]
    assert destroyed == []
    assert created_calls == []


def test_default_demo_name_uses_existing_demo_count(monkeypatch):
    monkeypatch.setattr(
        create_demo_popup_module,
        "get_all_demo_config",
        lambda project_name: {
            "Demo 1": {"robogym_demo": {"name": "Demo 1"}},
            "Demo 2": {"robogym_demo": {"name": "Demo 2"}},
        },
    )

    assert _default_demo_name({"name": "Demo Project"}) == "Demo 3"


def test_default_demo_name_starts_at_demo_one_when_no_demos(monkeypatch):
    monkeypatch.setattr(
        create_demo_popup_module,
        "get_all_demo_config",
        lambda project_name: {},
    )

    assert _default_demo_name({"name": "Demo Project"}) == "Demo 1"
