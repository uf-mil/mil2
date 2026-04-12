"""Tests for ViewProjectPage context normalization."""

from __future__ import annotations

from types import SimpleNamespace

import mil_robogym.ui.pages.view_project_page.view_project_page as view_project_page_module
from mil_robogym.ui.pages.view_project_page.view_project_page import ViewProjectPage


class _DummyController:
    def __init__(self) -> None:
        self.calls: list[tuple[tuple[object, ...], dict[str, object]]] = []

    def show_page(self, *args, **kwargs) -> None:
        self.calls.append((args, kwargs))


def test_view_project_page_uses_reloaded_project_for_demo_navigation():
    """Demo navigation should use the full reloaded project, not a name-only stub."""
    page = ViewProjectPage.__new__(ViewProjectPage)
    page.controller = _DummyController()
    page.project = None
    page.project_name = "Project"
    page._num_demos = 0
    page._demo_names = []
    page._demos = {}
    page._page_title = SimpleNamespace(configure=lambda **kwargs: None)
    page._render_demo_rows = lambda: None

    loaded_project = {
        "robogym_project": {
            "name": "Demo Project",
            "input_topics": {"/ping": ["frequency"]},
            "output_topics": {"/goal": ["yaw"]},
            "random_spawn_space": {
                "enabled": False,
                "coord1_4d": [0.0, 0.0, 0.0, 0.0],
                "coord2_4d": [0.0, 0.0, 0.0, 0.0],
            },
            "world_file": "world.sdf",
            "tensor_spec": {
                "input_features": ["/ping:frequency"],
                "output_features": ["/goal:yaw"],
                "input_dim": 1,
                "output_dim": 1,
            },
        },
        "num_demos": 1,
    }
    page._safe_get_project_by_name = lambda name: loaded_project
    page._safe_get_demo_names = lambda project_name: ["Demo 1"]

    page.set_context(project={"robogym_project": {"name": "Demo Project"}})
    page._on_demo_row_click("Demo 1", {"robogym_demo": {"name": "Demo 1"}})

    assert page.project == loaded_project
    assert page.controller.calls == [
        (
            ("view_demo",),
            {
                "project": loaded_project,
                "demo_name": "Demo 1",
                "demo": {"robogym_demo": {"name": "Demo 1"}},
            },
        ),
    ]


def test_view_project_page_record_demo_passes_project_and_navigates_on_create(
    monkeypatch,
):
    """Record-demo popup should receive the current project and route create callbacks."""
    page = ViewProjectPage.__new__(ViewProjectPage)
    page.controller = _DummyController()
    page.project = {"robogym_project": {"name": "Demo Project"}}
    page.create_demo_popup = None

    captured_kwargs: dict[str, object] = {}

    class _DummyPopup:
        def __init__(self, *_args, **kwargs) -> None:
            captured_kwargs.update(kwargs)
            self.win = SimpleNamespace(
                winfo_exists=lambda: True,
                lift=lambda: None,
                focus_force=lambda: None,
            )

    monkeypatch.setattr(view_project_page_module, "CreateDemoPopup", _DummyPopup)

    page._on_record_demo()

    assert captured_kwargs["project"] == {"name": "Demo Project"}
    on_created = captured_kwargs["on_created"]
    on_created("New Demo", {"robogym_demo": {"name": "New Demo"}})

    assert page.controller.calls == [
        (
            ("view_demo",),
            {
                "project": {"robogym_project": {"name": "Demo Project"}},
                "demo_name": "New Demo",
                "demo": {"robogym_demo": {"name": "New Demo"}},
            },
        ),
    ]
