"""Tests for shared mouse-wheel helpers."""

from types import SimpleNamespace

from mil_robogym.ui.components.mouse_wheel import mouse_wheel_units, scroll_canvas


class _DummyCanvas:
    def __init__(self, *, yview: tuple[float, float]) -> None:
        self._yview = yview
        self.scroll_calls: list[tuple[int, str]] = []

    def yview(self) -> tuple[float, float]:
        return self._yview

    def yview_scroll(self, amount: int, what: str) -> None:
        self.scroll_calls.append((amount, what))


def test_mouse_wheel_units_normalizes_mousewheel_delta() -> None:
    assert mouse_wheel_units(SimpleNamespace(delta=120)) == -1
    assert mouse_wheel_units(SimpleNamespace(delta=-240)) == 2


def test_mouse_wheel_units_normalizes_linux_button_events() -> None:
    assert mouse_wheel_units(SimpleNamespace(num=4, delta=0)) == -1
    assert mouse_wheel_units(SimpleNamespace(num=5, delta=0)) == 1


def test_scroll_canvas_scrolls_when_canvas_can_move() -> None:
    canvas = _DummyCanvas(yview=(0.2, 0.7))

    consumed = scroll_canvas(canvas, 1)

    assert consumed is True
    assert canvas.scroll_calls == [(3, "units")]


def test_scroll_canvas_bubbles_when_already_at_boundary() -> None:
    canvas = _DummyCanvas(yview=(0.0, 1.0))

    consumed = scroll_canvas(canvas, 1)

    assert consumed is False
    assert canvas.scroll_calls == []
