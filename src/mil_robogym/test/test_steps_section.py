"""Tests for View Demo steps-section scrolling behavior."""

from mil_robogym.ui.pages.view_demo_page.steps_section import StepsSection


class _DummyCanvas:
    def __init__(self, *, height: int) -> None:
        self._height = height
        self.yview_positions: list[float] = []

    def winfo_height(self) -> int:
        return self._height

    def yview_moveto(self, fraction: float) -> None:
        self.yview_positions.append(fraction)


class _DummyStepsFrame:
    def __init__(self, *, requested_height: int) -> None:
        self._requested_height = requested_height

    def winfo_reqheight(self) -> int:
        return self._requested_height


class _DummyScrollbar:
    def __init__(self) -> None:
        self.states: list[str] = []

    def configure(self, *, state: str) -> None:
        self.states.append(state)


def test_steps_section_disables_scrollbar_when_steps_fit() -> None:
    section = StepsSection.__new__(StepsSection)
    section.canvas = _DummyCanvas(height=240)
    section.steps_frame = _DummyStepsFrame(requested_height=180)
    section.scrollbar = _DummyScrollbar()

    section._update_scrollbar_state()

    assert section.canvas.yview_positions == [0.0]
    assert section.scrollbar.states == ["disabled"]


def test_steps_section_enables_scrollbar_when_steps_overflow() -> None:
    section = StepsSection.__new__(StepsSection)
    section.canvas = _DummyCanvas(height=240)
    section.steps_frame = _DummyStepsFrame(requested_height=420)
    section.scrollbar = _DummyScrollbar()

    section._update_scrollbar_state()

    assert section.canvas.yview_positions == []
    assert section.scrollbar.states == ["normal"]
