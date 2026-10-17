"""Tests for shared scrollable-frame sizing behavior."""

from mil_robogym.ui.components.scrollable_frame import ScrollableFrame


class _DummyCanvas:
    def __init__(self, *, width: int, height: int) -> None:
        self._width = width
        self._height = height
        self.itemconfigure_calls: list[tuple[object, dict[str, int]]] = []
        self.yview_positions: list[float] = []

    def itemconfigure(self, window: object, **kwargs: int) -> None:
        self.itemconfigure_calls.append((window, kwargs))

    def winfo_width(self) -> int:
        return self._width

    def winfo_height(self) -> int:
        return self._height

    def yview_moveto(self, fraction: float) -> None:
        self.yview_positions.append(fraction)


class _DummyContent:
    def __init__(self, *, requested_height: int) -> None:
        self._requested_height = requested_height

    def winfo_reqheight(self) -> int:
        return self._requested_height


def test_scrollable_frame_syncs_canvas_width_without_forcing_height() -> None:
    frame = ScrollableFrame.__new__(ScrollableFrame)
    frame.canvas = _DummyCanvas(width=360, height=240)
    frame.content = _DummyContent(requested_height=480)
    frame._win = object()
    frame._fill_height = False

    frame._sync_window_size()

    assert frame.canvas.itemconfigure_calls == [
        (frame._win, {"width": 360}),
    ]


def test_scrollable_frame_fill_height_uses_larger_of_content_and_viewport() -> None:
    frame = ScrollableFrame.__new__(ScrollableFrame)
    frame.canvas = _DummyCanvas(width=360, height=240)
    frame.content = _DummyContent(requested_height=480)
    frame._win = object()
    frame._fill_height = True

    frame._sync_window_size()

    assert frame.canvas.itemconfigure_calls == [
        (frame._win, {"width": 360}),
        (frame._win, {"height": 480}),
    ]


def test_scrollable_frame_reset_scroll_moves_to_top() -> None:
    frame = ScrollableFrame.__new__(ScrollableFrame)
    frame.canvas = _DummyCanvas(width=360, height=240)

    frame.reset_scroll()

    assert frame.canvas.yview_positions == [0.0]
