from __future__ import annotations

import tkinter as tk
from collections.abc import Callable

WheelScrollHandler = Callable[[int], bool]


def mouse_wheel_units(event: tk.Event) -> int:
    """Normalize Tk mouse-wheel events into signed vertical scroll units."""
    button_num = getattr(event, "num", None)
    if button_num == 4:
        return -1
    if button_num == 5:
        return 1

    delta = int(getattr(event, "delta", 0) or 0)
    if delta == 0:
        return 0

    magnitude = max(abs(delta) // 120, 1)
    return -magnitude if delta > 0 else magnitude


def scroll_canvas(canvas: tk.Canvas, units: int, *, unit_factor: int = 3) -> bool:
    """Scroll a canvas if it can move in the requested direction."""
    top, bottom = canvas.yview()
    if units < 0 and top <= 0.0:
        return False
    if units > 0 and bottom >= 1.0:
        return False

    canvas.yview_scroll(units * unit_factor, "units")
    return True


class _MouseWheelBinding:
    def __init__(self, region: tk.Widget, on_scroll: WheelScrollHandler) -> None:
        self.region = region
        self.on_scroll = on_scroll


class MouseWheelRouter:
    """Route global mouse-wheel input to the deepest matching scroll region."""

    def __init__(self, root: tk.Misc) -> None:
        self._root = root
        self._bindings: list[_MouseWheelBinding] = []

        for sequence in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            self._root.bind_all(sequence, self._dispatch, add="+")

    def register(
        self,
        region: tk.Widget,
        on_scroll: WheelScrollHandler,
    ) -> _MouseWheelBinding:
        binding = _MouseWheelBinding(region, on_scroll)
        self._bindings.append(binding)
        region.bind(
            "<Destroy>",
            lambda _event, registered=binding: self.unregister(registered),
            add="+",
        )
        return binding

    def unregister(self, binding: _MouseWheelBinding) -> None:
        if binding in self._bindings:
            self._bindings.remove(binding)

    def _dispatch(self, event: tk.Event) -> str | None:
        units = mouse_wheel_units(event)
        if units == 0:
            return None

        pointer_widget = self._widget_under_pointer()
        if pointer_widget is None:
            return None

        for binding in self._matching_bindings(pointer_widget):
            if binding.on_scroll(units):
                return "break"
        return None

    def _widget_under_pointer(self) -> tk.Widget | None:
        try:
            return self._root.winfo_containing(
                self._root.winfo_pointerx(),
                self._root.winfo_pointery(),
            )
        except tk.TclError:
            return None

    def _matching_bindings(
        self,
        pointer_widget: tk.Widget,
    ) -> list[_MouseWheelBinding]:
        matching: list[_MouseWheelBinding] = []
        stale: list[_MouseWheelBinding] = []

        for binding in self._bindings:
            region = binding.region
            try:
                if not bool(region.winfo_exists()):
                    stale.append(binding)
                    continue
            except tk.TclError:
                stale.append(binding)
                continue

            if self._contains_widget(region, pointer_widget):
                matching.append(binding)

        for binding in stale:
            self.unregister(binding)

        matching.sort(
            key=lambda binding: self._widget_depth(binding.region),
            reverse=True,
        )
        return matching

    def _contains_widget(
        self,
        ancestor: tk.Widget,
        widget: tk.Widget | None,
    ) -> bool:
        current = widget
        while current is not None:
            if current == ancestor:
                return True
            current = getattr(current, "master", None)
        return False

    def _widget_depth(self, widget: tk.Widget) -> int:
        depth = 0
        current: tk.Widget | None = widget
        while current is not None:
            depth += 1
            current = getattr(current, "master", None)
        return depth


def get_mouse_wheel_router(widget: tk.Widget) -> MouseWheelRouter:
    """Return the shared router for the widget's Tk toplevel."""
    root = widget.winfo_toplevel()
    router = getattr(root, "_mil_mouse_wheel_router", None)
    if router is None:
        router = MouseWheelRouter(root)
        setattr(root, "_mil_mouse_wheel_router", router)
    return router
