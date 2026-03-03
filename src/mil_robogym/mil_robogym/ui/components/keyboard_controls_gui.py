import tkinter as tk
from typing import Callable, Dict, Optional

from mil_robogym.nodes.teleop import TeleopNode

PUBLISH_RATE: float = 0.1  # seconds (10 Hz)


class ToolTip:
    """
    A small hover tooltip component that displays contextual information
    when the mouse enters a widget and disappears when it leaves.
    """

    def __init__(self, widget: tk.Widget, text: str) -> None:
        """
        Initialize the tooltip.

        Args:
            widget: The tkinter widget this tooltip is attached to.
            text: The text displayed inside the tooltip.
        """
        self.widget: tk.Widget = widget
        self.text: str = text
        self.tip: Optional[tk.Toplevel] = None

        widget.bind("<Enter>", self.show)
        widget.bind("<Leave>", self.hide)

    def show(self, _: tk.Event) -> None:
        """
        Display the tooltip near the widget.
        """
        x, y, _, _ = self.widget.bbox("insert") or (0, 0, 0, 0)
        x += self.widget.winfo_rootx() + 30
        y += self.widget.winfo_rooty() + 30

        self.tip = tk.Toplevel(self.widget)
        self.tip.wm_overrideredirect(True)
        self.tip.geometry(f"+{x}+{y}")

        label = tk.Label(
            self.tip,
            text=self.text,
            bg="black",
            fg="white",
            padx=6,
            pady=3,
        )
        label.pack()

    def hide(self, _: tk.Event) -> None:
        """
        Destroy the tooltip window if it exists.
        """
        if self.tip is not None:
            self.tip.destroy()
            self.tip = None


class KeyButton(tk.Label):
    """
    A clickable on-screen key representation that mirrors keyboard input
    and triggers press/release callbacks.
    """

    def __init__(
        self,
        master: tk.Widget,
        key_name: str,
        tooltip: str,
        press_cb: Callable[[str], None],
        release_cb: Callable[[str], None],
    ) -> None:
        """
        Initialize a KeyButton.

        Args:
            master: Parent tkinter widget.
            key_name: Name of the key (e.g., "w", "Up").
            tooltip: Text displayed when hovering over the button.
            press_cb: Callback invoked on key press.
            release_cb: Callback invoked on key release.
        """
        super().__init__(
            master,
            text=key_name,
            width=6,
            height=3,
            bg="#dddddd",
            relief="raised",
            font=("Arial", 12, "bold"),
        )

        self.key_name: str = key_name
        self.press_cb: Callable[[str], None] = press_cb
        self.release_cb: Callable[[str], None] = release_cb

        ToolTip(self, tooltip)

        self.bind("<ButtonPress-1>", self.on_press)
        self.bind("<ButtonRelease-1>", self.on_release)

    def highlight(self, active: bool) -> None:
        """
        Change the visual state of the button.

        Args:
            active: Whether the button should appear pressed.
        """
        self.configure(bg="red" if active else "#dddddd")

    def on_press(self, _: tk.Event) -> None:
        """
        Handle mouse press event.
        """
        self.highlight(True)
        self.press_cb(self.key_name)

    def on_release(self, _: tk.Event) -> None:
        """
        Handle mouse release event.
        """
        self.highlight(False)
        self.release_cb(self.key_name)


class KeyboardControlsGUI:
    """
    Graphical teleoperation interface for controlling the robot.

    This GUI provides both clickable buttons and physical keyboard
    bindings to control a TeleopNode.
    """

    def __init__(
        self,
        root: tk.Widget,
        on_close_callback: Callable[[], None],
    ) -> None:
        """
        Initialize the teleoperation GUI.

        Args:
            root: Root tkinter widget.
            on_close_callback: Function called when the window is closed.
        """
        self.on_close_callback: Callable[[], None] = on_close_callback
        self.node: TeleopNode = TeleopNode()

        self.root: tk.Toplevel = tk.Toplevel(root)
        self.root.withdraw()
        self.root.title("Subjugator Teleop")
        self.root.geometry("460x460")
        self.root.configure(bg="#222")

        self.root.bind("<KeyPress>", self.key_press)
        self.root.bind("<KeyRelease>", self.key_release)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.focus_set()

        self.buttons: Dict[str, KeyButton] = {}
        self.build_keyboard()

        self.update_loop()

    def build_keyboard(self) -> None:
        """
        Construct the visual keyboard layout.
        """
        frame = tk.Frame(self.root, bg="#222")
        frame.pack(pady=20)

        layout = [
            [("w", "+X Force"), ("e", "Roll Left"), ("r", "Roll Right")],
            [
                ("a", "+Y Force"),
                ("s", "-X Force"),
                ("d", "-Y Force"),
                ("m", "Spawn Marble"),
            ],
            [("z", "-Z Force"), ("x", "+Z Force")],
            [("Up", "Pitch Up")],
            [("Left", "Yaw Left"), ("Down", "Pitch Down"), ("Right", "Yaw Right")],
        ]

        for row in layout:
            r = tk.Frame(frame, bg="#222")
            r.pack()
            for key, tip in row:
                btn = KeyButton(r, key, tip, self.activate, self.deactivate)
                btn.pack(side="left", padx=5, pady=5)
                self.buttons[key] = btn

    def key_press(self, event: tk.Event) -> None:
        """
        Handle physical keyboard key press.
        """
        key = event.keysym
        if key in self.buttons:
            self.buttons[key].highlight(True)
            self.activate(key)

    def key_release(self, event: tk.Event) -> None:
        """
        Handle physical keyboard key release.
        """
        key = event.keysym
        if key in self.buttons:
            self.buttons[key].highlight(False)
            self.deactivate(key)

    def activate(self, key: str) -> None:
        """
        Activate a control key.

        Args:
            key: Key identifier.
        """
        if key == "m":
            self.node.spawn_marble()
            return

        self.node.active_keys.add(key)

    def deactivate(self, key: str) -> None:
        """
        Deactivate a control key.

        Args:
            key: Key identifier.
        """
        self.node.active_keys.discard(key)

    def update_loop(self) -> None:
        """
        Periodically publish teleoperation commands.
        """
        self.node.publish()
        self.root.after(int(PUBLISH_RATE * 1000), self.update_loop)

    def show(self) -> None:
        """
        Display and focus the teleoperation window.
        """
        self.root.deiconify()
        self.root.lift()
        self.root.focus_force()

    def _on_close(self) -> None:
        """
        Handle window close event.
        """
        self.on_close_callback()
        self.root.destroy()
