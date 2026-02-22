import tkinter as tk

from mil_robogym.nodes.teleop import TeleopNode

PUBLISH_RATE = 0.1  # seconds (10 Hz)


class ToolTip:
    def __init__(self, widget, text):
        self.widget = widget
        self.text = text
        self.tip = None

        widget.bind("<Enter>", self.show)
        widget.bind("<Leave>", self.hide)

    def show(self, _):
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

    def hide(self, _):
        if self.tip:
            self.tip.destroy()
            self.tip = None


class KeyButton(tk.Label):
    def __init__(self, master, key_name, tooltip, press_cb, release_cb):
        super().__init__(
            master,
            text=key_name,
            width=6,
            height=3,
            bg="#dddddd",
            relief="raised",
            font=("Arial", 12, "bold"),
        )

        self.key_name = key_name
        self.press_cb = press_cb
        self.release_cb = release_cb

        ToolTip(self, tooltip)

        self.bind("<ButtonPress-1>", self.on_press)
        self.bind("<ButtonRelease-1>", self.on_release)

    def highlight(self, active):
        self.configure(bg="red" if active else "#dddddd")

    def on_press(self, _):
        self.highlight(True)
        self.press_cb(self.key_name)

    def on_release(self, _):
        self.highlight(False)
        self.release_cb(self.key_name)


class TeleopGUI:
    def __init__(self, root, on_close_callback):

        self.on_close_callback = on_close_callback

        self.node = TeleopNode()

        self.root = tk.Toplevel(root)
        self.root.withdraw()
        self.root.title("Subjugator Teleop")
        self.root.geometry("460x460")
        self.root.configure(bg="#222")

        self.root.bind("<KeyPress>", self.key_press)
        self.root.bind("<KeyRelease>", self.key_release)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.focus_set()

        self.buttons = {}
        self.build_keyboard()

        self.update_loop()

    def build_keyboard(self):
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

    # --- keyboard events ---
    def key_press(self, event):
        key = event.keysym
        if key in self.buttons:
            self.buttons[key].highlight(True)
            self.activate(key)

    def key_release(self, event):
        key = event.keysym
        if key in self.buttons:
            self.buttons[key].highlight(False)
            self.deactivate(key)

    def activate(self, key):
        if key == "m":
            self.node.spawn_marble()
            return
        self.node.active_keys.add(key)

    def deactivate(self, key):
        self.node.active_keys.discard(key)

    def update_loop(self):
        self.node.publish()
        self.root.after(int(PUBLISH_RATE * 1000), self.update_loop)

    def show(self):
        self.root.deiconify()
        self.root.lift()
        self.root.focus_force()

    def hide(self):
        self.root.withdraw()

    def _on_close(self):
        self.hide()
        self.on_close_callback()
