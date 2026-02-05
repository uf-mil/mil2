import tkinter as tk


class ProjectRow(tk.Frame):
    def __init__(self, parent, left_text, right_text, command=None):
        """
        Creates a button-like row with left and right text.
        Clicking anywhere on the row triggers `command` (if provided).
        """
        super().__init__(
            parent,
            bg="#E6E6E6",
            bd=1,
            relief="solid",
            highlightthickness=0,
        )

        # Inner padding area (so the text doesn't touch the border)
        inner = tk.Frame(self, bg="#E6E6E6")
        inner.grid(row=0, column=0, sticky="nsew", padx=10, pady=6)

        inner.grid_columnconfigure(0, weight=1)
        inner.grid_columnconfigure(1, weight=0)

        left = tk.Label(inner, text=left_text, bg="#E6E6E6", fg="black", anchor="w")
        right = tk.Label(inner, text=right_text, bg="#E6E6E6", fg="black", anchor="e")

        left.grid(row=0, column=0, sticky="w")
        right.grid(row=0, column=1, sticky="e", padx=(20, 0))

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # Make the whole row clickable
        def on_click(_event=None):
            if callable(command):
                command()

        if command is not None:
            self.configure(cursor="hand2")
            for w in (self, inner, left, right):
                w.bind("<Button-1>", on_click)

        # Hover effect (subtle)
        def on_enter(_e=None):
            if command is None:
                return
            for w in (self, inner, left, right):
                w.configure(bg="#DCDCDC")

        def on_leave(_e=None):
            if command is None:
                return
            for w in (self, inner, left, right):
                w.configure(bg="#E6E6E6")

        if command is not None:
            for w in (self, inner, left, right):
                w.bind("<Enter>", on_enter)
                w.bind("<Leave>", on_leave)
