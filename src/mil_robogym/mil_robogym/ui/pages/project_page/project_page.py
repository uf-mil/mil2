import tkinter as tk


class ProjectPage(tk.Frame):
    def __init__(self, parent, controller=None):
        super().__init__(parent, bg="#DADADA")
        self.controller = controller

        self._name_label = tk.Label(self, text="Project", bg="#DADADA", fg="black")
        self._name_label.pack(anchor="w", padx=12, pady=10)

    def set_context(self, project=None, **_kwargs):
        if project is None:
            self._name_label.configure(text="Project")
            return

        name = project.get("robogym_project", {}).get("name")
        if not name:
            name = project.get("name", "Project")
        self._name_label.configure(text=str(name))
