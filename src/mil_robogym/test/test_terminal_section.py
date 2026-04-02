from mil_robogym.ui.pages.train_test_page.terminal_section import TerminalSection


class _DummyText:
    def __init__(self) -> None:
        self.content = ""

    def configure(self, **_kwargs) -> None:
        return

    def insert(self, _index, text: str) -> None:
        self.content += text

    def see(self, _index) -> None:
        return


def _make_section() -> TerminalSection:
    section = TerminalSection.__new__(TerminalSection)
    section.output_text = _DummyText()
    section._terminal_line_buffer = ""
    return section


def test_filter_terminal_text_hides_pose_success_line_case_insensitive() -> None:
    section = _make_section()

    filtered = section._filter_terminal_text(
        "[INFO] Pose set successfully: pos=(1.0, 2.0, -1.0)\n"
        "[INFO] pose set successfully: pos=(3.0, 4.0, -1.0)\n"
        "keep this line\n",
    )

    assert "Pose set successfully" not in filtered
    assert "pose set successfully" not in filtered
    assert "keep this line" in filtered


def test_append_text_filters_phrase_split_across_chunks() -> None:
    section = _make_section()

    section.append_text("[INFO] Pose set succe")
    assert section.output_text.content == ""
    assert section._terminal_line_buffer == "[INFO] Pose set succe"

    section.append_text("ssfully: pos=(1.0, 2.0, -1.0)\nkeep this line\n")

    assert "Pose set successfully" not in section.output_text.content
    assert "keep this line" in section.output_text.content
