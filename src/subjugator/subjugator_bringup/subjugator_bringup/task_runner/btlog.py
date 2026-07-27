"""Parse a mission_planner console.log into an outcome and a stage table.

Two line shapes matter. BT.CPP's StdCoutLogger prints transitions:

    [1783913900.456]: Sequence   RUNNING -> SUCCESS      (with ANSI colouring)

and the mission node prints its verdict exactly once:

    [INFO] [...] [mission_planner]: Mission finished with status: SUCCESS.

The verdict line is the source of truth for the outcome: the process exits 0
whether the mission succeeded or failed, so the exit code cannot be used.

Timestamps are absolute Unix seconds on both line shapes, which is what lets the
runner map them onto sim time using its own /clock samples.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

ANSI = re.compile(r"\x1b\[[0-9;]*m")
TRANSITION = re.compile(r"^\[(\d+\.\d+)\]:\s+(\S+)\s+(\w+)\s*->\s*(\w+)\s*$")
VERDICT = re.compile(r"Mission finished with status:\s*(SUCCESS|FAILURE)")
TERMINAL = ("SUCCESS", "FAILURE")


@dataclass
class Stage:
    name: str
    occurrence: int
    result: str
    start_wall: float
    end_wall: float

    @property
    def duration(self) -> float:
        return round(self.end_wall - self.start_wall, 3)


@dataclass
class ParsedLog:
    outcome: str | None = None
    last_node: str | None = None
    start_wall: float | None = None
    end_wall: float | None = None
    stages: list = field(default_factory=list)


def parse(text: str, tree_ids: set) -> ParsedLog:
    """Extract outcome and per-stage records. `tree_ids` selects which node
    names count as stages -- pass the mission XML's BehaviorTree IDs so the
    parser stays task-agnostic."""
    result = ParsedLog()
    open_stages: dict = {}
    counts: dict = {}

    for raw in text.splitlines():
        line = ANSI.sub("", raw).rstrip()

        verdict = VERDICT.search(line)
        if verdict:
            result.outcome = verdict.group(1)
            continue

        match = TRANSITION.match(line)
        if not match:
            continue

        ts = float(match.group(1))
        name = match.group(2)
        new = match.group(4)

        if result.start_wall is None:
            result.start_wall = ts
        result.end_wall = ts

        if name not in tree_ids:
            continue

        if new == "RUNNING":
            open_stages[name] = ts
        elif new in TERMINAL:
            started = open_stages.pop(name, ts)
            counts[name] = counts.get(name, 0) + 1
            result.stages.append(
                Stage(
                    name=name,
                    occurrence=counts[name],
                    result=new,
                    start_wall=started,
                    end_wall=ts,
                ),
            )
            result.last_node = name

    return result


def parse_file(path: str, tree_ids: set) -> ParsedLog:
    with open(path, errors="replace") as handle:
        return parse(handle.read(), tree_ids)
