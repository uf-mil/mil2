"""Declarative task configuration for run_task.

No orchestration logic lives here -- only data plus the trivial resolver that
turns a start preset into world coordinates. Every value the harness needs to
bring up, run, and score a task is data, so adding a task is adding a dict entry
rather than writing code.
Budgets are derived from each mission tree's own declared timeouts -- see
docs/superpowers/specs/2026-07-26-run-task-harness-design.md and the guard test
in test/test_btbudget.py, which fails if a tree outgrows its budget.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Union


@dataclass(frozen=True)
class Start:
    """Start pose: horizontal offset from the task anchor, absolute depth."""

    dx: float
    dy: float
    z: float


@dataclass(frozen=True)
class Settle:
    """Convergence gate for the pre-anchor settle (see sim_bringup.py).

    Wall-clock settling is meaningless here: RTF on this box varies ~20x run to
    run, so a fixed wall duration buys 20x different amounts of actual settling.
    Hold until the vehicle is measurably still instead, capped in sim seconds.
    """

    twist_tol: float = 0.02
    stable_samples: int = 10
    cap_sim_s: float = 40.0


@dataclass(frozen=True)
class Stage:
    """One runnable mission tree plus the sim time it is allowed to take.

    `budget_note` documents a budget that was set by hand rather than derived
    from the tree's own timeouts; --list prints it so an unverified number is
    never mistaken for a measured one.
    """

    mission: str
    sim_budget: float
    start: str
    params: dict[str, object] = field(default_factory=dict)
    budget_note: str = ""


@dataclass(frozen=True)
class Detections:
    topic: str
    kind: str = "detections"


@dataclass(frozen=True)
class ModelPose:
    models: tuple[str, ...]
    kind: str = "model_pose"


# typing.Union, not `A | B`: this is a runtime expression, evaluated at import
# even under `from __future__ import annotations`, and ruff.toml targets py38.
Probe = Union[Detections, ModelPose]


@dataclass(frozen=True)
class TaskSpec:
    name: str
    launch_pkg: str
    launch_file: str
    launch_args: dict[str, str]
    anchor: tuple[float, float]
    starts: dict[str, Start]
    settle: Settle
    ready_topics: tuple[str, ...]
    stages: dict[str, Stage]
    default_stage: str
    probes: tuple[Probe, ...]
    scorer: str = ""
    # Mission role parameter (`-p role:=`). Task-specific vocabulary, so it lives
    # here and not as a CLI default; empty means the task takes no role.
    role: str = ""

    def world_start(self, name: str) -> tuple[float, float, float]:
        """Resolve a start preset to absolute world (x, y, z)."""
        s = self.starts[name]
        return (self.anchor[0] + s.dx, self.anchor[1] + s.dy, s.z)


# Task 5 anchor = the table's world x,y in robosub_2025.world. Depth is not part
# of the anchor: every start names its own absolute z, because the useful start
# depth is set by the down cam's view of the table, not by the table's own pose.
# Starts are stored relative to the anchor so they survive the table being moved.
TASKS: dict[int, TaskSpec] = {
    5: TaskSpec(
        name="Octagon: table, grasp, place",
        launch_pkg="subjugator_bringup",
        launch_file="task5_sim.launch.py",
        launch_args={"world": "robosub_2025.world"},
        anchor=(-7.25, 14.0),
        starts={
            # Table already fills the down cam: correct for tuning stages.
            "over_table": Start(dx=-0.45, dy=-0.03, z=-0.35),
            # Outside the initial view but inside S2's reach (DescendUntilDetected
            # covers 2.4 m of descent, the SearchForTarget spiral 2.0 m of radius),
            # so the capstone actually practises finding the table.
            "near": Start(dx=0.0, dy=-1.5, z=-0.35),
        },
        settle=Settle(),
        ready_topics=("/odometry/filtered", "/yolo_down/detections"),
        stages={
            "calib": Stage("CenterCameraTest", 60, "over_table"),
            "combined": Stage("HoneOverTableSelect", 90, "over_table"),
            "grasp": Stage("OctagonGraspMission", 330, "over_table"),
            # do_pinger=0: S1 sweeps forever when it hears no ping, which would
            # make the tree unbounded. The far/pinger start is a documented
            # follow-up, not v1.
            "full": Stage("OctagonMission", 1300, "near", params={"do_pinger": 0}),
        },
        default_stage="full",
        probes=(
            Detections("/yolo_down/detections"),
            ModelPose(
                (
                    "table",
                    "electric_box",
                    "nut_cylinder",
                    "pill_cylinder",
                    "bandaid_box",
                ),
            ),
        ),
        scorer="subjugator_bringup.task_runner.scorers.task5",
        role="survey_repair",
    ),
}
