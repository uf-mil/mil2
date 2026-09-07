"""Task 5 ground-truth verdict.

The generic layer can only report facts computable without task knowledge --
"electric_box moved 1.42 m and ended at (-7.1, 14.3, -1.9)". Deciding that this
counts as placed in the correct basket needs Task 5 semantics, so that judgment
lives here and never leaks into the core collectors.

Why bother when the behaviour tree already reports SUCCESS or FAILURE: the tree's
verdict comes from the tree's own perception and internal state. This one comes
from gz truth. They disagree exactly where it matters -- ConfirmGraspByScale
reporting a grasp when the box never left the table, or S5 reporting a place when
the object is on the floor beside the basket.

Geometry is derived, not measured by hand: `bin` and `bin.001` are named objects
inside table_2026/table.obj, and the table is static at (-7.25, 14, -1.7) with
identity rotation and unit scale. test_basket_constants.py re-derives every
number below from the mesh and fails on drift.
"""

from __future__ import annotations

from dataclasses import dataclass

TABLE_WORLD = (-7.25, 14.0, -1.7)

# Thresholds. Deliberately generous: this is a "did it physically happen" check,
# not a precision measurement.
MOVED_M = 0.05
LIFTED_M = 0.10
FOOTPRINT_TOL_M = 0.02


@dataclass(frozen=True)
class Basket:
    marker: str
    center: tuple  # world (x, y)
    half: tuple  # (x, y) half-extents of the well
    rim_z: float  # world z of the well rim; inside means below this


BASKETS = {
    # bin / Plane.001, textured Task5_Warning.png
    "warning": Basket("warning", (-7.25, 13.600), (0.133, 0.092), -0.839),
    # bin.001 / Plane.002, textured Task5_RedCross.png
    "red_cross": Basket("red_cross", (-7.25, 14.400), (0.133, 0.092), -0.839),
}

# Mirrors select_basket_logic.hpp.
ROLE_BASKET = {"survey_repair": "warning", "search_rescue": "red_cross"}

# Mirrors select_target's role -> label mapping.
ROLE_TARGETS = {
    "survey_repair": ("nut_cylinder", "electric_box"),
    "search_rescue": ("pill_cylinder", "bandaid_box"),
}


def basket_for_role(role: str) -> Basket:
    return BASKETS[ROLE_BASKET[role]]


def _inside(basket: Basket, end: tuple) -> bool:
    x, y, z = end
    return (
        abs(x - basket.center[0]) <= basket.half[0] + FOOTPRINT_TOL_M
        and abs(y - basket.center[1]) <= basket.half[1] + FOOTPRINT_TOL_M
        and z <= basket.rim_z
    )


def _verdict(model: dict, role_basket: Basket) -> str:
    if not model.get("present"):
        return "absent"
    if model["max_displacement"] < MOVED_M:
        return "untouched"
    if _inside(role_basket, model["end"]):
        return "placed"
    for basket in BASKETS.values():
        if basket.marker != role_basket.marker and _inside(basket, model["end"]):
            return "wrong_basket"
    if model["max_z_rise"] >= LIFTED_M:
        return "dropped"
    return "moved"


def score(role: str, model_summary: dict, bt_outcome: str | None) -> dict:
    """Physical verdict for a Task 5 run, independent of the tree's own opinion."""
    role_basket = basket_for_role(role)
    targets = ROLE_TARGETS.get(role, ())

    objects = {}
    for name in targets:
        model = model_summary.get(name, {"present": False})
        objects[name] = {
            "verdict": _verdict(model, role_basket),
            "displacement": round(model.get("max_displacement", 0.0), 3),
            "max_z_rise": round(model.get("max_z_rise", 0.0), 3),
            "end": model.get("end"),
        }

    placed = sum(1 for o in objects.values() if o["verdict"] == "placed")

    return {
        "role": role,
        "basket": role_basket.marker,
        "basket_center": role_basket.center,
        "objects": objects,
        "placed": placed,
        "targets": len(targets),
        # The loud case: the tree declared victory but nothing physically landed.
        "disagrees_with_bt": bt_outcome == "SUCCESS" and placed == 0,
    }
