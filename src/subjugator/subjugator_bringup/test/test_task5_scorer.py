import pytest
from subjugator_bringup.task_runner.scorers import task5


def placed_summary():
    return {
        "electric_box": {
            "present": True,
            "start": (-7.10, 13.89, -0.80),
            "end": (-7.25, 13.60, -0.90),  # inside the warning basket well
            "displacement": 0.33,
            "max_displacement": 0.33,
            "max_z_rise": 0.25,
        },
        "nut_cylinder": {
            "present": True,
            "start": (-7.30, 13.90, -0.80),
            "end": (-7.30, 13.90, -0.80),
            "displacement": 0.0,
            "max_displacement": 0.0,
            "max_z_rise": 0.0,
        },
    }


def test_role_to_basket():
    assert task5.basket_for_role("survey_repair").marker == "warning"
    assert task5.basket_for_role("search_rescue").marker == "red_cross"


def test_object_in_the_role_basket_is_placed():
    result = task5.score("survey_repair", placed_summary(), bt_outcome="SUCCESS")
    assert result["objects"]["electric_box"]["verdict"] == "placed"
    assert result["placed"] == 1


def test_untouched_object():
    result = task5.score("survey_repair", placed_summary(), bt_outcome="SUCCESS")
    assert result["objects"]["nut_cylinder"]["verdict"] == "untouched"


def test_lifted_but_released_outside_is_dropped():
    summary = placed_summary()
    summary["electric_box"]["end"] = (-6.50, 13.60, -1.60)
    result = task5.score("survey_repair", summary, bt_outcome="SUCCESS")
    assert result["objects"]["electric_box"]["verdict"] == "dropped"
    assert result["placed"] == 0


def test_wrong_basket_is_not_placed():
    summary = placed_summary()
    summary["electric_box"]["end"] = (-7.25, 14.40, -0.90)  # red_cross basket
    result = task5.score("survey_repair", summary, bt_outcome="SUCCESS")
    assert result["objects"]["electric_box"]["verdict"] == "wrong_basket"


def nothing_happened_summary():
    # The plan's version left max_displacement at 0.33, so "nothing placed"
    # held for the wrong reason (the box counted as moved-but-not-lifted).
    # Zeroing every motion field makes the fixture mean what its name says.
    summary = placed_summary()
    summary["electric_box"]["end"] = summary["electric_box"]["start"]
    summary["electric_box"]["max_z_rise"] = 0.0
    summary["electric_box"]["displacement"] = 0.0
    summary["electric_box"]["max_displacement"] = 0.0
    return summary


def test_bt_success_with_nothing_placed_disagrees():
    result = task5.score(
        "survey_repair",
        nothing_happened_summary(),
        bt_outcome="SUCCESS",
    )
    assert result["placed"] == 0
    assert result["disagrees_with_bt"] is True


def test_bt_failure_with_nothing_placed_agrees():
    result = task5.score(
        "survey_repair",
        nothing_happened_summary(),
        bt_outcome="FAILURE",
    )
    assert result["placed"] == 0
    assert result["disagrees_with_bt"] is False


# --- strengthened tests --------------------------------------------------
#
# The plan's fixtures exercise one point per verdict, all of them comfortably
# inside or outside a basket. Several branches of the ladder survive that: the
# rim check, the footprint tolerance on each axis independently, whether the
# halves are read x-then-y, and the "moved" and "absent" rungs. The tests below
# put a point on each side of every boundary the ladder actually uses.

WARNING = task5.BASKETS["warning"]
RED_CROSS = task5.BASKETS["red_cross"]
X_BOUND = WARNING.half[0] + task5.FOOTPRINT_TOL_M  # 0.153
Y_BOUND = WARNING.half[1] + task5.FOOTPRINT_TOL_M  # 0.112
IN_WELL_Z = WARNING.rim_z - 0.05


def at(dx=0.0, dy=0.0, z=IN_WELL_Z, basket=WARNING):
    """A world point offset from a basket centre."""
    return (basket.center[0] + dx, basket.center[1] + dy, z)


def lifted(end, **overrides):
    """A model that was picked up and released at `end`."""
    model = {
        "present": True,
        "start": (-7.10, 13.89, -0.80),
        "end": end,
        "displacement": 0.33,
        "max_displacement": 0.33,
        "max_z_rise": 0.25,
    }
    model.update(overrides)
    return model


def verdict_of(model, role="survey_repair"):
    result = task5.score(role, {"electric_box": model}, bt_outcome=None)
    return result["objects"]["electric_box"]["verdict"]


# -- the rim: inside means below it, not merely over the footprint --------


def test_an_object_left_on_top_of_the_basket_is_not_placed():
    # x/y dead centre, but resting at the props' table height, above the rim.
    # This is the "shoved it across the table" failure, and it must not score.
    assert verdict_of(lifted(at(z=-0.80))) == "dropped"


def test_the_rim_is_the_boundary():
    assert verdict_of(lifted(at(z=WARNING.rim_z))) == "placed"
    assert verdict_of(lifted(at(z=WARNING.rim_z + 0.001))) == "dropped"


def test_deep_in_the_well_is_still_placed():
    assert verdict_of(lifted(at(z=WARNING.rim_z - 0.04))) == "placed"


# -- the footprint, one axis at a time -----------------------------------


# A hair inside/outside the boundary rather than exactly on it: the offsets go
# through a world-frame addition, so an exact-boundary point lands a couple of
# ULPs either side and the test would pass or fail on rounding luck.
EPS = 1e-6


def test_the_x_footprint_boundary():
    assert verdict_of(lifted(at(dx=X_BOUND - EPS))) == "placed"
    assert verdict_of(lifted(at(dx=X_BOUND + 0.001))) == "dropped"
    assert verdict_of(lifted(at(dx=-X_BOUND + EPS))) == "placed"
    assert verdict_of(lifted(at(dx=-X_BOUND - 0.001))) == "dropped"


def test_the_y_footprint_boundary():
    assert verdict_of(lifted(at(dy=Y_BOUND - EPS))) == "placed"
    assert verdict_of(lifted(at(dy=Y_BOUND + 0.001))) == "dropped"
    assert verdict_of(lifted(at(dy=-Y_BOUND + EPS))) == "placed"
    assert verdict_of(lifted(at(dy=-Y_BOUND - 0.001))) == "dropped"


def test_the_footprint_is_not_square():
    # The well is wider in x than in y. A point 0.14 m out is inside along x
    # and outside along y; reading the halves in the wrong order flips both.
    assert verdict_of(lifted(at(dx=0.14))) == "placed"
    assert verdict_of(lifted(at(dy=0.14))) == "dropped"


def test_the_tolerance_is_applied_to_both_axes():
    # Just past the mesh half-extent on each axis, inside the tolerance.
    assert verdict_of(lifted(at(dx=WARNING.half[0] + 0.01))) == "placed"
    assert verdict_of(lifted(at(dy=WARNING.half[1] + 0.01))) == "placed"


# -- the rest of the ladder ----------------------------------------------


def test_an_object_that_moved_without_rising_is_moved():
    # Nudged off its start but never lifted and nowhere near a basket.
    assert verdict_of(lifted((-7.10, 13.30, -0.80), max_z_rise=0.0)) == "moved"


def test_the_lift_threshold_separates_moved_from_dropped():
    outside = (-6.50, 13.60, -1.60)
    assert verdict_of(lifted(outside, max_z_rise=task5.LIFTED_M)) == "dropped"
    assert verdict_of(lifted(outside, max_z_rise=task5.LIFTED_M - 0.001)) == "moved"


def test_the_movement_threshold_separates_untouched_from_moved():
    outside = (-6.50, 13.60, -1.60)
    assert (
        verdict_of(lifted(outside, max_displacement=task5.MOVED_M - 0.001))
        == "untouched"
    )
    assert verdict_of(lifted(outside, max_displacement=task5.MOVED_M)) == "dropped"


def test_displacement_is_the_peak_not_the_net():
    # Grabbed, carried, and dropped back where it started: net displacement is
    # zero, but the run is not "untouched" and the report must say 1.4 m.
    model = lifted((-7.10, 13.89, -0.80), displacement=0.0, max_displacement=1.4)
    result = task5.score("survey_repair", {"electric_box": model}, bt_outcome=None)
    box = result["objects"]["electric_box"]
    assert box["verdict"] == "dropped"
    assert box["displacement"] == pytest.approx(1.4)


def test_an_object_that_never_moved_is_untouched_even_inside_a_basket():
    # A prop authored inside the well was not carried there by the sub, so the
    # displacement rung has to come before the geometry.
    still = lifted(at(), max_displacement=0.0, max_z_rise=0.0)
    still["start"] = still["end"]
    assert verdict_of(still) == "untouched"


def test_an_absent_model_is_absent():
    result = task5.score(
        "survey_repair",
        {"electric_box": {"present": False}},
        bt_outcome=None,
    )
    box = result["objects"]["electric_box"]
    assert box["verdict"] == "absent"
    assert box["end"] is None
    assert box["displacement"] == 0.0


def test_a_model_the_probe_never_saw_is_absent():
    result = task5.score("survey_repair", {}, bt_outcome=None)
    assert result["objects"]["electric_box"]["verdict"] == "absent"
    assert result["objects"]["nut_cylinder"]["verdict"] == "absent"


# -- wrong_basket --------------------------------------------------------


def test_wrong_basket_needs_no_lift():
    # Slid into the other well without ever being raised: still the wrong
    # basket, not "moved" -- the basket branches sit above the lift check.
    slid = lifted(at(basket=RED_CROSS), max_z_rise=0.0)
    assert verdict_of(slid) == "wrong_basket"


def test_wrong_basket_respects_the_rim_too():
    assert verdict_of(lifted(at(z=-0.80, basket=RED_CROSS))) == "dropped"


def test_wrong_basket_is_symmetric_between_roles():
    in_warning = lifted(at(basket=WARNING))
    result = task5.score(
        "search_rescue",
        {"pill_cylinder": in_warning},
        bt_outcome=None,
    )
    assert result["objects"]["pill_cylinder"]["verdict"] == "wrong_basket"


# -- roles ---------------------------------------------------------------


def test_the_role_picks_both_the_basket_and_the_targets():
    result = task5.score("search_rescue", {}, bt_outcome=None)
    assert result["role"] == "search_rescue"
    assert result["basket"] == "red_cross"
    assert result["basket_center"] == RED_CROSS.center
    assert set(result["objects"]) == {"pill_cylinder", "bandaid_box"}
    assert result["targets"] == 2


def test_the_survey_role_reports_its_own_basket():
    # Both roles have to be checked: a scorer that always reported the
    # red_cross centre would pass the search_rescue case above.
    result = task5.score("survey_repair", {}, bt_outcome=None)
    assert result["role"] == "survey_repair"
    assert result["basket"] == "warning"
    assert result["basket_center"] == WARNING.center
    assert set(result["objects"]) == {"nut_cylinder", "electric_box"}


def test_the_target_count_follows_the_role(monkeypatch):
    # `targets` is the denominator in "1 of 2 placed", so it has to be the
    # number of objects this role asks for, not a 2 that is right by accident.
    monkeypatch.setitem(task5.ROLE_TARGETS, "survey_repair", ("electric_box",))
    result = task5.score("survey_repair", {}, bt_outcome=None)
    assert result["targets"] == 1
    assert set(result["objects"]) == {"electric_box"}


def test_objects_outside_the_role_are_not_scored():
    # The other role's props are in the same summary (the probe watches all
    # four); scoring them would inflate `placed`.
    summary = {
        "electric_box": lifted(at()),
        "pill_cylinder": lifted(at(basket=RED_CROSS)),
    }
    result = task5.score("survey_repair", summary, bt_outcome=None)
    assert set(result["objects"]) == {"nut_cylinder", "electric_box"}
    assert result["placed"] == 1


def test_an_unknown_role_is_an_error_not_a_silent_zero():
    with pytest.raises(KeyError):
        task5.score("nonsense", {}, bt_outcome="SUCCESS")


def test_both_objects_in_the_basket_score_two():
    summary = {
        "electric_box": lifted(at(dx=0.05)),
        "nut_cylinder": lifted(at(dx=-0.05)),
    }
    result = task5.score("survey_repair", summary, bt_outcome="SUCCESS")
    assert result["placed"] == 2
    assert result["targets"] == 2
    assert result["disagrees_with_bt"] is False


# -- the disagreement flag -----------------------------------------------


def test_success_with_something_placed_agrees():
    result = task5.score("survey_repair", placed_summary(), bt_outcome="SUCCESS")
    assert result["disagrees_with_bt"] is False


def test_failure_with_something_placed_does_not_disagree():
    # The flag is deliberately one-sided: it fires only on the loud case, a
    # tree claiming victory over nothing.
    result = task5.score("survey_repair", placed_summary(), bt_outcome="FAILURE")
    assert result["placed"] == 1
    assert result["disagrees_with_bt"] is False


@pytest.mark.parametrize("outcome", [None, "", "RUNNING", "TIMEOUT", "success"])
def test_only_a_success_outcome_can_disagree(outcome):
    result = task5.score(
        "survey_repair",
        nothing_happened_summary(),
        bt_outcome=outcome,
    )
    assert result["disagrees_with_bt"] is False


# -- reporting -----------------------------------------------------------


def test_reported_numbers_are_rounded_but_the_end_pose_is_verbatim():
    model = lifted(
        (-7.123456, 13.6, -0.9),
        max_displacement=0.123456,
        max_z_rise=0.98765,
    )
    box = task5.score("survey_repair", {"electric_box": model}, bt_outcome=None)[
        "objects"
    ]["electric_box"]
    assert box["displacement"] == 0.123
    assert box["max_z_rise"] == 0.988
    assert box["end"] == (-7.123456, 13.6, -0.9)


def test_scoring_does_not_mutate_the_summary():
    summary = placed_summary()
    before = {name: dict(model) for name, model in summary.items()}
    task5.score("survey_repair", summary, bt_outcome="SUCCESS")
    assert summary == before
