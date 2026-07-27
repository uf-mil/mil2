"""Re-derive the scorer's basket geometry from the mesh and the world file.

If the table is re-modelled or moved, this test fails instead of the scorer
quietly producing wrong verdicts.

Paths resolve from this file, not from the CWD: `colcon test` runs pytest with
the package source dir as the working directory, `python3 -m pytest ...` from
the repo root, and both have to find the same mesh.
"""

import re
from pathlib import Path

import pytest
from helpers import objgroups
from subjugator_bringup.task_runner.scorers import task5


def _repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "src" / "subjugator" / "simulation").is_dir():
            return parent
    raise AssertionError("cannot locate the repo root from " + __file__)


ROOT = _repo_root()
MODEL_DIR = ROOT / "src/subjugator/simulation/subjugator_description/models/table_2026"
OBJ = str(MODEL_DIR / "table.obj")
MTL = str(MODEL_DIR / "table.mtl")
WORLD = str(
    ROOT / "src/subjugator/simulation/subjugator_gazebo/worlds/robosub_2025.world",
)
OPERATIONS = ROOT / "src/subjugator/mission_planner/subjugator_operations"

# Mesh object -> the marker plane sitting inside it, by shared y position.
BINS = ("bin", "bin.001")

# The decal texture is what actually assigns a marker to a bin, so the mapping
# is spelled out here and looked up -- never inferred from bin order or sign.
TEXTURE_MARKER = {
    "Task5_Warning.png": "warning",
    "Task5_RedCross.png": "red_cross",
}

PROPS = ("electric_box", "nut_cylinder", "pill_cylinder", "bandaid_box")


@pytest.fixture(scope="module")
def groups():
    return objgroups.read_groups(OBJ)


@pytest.fixture(scope="module")
def derived(groups):
    textures = objgroups.material_textures(MTL)
    tx, ty, tz = objgroups.table_pose(WORLD)

    out = {}
    for bin_name in BINS:
        bin_group = groups[bin_name]
        # The marker plane inside this bin is the Plane whose centre y matches.
        planes = [
            g
            for name, g in groups.items()
            if name.startswith("Plane")
            and abs(g["center"][1] - bin_group["center"][1]) < 1e-3
        ]
        assert len(planes) == 1, f"{bin_name} matched {len(planes)} marker planes"
        texture = textures.get(planes[0]["material"])
        assert texture in TEXTURE_MARKER, (
            f"{bin_name}'s plane uses material {planes[0]['material']!r} "
            f"-> texture {texture!r}, which names no known marker"
        )
        marker = TEXTURE_MARKER[texture]
        out[marker] = {
            "center": (tx + bin_group["center"][0], ty + bin_group["center"][1]),
            "half": (bin_group["half"][0], bin_group["half"][1]),
            "rim_z": tz + bin_group["max"][2],
        }
    assert set(out) == set(
        TEXTURE_MARKER.values(),
    ), f"the two bins derived to {sorted(out)}, not one marker each"
    return out


def test_table_pose_matches_the_scorer():
    assert objgroups.table_pose(WORLD) == pytest.approx(task5.TABLE_WORLD)


@pytest.mark.parametrize("marker", ["warning", "red_cross"])
def test_basket_center_matches(marker, derived):
    assert task5.BASKETS[marker].center == pytest.approx(
        derived[marker]["center"],
        abs=1e-3,
    )


@pytest.mark.parametrize("marker", ["warning", "red_cross"])
def test_basket_extents_match(marker, derived):
    assert task5.BASKETS[marker].half == pytest.approx(
        derived[marker]["half"],
        abs=1e-3,
    )


@pytest.mark.parametrize("marker", ["warning", "red_cross"])
def test_basket_rim_matches(marker, derived):
    assert task5.BASKETS[marker].rim_z == pytest.approx(
        derived[marker]["rim_z"],
        abs=1e-3,
    )


def test_markers_are_on_opposite_bins(derived):
    assert derived["warning"]["center"][1] != derived["red_cross"]["center"][1]


# --- the derivation itself, and the assumptions it rests on -------------
#
# Everything above compares a derived number against a constant. The tests
# below check that the derivation could have come out differently: that the
# marker assignment follows the textures rather than the bin order, that the
# mesh really does load Z-up and unrotated, and that the two bins are distinct
# volumes rather than one bounding box counted twice.


def test_marker_assignment_follows_the_texture_not_the_bin_order(groups):
    # The decisive fact: `bin` (the low-y one) carries the Warning decal and
    # `bin.001` the RedCross one. Swap the two PNGs in table.mtl and this is
    # the assertion that fails.
    textures = objgroups.material_textures(MTL)
    by_bin = {}
    for bin_name in BINS:
        bin_y = groups[bin_name]["center"][1]
        plane = next(
            g
            for name, g in groups.items()
            if name.startswith("Plane") and abs(g["center"][1] - bin_y) < 1e-3
        )
        by_bin[bin_name] = TEXTURE_MARKER[textures[plane["material"]]]

    assert by_bin == {"bin": "warning", "bin.001": "red_cross"}


def test_the_warning_basket_is_the_low_y_one(derived):
    # Which bin is which decides every wrong_basket verdict, so pin the sign.
    assert derived["warning"]["center"][1] < derived["red_cross"]["center"][1]
    assert task5.BASKETS["warning"].center[1] < task5.BASKETS["red_cross"].center[1]


def test_the_two_bins_are_separate_volumes(groups):
    low, high = groups["bin"], groups["bin.001"]
    assert low["max"][1] < high["min"][1]
    # ... and both wells sit inside the table body, i.e. these are two recesses
    # in one table rather than unrelated geometry that happens to be named bin.
    body = groups["model"]
    for well in (low, high):
        for axis in range(3):
            assert body["min"][axis] <= well["min"][axis]
            assert well["max"][axis] <= body["max"][axis]


def test_the_mesh_loads_z_up_and_unrotated(groups, derived):
    # Two independent cross-checks that the mesh is not rotated in the world:
    # the computed table top has to sit just under the props' authored rest
    # height, and the props have to land between the two bins in y. Both use
    # the derived geometry, so a rotated mesh cannot hide behind the constants.
    tx, _ty, tz = objgroups.table_pose(WORLD)
    table_top = tz + groups["model"]["max"][2]
    for prop in PROPS:
        px, py, pz = objgroups.table_pose(WORLD, prop)
        gap = pz - table_top
        assert 0.0 < gap < 0.05, f"{prop} rests {gap:.3f} m off the table top"
        assert (
            derived["warning"]["center"][1] < py < derived["red_cross"]["center"][1]
        ), f"{prop} does not start between the two bins"
        assert abs(px - tx) < 0.5


def test_the_rim_is_above_the_well_floor(groups, derived):
    # rim_z is the top of the bin, not its bottom: an object "inside" is below
    # the rim, so getting these two confused would make _inside vacuous.
    for bin_name, marker in (("bin", "warning"), ("bin.001", "red_cross")):
        _tx, _ty, tz = objgroups.table_pose(WORLD)
        floor = tz + groups[bin_name]["min"][2]
        assert derived[marker]["rim_z"] > floor
        assert task5.BASKETS[marker].rim_z > floor


# --- the constants that mirror the mission source -----------------------


def _port_default(source: Path, port: str) -> str:
    text = source.read_text()
    match = re.search(rf'InputPort<std::string>\("{port}",\s*"([^"]*)"', text)
    assert match, f"no default for port '{port}' in {source}"
    return match.group(1)


def test_role_to_basket_mirrors_select_basket():
    src = OPERATIONS / "src/select_basket.cpp"
    assert task5.ROLE_BASKET["survey_repair"] == _port_default(src, "survey_marker")
    assert task5.ROLE_BASKET["search_rescue"] == _port_default(src, "rescue_marker")


def test_role_to_targets_mirrors_select_target():
    src = OPERATIONS / "src/select_target.cpp"
    survey = _port_default(src, "survey_labels").split(",")
    rescue = _port_default(src, "rescue_labels").split(",")
    assert sorted(task5.ROLE_TARGETS["survey_repair"]) == sorted(survey)
    assert sorted(task5.ROLE_TARGETS["search_rescue"]) == sorted(rescue)


def test_the_scorer_knows_both_roles_select_basket_knows():
    logic = (OPERATIONS / "include/select_basket_logic.hpp").read_text()
    roles = set(re.findall(r'role == "([a-z_]+)"', logic))
    assert roles == set(task5.ROLE_BASKET)


# --- the reader's own contract ------------------------------------------
#
# The table mesh exercises only the happy path (every object has faces and one
# material, every face index is absolute), so these two pin the parts of the
# reader that the real file cannot.


def test_a_group_with_no_faces_is_left_out(tmp_path):
    obj = tmp_path / "stub.obj"
    obj.write_text(
        """o empty
o solid
v 0 0 0
v 2 4 6
usemtl Paint
f 1 2 1
""",
    )
    groups = objgroups.read_groups(str(obj))
    assert set(groups) == {"solid"}
    assert groups["solid"]["center"] == (1.0, 2.0, 3.0)
    assert groups["solid"]["half"] == (1.0, 2.0, 3.0)
    assert groups["solid"]["material"] == "Paint"


def test_negative_face_indices_count_back_from_the_last_vertex(tmp_path):
    obj = tmp_path / "relative.obj"
    obj.write_text(
        "v 100 100 100\n"  # an earlier object's vertex, not referenced
        "o solid\n"
        "v 0 0 0\n"
        "v 2 4 6\n"
        "f -2 -1 -2\n",
    )
    groups = objgroups.read_groups(str(obj))
    assert groups["solid"]["min"] == (0.0, 0.0, 0.0)
    assert groups["solid"]["max"] == (2.0, 4.0, 6.0)
