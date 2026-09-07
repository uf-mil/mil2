"""Minimal OBJ reader: per-object bounding boxes and material assignment.

Test-only. Exists so the scorer's basket constants can be re-derived from the
mesh rather than trusted, which is what makes a re-modelled table a failing test
instead of a silently wrong verdict.

Assumes one material per object, which is how the table is authored; if an
object ever carried several, the last `usemtl` would win and `material` would
become a half-truth.
"""

from __future__ import annotations

import re


def read_groups(path: str) -> dict:
    """name -> {"center": (x, y, z), "half": (x, y, z), "min"/"max", "material"}."""
    verts: list = []
    groups: dict = {}
    current = None

    with open(path, errors="replace") as handle:
        for line in handle:
            if line.startswith("v "):
                verts.append(tuple(float(v) for v in line.split()[1:4]))
            elif line.startswith("o "):
                current = line[2:].strip()
                groups[current] = {"idx": set(), "material": None}
            elif line.startswith("usemtl ") and current:
                groups[current]["material"] = line.split(maxsplit=1)[1].strip()
            elif line.startswith("f ") and current:
                for token in line.split()[1:]:
                    i = int(token.split("/")[0])
                    groups[current]["idx"].add(i - 1 if i > 0 else len(verts) + i)

    out = {}
    for name, data in groups.items():
        if not data["idx"]:
            continue
        pts = [verts[i] for i in data["idx"]]
        lo = tuple(min(p[a] for p in pts) for a in range(3))
        hi = tuple(max(p[a] for p in pts) for a in range(3))
        out[name] = {
            "min": lo,
            "max": hi,
            "center": tuple((lo[a] + hi[a]) / 2 for a in range(3)),
            "half": tuple((hi[a] - lo[a]) / 2 for a in range(3)),
            "material": data["material"],
        }
    return out


def material_textures(path: str) -> dict:
    """material name -> texture filename, from a .mtl file."""
    textures: dict = {}
    current = None
    with open(path, errors="replace") as handle:
        for line in handle:
            if line.startswith("newmtl "):
                current = line.split(maxsplit=1)[1].strip()
            elif line.startswith("map_Kd ") and current:
                textures[current] = line.split(maxsplit=1)[1].strip()
    return textures


def table_pose(world_path: str, model: str = "table") -> tuple:
    """Read a model's <pose> out of a .world file."""
    with open(world_path, errors="replace") as handle:
        text = handle.read()
    block = re.search(
        rf"<model name='{model}'>(.*?)</model>",
        text,
        re.DOTALL,
    )
    if not block:
        raise AssertionError(f"no <model name='{model}'> in {world_path}")
    pose = re.search(r"<pose>([^<]+)</pose>", block.group(1))
    if not pose:
        raise AssertionError(f"model '{model}' has no <pose>")
    nums = [float(v) for v in pose.group(1).split()]
    return tuple(nums[:3])
