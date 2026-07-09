#!/usr/bin/env python3
"""
generate_sim_objects_yaml.py

Fully automatic: parses a Gazebo .world SDF file and generates a sim_objects
yaml config for sim_detection_publisher.py.

  - Object positions are read from the world file
  - Bounding box sizes are computed from the mesh files using trimesh
  - Class IDs and names are auto-assigned and saved to classes.yaml so the
    same object always gets the same ID across different world files

Usage:
    python3 generate_sim_objects_yaml.py <world_file> [output.yaml]

    If output path is omitted the yaml is printed to stdout.

Requirements:
    pip install trimesh pycollada
"""

import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

# Models that are environment/infrastructure — not competition objects to detect
SKIP_MODELS = {
    "Water",
    "WoollettPool",
    "Task5_Table",
    "North East Down frame",
}
SKIP_PREFIXES = ("Pinger",)


def find_workspace_root(start_path: str) -> str | None:
    """Walk up from start_path until we find a colcon workspace.
    A colcon workspace has src/ alongside build/ or install/."""
    path = os.path.abspath(start_path)
    while path != os.path.dirname(path):
        has_src = os.path.isdir(os.path.join(path, "src"))
        has_build = os.path.isdir(os.path.join(path, "build")) or os.path.isdir(
            os.path.join(path, "install"),
        )
        if has_src and has_build:
            return path
        path = os.path.dirname(path)
    return None


def resolve_uri(uri: str, workspace_root: str) -> str | None:
    """
    Resolve a Gazebo package:// URI to an absolute file path.
    Searches the workspace src/ tree for the file by name.
    """
    uri = uri.strip()
    filename = os.path.basename(uri)
    src_root = os.path.join(workspace_root, "src")
    for dirpath, _, filenames in os.walk(src_root):
        if filename in filenames:
            return os.path.join(dirpath, filename)
    return None


def mesh_half_extents(mesh_path: str) -> list[float] | None:
    """Load a mesh file and return its half-extents [dx, dy, dz] in metres."""
    try:
        import trimesh

        mesh = trimesh.load(mesh_path, force="mesh")
        if mesh is None or not hasattr(mesh, "bounds"):
            # Scene with multiple geometries
            mesh = trimesh.load(mesh_path)
            if hasattr(mesh, "geometry") and mesh.geometry:
                all_bounds = [g.bounds for g in mesh.geometry.values()]
                mins = np.min([b[0] for b in all_bounds], axis=0)
                maxs = np.max([b[1] for b in all_bounds], axis=0)
            else:
                return None
        else:
            mins, maxs = mesh.bounds[0], mesh.bounds[1]

        he = (maxs - mins) / 2
        return [round(float(v), 4) for v in he]
    except Exception as e:
        print(f"  WARNING: trimesh failed on {mesh_path}: {e}", file=sys.stderr)
        return None


def model_to_class_name(model_name: str) -> str:
    """
    Convert a model name like 'StartGate_A', 'RedPole1', 'LeftWhitePole2'
    to a canonical class name like 'start_gate', 'red_pole', 'white_pole'.

    Strips leading Left/Right/Top/Bottom directional prefixes and trailing
    letter/number suffixes that distinguish instances of the same object type.
    """
    name = model_name

    # Strip trailing instance identifiers: _A, _B1, _A2, 1, 2, etc.
    name = re.sub(r"[_\s]+[A-Z][0-9]*$", "", name)
    name = re.sub(r"[0-9]+$", "", name)
    name = re.sub(r"[_\s]+[A-Z][0-9]*$", "", name)  # second pass for _A1 → _A → ""

    # Strip leading directional words
    name = re.sub(r"^(Left|Right|Top|Bottom|Upper|Lower)", "", name)

    # Convert CamelCase to snake_case
    name = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", name)
    name = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", name)
    name = name.lower().strip("_")

    return name


def load_or_create_classes(classes_path: str) -> dict[str, int]:
    """Load existing class name → ID mapping, or return empty dict."""
    if os.path.exists(classes_path):
        with open(classes_path) as f:
            data = yaml.safe_load(f) or {}
        return {k: int(v) for k, v in data.items()}
    return {}


def save_classes(classes_path: str, classes: dict[str, int]) -> None:
    with open(classes_path, "w") as f:
        yaml.dump(dict(sorted(classes.items(), key=lambda x: x[1])), f)


def assign_class_id(class_name: str, classes: dict[str, int]) -> int:
    """Return existing ID for class_name, or assign the next available ID."""
    if class_name not in classes:
        next_id = max(classes.values(), default=-1) + 1
        classes[class_name] = next_id
    return classes[class_name]


def parse_pose(pose_text: str) -> list[float]:
    parts = pose_text.strip().split()
    return [float(parts[0]), float(parts[1]), float(parts[2])]


def get_model_mesh_uri(model_elem) -> str | None:
    """Return the first package:// mesh URI found inside a model element.
    Handles both package:// and the gz-specific package:// variant."""
    for uri_elem in model_elem.iter("uri"):
        text = (uri_elem.text or "").strip()
        if re.match(r"package::?//", text) and any(
            text.endswith(ext) for ext in (".dae", ".obj", ".stl")
        ):
            return text
    return None


def main():
    if len(sys.argv) < 2:
        print("Usage: generate_sim_objects_yaml.py <world_file> [output.yaml]")
        sys.exit(1)

    world_path = os.path.abspath(sys.argv[1])
    output_path = sys.argv[2] if len(sys.argv) > 2 else None

    workspace_root = find_workspace_root(world_path)
    if workspace_root is None:
        print(
            "ERROR: could not find workspace root (no src/ directory found above world file)",
            file=sys.stderr,
        )
        sys.exit(1)

    # classes.yaml lives next to this script's config directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.join(script_dir, "..", "config")
    classes_path = os.path.join(config_dir, "classes.yaml")
    classes = load_or_create_classes(classes_path)

    tree = ET.parse(world_path)
    root = tree.getroot()
    world_elem = root.find("world")
    if world_elem is None:
        print("ERROR: no <world> element found", file=sys.stderr)
        sys.exit(1)

    world_name = world_elem.get("name", "unknown")
    objects = []

    for model in world_elem.findall("model"):
        name = model.get("name", "")

        if name in SKIP_MODELS or any(name.startswith(p) for p in SKIP_PREFIXES):
            continue

        pose_elem = model.find("pose")
        if pose_elem is None or not pose_elem.text:
            continue

        position = parse_pose(pose_elem.text)

        mesh_uri = get_model_mesh_uri(model)
        if mesh_uri is None:
            print(f"  SKIP {name}: no mesh URI found", file=sys.stderr)
            continue

        mesh_path = resolve_uri(mesh_uri, workspace_root)
        if mesh_path is None:
            print(f"  SKIP {name}: could not resolve URI {mesh_uri}", file=sys.stderr)
            continue

        half_extents = mesh_half_extents(mesh_path)
        if half_extents is None:
            print(f"  SKIP {name}: mesh load failed", file=sys.stderr)
            continue

        class_name = model_to_class_name(name)
        class_id = assign_class_id(class_name, classes)

        print(
            f"  {name} → class {class_id} ({class_name}), he={half_extents}",
            file=sys.stderr,
        )
        objects.append((name, position, half_extents, class_id, class_name))

    # Save updated classes
    save_classes(classes_path, classes)

    lines = [
        "---",
        f"# Auto-generated from {os.path.basename(world_path)}",
        f"# Re-generate with: python3 generate_sim_objects_yaml.py {os.path.basename(world_path)}",
        "",
        f'world_name: "{world_name}"',
        'model_name: "sub9"',
        'camera_link: "front_cam_link"',
        f'gz_pose_topic: "/world/{world_name}/dynamic_pose/info"',
        "",
        "objects:",
    ]

    for name, pos, he, class_id, class_name in objects:
        lines += [
            f'  - name: "{name}"',
            f"    position: [{pos[0]}, {pos[1]}, {pos[2]}]",
            f"    half_extents: [{he[0]}, {he[1]}, {he[2]}]",
            f"    class_id: {class_id}",
            f'    class_name: "{class_name}"',
            "",
        ]

    output = "\n".join(lines).rstrip() + "\n"

    if output_path:
        with open(output_path, "w") as f:
            f.write(output)
        print(f"\nWritten to {output_path} ({len(objects)} objects)", file=sys.stderr)
    else:
        print(output)


if __name__ == "__main__":
    main()
