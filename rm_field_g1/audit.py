"""Inventory the locally checked RMUC field without modifying source assets.

This deliberately audits the inspected C0 structure. Unsupported additional
models/transforms fail rather than silently producing an incomplete inventory.
"""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT.parents[1] / "rm_uav_lab/rm27_uav_sim/src/rm27_field"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(source):
    world_path = source / "worlds/rmuc_2025_world.sdf"
    model_path = source / "models/rmuc_2025/model.sdf"
    mesh_path = source / "models/rmuc_2025/meshes/rmuc_2025.stl"
    world = ET.parse(world_path).getroot().find("world")
    model = ET.parse(model_path).getroot().find("model")
    link = model.find("link")
    assert len(world.findall("model")) == 1
    assert len(model.findall("link")) == 1
    include = world.find("model/include")
    assert include.findtext("uri") == "model://rmuc_2025"
    assert not world.findall("include")
    assert np.allclose(np.fromstring(include.findtext("pose"), sep=" "), 0)
    assert world.find("model/pose") is None and model.find("pose") is None
    pose_element = link.find("pose")
    assert not pose_element.get("frame") and not pose_element.get("relative_to")
    link_pose = np.fromstring(pose_element.text, sep=" ")
    assert np.allclose(link_pose[3:], 0)
    collisions = []
    for collision in link.findall("collision"):
        pose = np.fromstring(collision.findtext("pose", "0 0 0 0 0 0"), sep=" ")
        assert np.allclose(pose[3:], 0)
        box = collision.find("geometry/box")
        assert box is not None, "New geometry requires audit extension"
        size = np.fromstring(box.findtext("size"), sep=" ")
        center = link_pose[:3] + pose[:3]
        collisions.append(
            {
                "name": collision.get("name"),
                "geometry": "box",
                "local_pose_xyz_rpy": pose.tolist(),
                "size_m": size.tolist(),
                "world_center_enu_m": center.tolist(),
                "world_aabb_enu_m": [list(center - size / 2), list(center + size / 2)],
                "surface_override": collision.find("surface") is not None,
            }
        )
    visual = link.find("visual")
    assert len(link.findall("visual")) == 1 and visual.find("pose") is None
    scale = np.fromstring(visual.findtext("geometry/mesh/scale"), sep=" ")
    mesh = trimesh.load(mesh_path, force="mesh")
    # No repair: count and bounds must describe the actual source.
    parts = mesh.split(only_watertight=False, repair=False)
    components = []
    for index, part in enumerate(parts):
        components.append(
            {
                "component_index": index,
                "faces": len(part.faces),
                "watertight": bool(part.is_watertight),
                "mesh_local_aabb_m": part.bounds.tolist(),
                "world_aabb_enu_m": (part.bounds * scale + link_pose[:3]).tolist(),
                "semantic_name": None,
            }
        )
    files = [p for p in source.rglob("*") if p.is_file()]
    return {
        "schema_version": 1,
        "source_root": str(source.resolve()),
        "files": [
            {
                "path": str(p.relative_to(source)),
                "bytes": p.stat().st_size,
                "sha256": digest(p),
            }
            for p in sorted(files)
        ],
        "world": {
            "name": world.get("name"),
            "sdf_version": "1.9",
            "plugins": [
                ET.tostring(p, encoding="unicode") for p in world.findall("plugin")
            ],
            "physics_element": world.find("physics") is not None,
            "note": "max_step_size occurs inside Physics plugin; no world physics element",
            "static": True,
            "moving_components": [],
            "include_uri": include.findtext("uri"),
            "include_pose_xyz_rpy": [0] * 6,
            "link_pose_xyz_rpy": link_pose.tolist(),
        },
        "visual": {
            "uri": visual.findtext("geometry/mesh/uri"),
            "scale": scale.tolist(),
            "faces": len(mesh.faces),
            "vertices_after_duplicate_merge": len(mesh.vertices),
            "watertight": bool(mesh.is_watertight),
            "material_xml": ET.tostring(visual.find("material"), encoding="unicode"),
            "textures": [],
            "world_aabb_enu_m": (mesh.bounds * scale + link_pose[:3]).tolist(),
            "connected_components": components,
        },
        "collisions": collisions,
        "findings": [
            "Only floor and four perimeter colliders; all interior mesh geometry is visual-only.",
            "No named base, outpost, ramp or central obstacle entities in SDF; mesh parts lack semantic labels.",
            "Visual floor is 29 x 16 m; collision floor is 30 x 18 m, top z=0.2 m.",
            "Visual perimeter reaches world z≈18.7546 m; perimeter colliders end at z=2.2 m.",
            "No field joints, moving components, model plugins or collision plugins.",
            "This source cannot alone support claims of equivalent interior obstacle collision in both simulators.",
        ],
        "g1_complete": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "models/rm_field/metadata/gazebo_inventory.json",
    )
    args = parser.parse_args()
    result = audit(args.source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(
        f"Inventoried {len(result['collisions'])} colliders and {len(result['visual']['connected_components'])} visual components"
    )


if __name__ == "__main__":
    main()
