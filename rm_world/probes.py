"""Deterministic low-speed contacts with the full, unfiltered composed field."""

import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from .field_adapter import compose_xml

CASES = {
    "floor_drop": ((-12, 0, 0.35), (0, 0, 0), False, "field__collision__ground"),
    "central_drop": ((0, 0, 0.65), (0, 0, 0), False, "field__collision__planar_"),
    "ramp_drop": ((-0.8, -6.9, 1.1), (0, 0, 0), False, "field__collision__planar_"),
    "deck_drop": (
        (-9.55, 3.75, 0.85),
        (0, 0, 0),
        False,
        "field__collision__commanding_northwest_platform",
    ),
    "ramp_wall_lower": (
        (-0.15, 0.15, 0.5),
        (-0.2, 0.2, 0),
        True,
        "field__collision__ramp_side_barrier_",
    ),
    "ramp_wall_upper": (
        (-0.15, 0.15, 0.72),
        (-0.2, 0.2, 0),
        True,
        "field__collision__ramp_side_barrier_",
    ),
    "launch_wall": (
        (-0.8, -6.9, 0.5),
        (0, -0.2, 0),
        True,
        "field__collision__flight_ramp_barrier_",
    ),
    "highland_wall": (
        (-10.8, 5.6, 0.75),
        (-0.2, 0, 0),
        True,
        "field__collision__side_highland_upper_",
    ),
}


def probe(name):
    pos, vel, compensate, prefix = CASES[name]
    root = ET.fromstring(compose_xml())
    root.find("worldbody").remove(root.find("worldbody/body[@name='quadrotor']"))
    root.remove(root.find("actuator"))
    body = ET.SubElement(
        root.find("worldbody"),
        "body",
        name="probe",
        pos=" ".join(map(str, pos)),
        gravcomp=str(int(compensate)),
    )
    ET.SubElement(body, "freejoint", name="probe_joint")
    ET.SubElement(
        body,
        "geom",
        name="probe_geom",
        type="sphere",
        size=".1",
        mass="1",
        friction=".8 .01 .001",
    )
    start = time.perf_counter()
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    load = time.perf_counter() - start
    data = mujoco.MjData(model)
    data.qvel[:3] = vel
    mujoco.mj_forward(model, data)
    hit = set()
    forces = []
    distances = []
    normals = []
    steps = []
    counts = []
    first = None
    max_speed = 0
    for _ in range(1500):
        before = data.qvel[:3].copy()
        start = time.perf_counter_ns()
        mujoco.mj_step(model, data)
        steps.append((time.perf_counter_ns() - start) / 1000)
        assert np.all(np.isfinite(np.r_[data.qpos, data.qvel, data.qacc]))
        max_speed = max(max_speed, float(np.linalg.norm(data.qvel[:3])))
        counts.append(data.ncon)
        for i in range(data.ncon):
            contact = data.contact[i]
            names = [
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g))
                for g in contact.geom
            ]
            if "probe_geom" not in names:
                continue
            field = [n for n in names if n and n.startswith("field__collision__")]
            hit.update(field)
            force = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, force)
            forces.append(float(np.linalg.norm(force[:3])))
            distances.append(float(contact.dist))
            normals.append(contact.frame[:3].tolist())
            if first is None:
                first = {
                    "time": data.time,
                    "pre_contact_velocity": before.tolist(),
                    "geoms": names,
                    "normal": contact.frame[:3].tolist(),
                }
    expected = any(n.startswith(prefix) for n in hit)
    result = {
        "name": name,
        "passed": expected
        and bool(forces)
        and max_speed < 5
        and min(distances, default=0) > -0.03,
        "expected_prefix": prefix,
        "hit_geoms": sorted(hit),
        "first_contact": first,
        "final_position": data.qpos[:3].tolist(),
        "final_velocity": data.qvel[:3].tolist(),
        "max_force_n": max(forces, default=0),
        "min_distance_m": min(distances, default=0),
        "max_speed": max_speed,
        "load_seconds": load,
        "mean_mj_step_us": float(np.mean(steps)),
        "p95_mj_step_us": float(np.percentile(steps, 95)),
        "mean_contacts": float(np.mean(counts)),
        "max_contacts": max(counts),
        "physics_rtf": 2000 / float(np.mean(steps)),
    }
    if name == "floor_drop":
        result["passed"] &= (
            0.095 < data.qpos[2] < 0.105 and np.linalg.norm(data.qvel) < 0.01
        )
    if name.startswith("ramp_wall"):
        # Cannot cross to the far side of the audited central ramp wall.
        offset = -0.578 if name.endswith("lower") else -0.652
        n = (
            np.array([0.70710678, -0.70710678, 0])
            if name.endswith("lower")
            else np.array([0.701, -0.701, -0.131])
        )
        result["signed_final_side"] = float(n @ data.qpos[:3] - offset)
        result["passed"] &= result["signed_final_side"] > 0
    if name == "launch_wall":
        result["passed"] &= data.qpos[1] > -7.34
    if name == "highland_wall":
        result["passed"] &= data.qpos[0] > -11.2452
    result["passed"] = bool(result["passed"])
    return result


def main():
    results = [probe(name) for name in CASES]
    path = Path(__file__).parent / "artifacts/contact-probes.json"
    path.write_text(json.dumps(results, indent=2))
    for r in results:
        print(r["name"], r["passed"], r["hit_geoms"], r["min_distance_m"])
    return int(not all(r["passed"] for r in results))


if __name__ == "__main__":
    raise SystemExit(main())
