"""Measure full-field idle cost and summarize recorded flight physics timings."""

import argparse
import hashlib
import json
from pathlib import Path
import time
import mujoco
import numpy as np
from .field_adapter import FIELD, FieldPlant


def timing(values):
    return {
        "samples": len(values),
        "mean_us": float(np.mean(values)),
        "median_us": float(np.median(values)),
        "p95_us": float(np.percentile(values, 95)),
        "physics_rtf": 2000 / float(np.mean(values)),
    }


def history_digest(path):
    h = hashlib.sha256()
    with path.open() as f:
        for line in f:
            row = json.loads(line)
            h.update(
                json.dumps(
                    {k: row[k] for k in ["frame_count", "pwm", "state_sent", "truth"]},
                    sort_keys=True,
                ).encode()
            )
    return h.hexdigest()


def analyze(field_run, baseline_run):
    start = time.perf_counter()
    m = mujoco.MjModel.from_xml_path(str(FIELD))
    d = mujoco.MjData(m)
    load = time.perf_counter() - start
    values = []
    for i in range(6000):
        t = time.perf_counter_ns()
        mujoco.mj_step(m, d)
        if i >= 1000:
            values.append((time.perf_counter_ns() - t) / 1000)
    idle = {
        "compile_seconds": load,
        "bodies": m.nbody,
        "geoms": m.ngeom,
        "collision_geoms": int(np.count_nonzero(m.geom_contype | m.geom_conaffinity)),
        "active_contacts": d.ncon,
        **timing(values),
    }
    plant = FieldPlant()
    for _ in range(1500):
        plant.step([1000] * 4)
    plant.step_us = []
    contacts = []
    for _ in range(5000):
        plant.step([1000] * 4)
        contacts.append(plant.data.ncon)
    resting = {
        "compile_seconds": plant.compile_seconds,
        "bodies": plant.model.nbody,
        "geoms": plant.model.ngeom,
        "mean_contacts": float(np.mean(contacts)),
        **timing(plant.step_us),
    }
    rows = [json.loads(l) for l in (field_run / "frames.jsonl").open()]
    baseline = [json.loads(l) for l in (baseline_run / "frames.jsonl").open()]
    summary = json.loads((field_run / "summary.json").read_text())
    hover_start = next(
        e["physics_time"]
        for e in summary["events"]
        if e["event"] == "hover_window_start"
    )
    hover_end = next(
        e["physics_time"] for e in summary["events"] if e["event"] == "hover_complete"
    )
    result = {"field_only_idle": idle, "field_with_resting_uav": resting, "phases": {}}
    field_result = json.loads((field_run / "field-result.json").read_text())
    first = field_result["experiment"]["first_contact"]["physics_time"]
    for name, lo, hi in [
        ("hover", hover_start, hover_end),
        ("near_contact", first - 1, first + 0.6),
    ]:
        selected = [r for r in rows if lo <= r["state_sent"]["timestamp"] <= hi]
        result["phases"][name] = {
            **timing([r["truth"]["mj_step_us"] for r in selected]),
            "mean_contacts": float(
                np.mean([len(r["truth"]["contacts"]) for r in selected])
            ),
            "max_contacts": max(len(r["truth"]["contacts"]) for r in selected),
        }
    count = min(len(rows), len(baseline))
    # Compare wire positions: both use their own documented navigation origin.
    a = np.asarray([r["state_sent"]["position"] for r in rows[:count]])
    b = np.asarray([r["state_sent"]["position"] for r in baseline[:count]])
    result["comparison"] = {
        "frames_compared": count,
        "max_position_difference_m": float(np.linalg.norm(a - b, axis=1).max()),
        "max_altitude_difference_m": float(np.abs(a[:, 2] - b[:, 2]).max()),
        "standalone_sha256": history_digest(baseline_run / "frames.jsonl"),
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--field-run", type=Path, required=True)
    parser.add_argument("--baseline-run", type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.field_run, args.baseline_run)
    (Path(__file__).parent / "artifacts/performance.json").write_text(
        json.dumps(result, indent=2)
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
