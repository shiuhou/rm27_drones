"""Bounded G1' flight experiment; reuses the verified SITL runner and guards."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from pymavlink import mavutil
from ardupilot_bridge.bridge import Bridge
from ardupilot_bridge.launch import run
from .field_adapter import FieldPlant, FIELD, ROOT


class ContactExperiment:
    def __init__(self):
        self.start = None
        self.descent_start = None
        self.next_send = 0
        self.first_contact = None
        self.max_penetration = 0
        self.max_force = 0
        self.max_tilt = 0
        self.movement = None
        self.post_contact = None

    def __call__(self, link, bridge, telemetry, event):
        p = bridge.plant
        t = p.data.time
        local = p.data.qpos[:3] - p.origin
        if self.start is None:
            self.start = t
            event("translation_start", speed_m_s=0.1, distance_m=0.3)
        elapsed = t - self.start
        target = np.array([min(0.3, 0.1 * elapsed), 0.0, 1.5])
        if self.descent_start is None and elapsed >= 6:
            error = float(np.linalg.norm(local - target))
            if error > 0.08 or np.linalg.norm(p.data.qvel[:3]) > 0.08:
                raise RuntimeError(f"free-space translation did not settle: {error}")
            self.movement = {
                "duration": elapsed,
                "target_local_nwu": target.tolist(),
                "actual_local_nwu": local.tolist(),
                "velocity": p.data.qvel[:3].tolist(),
            }
            if p.contacts():
                raise RuntimeError("unexpected contact during free translation")
            event("translation_complete", **self.movement)
            self.descent_start = t
            event("controlled_descent", speed_m_s=0.08)
        if self.descent_start is not None:
            # AP's EKF altitude origin is near the resting body, ~3.5 cm
            # above the deck-surface wire origin. A small negative final
            # setpoint ensures the .08 m/s descent reaches the deck.
            target[2] = max(-0.03, 1.5 - 0.08 * (t - self.descent_start))
            contacts = p.contacts()
            if contacts and self.first_contact is None:
                if not any(
                    "field__collision__commanding_northwest_platform" in c["geoms"]
                    for c in contacts
                ):
                    raise RuntimeError("contact with unintended field geometry")
                speed = float(np.linalg.norm(p.pre_step_velocity))
                if speed > 0.2:
                    raise RuntimeError(f"contact speed too high: {speed}")
                self.first_contact = {
                    "physics_time": t,
                    "pre_contact_velocity_nwu": p.pre_step_velocity.tolist(),
                    "contacts": contacts,
                    "position_nwu": p.data.qpos[:3].tolist(),
                    "quaternion": p.data.qpos[3:7].tolist(),
                }
                event("first_controlled_contact", **self.first_contact)
            if self.first_contact is not None:
                target[2] = 0.0
                for c in contacts:
                    self.max_penetration = max(self.max_penetration, -c["distance_m"])
                    self.max_force = max(
                        self.max_force,
                        float(np.linalg.norm(c["force_contact_frame"][:3])),
                    )
                tilt = float(
                    np.arccos(np.clip(p.data.xmat[p.body].reshape(3, 3)[2, 2], -1, 1))
                )
                self.max_tilt = max(self.max_tilt, tilt)
                if (
                    self.max_penetration > 0.02
                    or self.max_force > 200
                    or tilt > np.deg2rad(20)
                ):
                    raise RuntimeError("unstable controlled contact")
                if t - self.first_contact["physics_time"] >= 0.6:
                    self.post_contact = {
                        "position_nwu": p.data.qpos[:3].tolist(),
                        "velocity_nwu": p.data.qvel[:3].tolist(),
                        "quaternion": p.data.qpos[3:7].tolist(),
                        "armed": telemetry["armed"],
                        "mode": telemetry["mode"],
                    }
                    return True
        if t >= self.next_send:
            # Position-only local NED setpoint. Ignore velocity, acceleration and yaw.
            m = mavutil.mavlink
            mask = (
                m.POSITION_TARGET_TYPEMASK_VX_IGNORE
                | m.POSITION_TARGET_TYPEMASK_VY_IGNORE
                | m.POSITION_TARGET_TYPEMASK_VZ_IGNORE
                | m.POSITION_TARGET_TYPEMASK_AX_IGNORE
                | m.POSITION_TARGET_TYPEMASK_AY_IGNORE
                | m.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                | m.POSITION_TARGET_TYPEMASK_YAW_IGNORE
                | m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            )
            link.mav.set_position_target_local_ned_send(
                int(t * 1000),
                1,
                1,
                m.MAV_FRAME_LOCAL_NED,
                mask,
                target[0],
                -target[1],
                -target[2],
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            self.next_send = t + 0.1
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--instance", type=int, default=21)
    parser.add_argument("--hover-only", action="store_true")
    args = parser.parse_args()
    output = args.output.resolve()
    plant = FieldPlant()
    bridge = Bridge(plant)
    if plant.data.ncon:
        raise RuntimeError("spawn collides")
    experiment = ContactExperiment()
    run_started = time.perf_counter()
    code = run(
        ROOT.parent / "ardupilot",
        output,
        args.instance,
        5,
        bridge=bridge,
        after_hover=None if args.hover_only else experiment,
    )
    run_wall_seconds = time.perf_counter() - run_started
    (output / "composed.xml").write_text(plant.xml)
    result = {
        "field_commit": subprocess.check_output(
            ["git", "-C", str(FIELD.parent), "rev-parse", "HEAD"], text=True
        ).strip(),
        "field_model_sha256": hashlib.sha256(FIELD.read_bytes()).hexdigest(),
        "origin_nwu": plant.origin.tolist(),
        "compile_seconds": plant.compile_seconds,
        "run_wall_seconds": run_wall_seconds,
        "end_to_end_rtf": plant.data.time / run_wall_seconds,
        "bodies": plant.model.nbody,
        "geoms": plant.model.ngeom,
        "collision_geoms": int(
            np.count_nonzero(plant.model.geom_contype | plant.model.geom_conaffinity)
        ),
        "mean_mj_step_us": float(np.mean(plant.step_us)),
        "p95_mj_step_us": float(np.percentile(plant.step_us, 95)),
        "physics_rtf": 2000 / float(np.mean(plant.step_us)),
        "experiment": experiment.__dict__,
        "passed": code == 0,
    }
    (output / "field-result.json").write_text(json.dumps(result, indent=2))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
