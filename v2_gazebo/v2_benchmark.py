#!/usr/bin/env python3
"""V2 Gazebo closed-loop benchmark.

Chain under test:

    Gazebo truth poses -> TruthTargetAdapter -> TargetEstimate
        -> UAVGuidance -> GuidanceCommand -> TruthSupervisor (stop gate)
        -> GuidanceMavlinkAdapter -> ArduPilot SITL -> Gazebo motion

Prerequisites:
    1. Gazebo running:  gz sim -v4 -r iris_runway.sdf (or headless -s)
       with GZ_SIM_SYSTEM_PLUGIN_PATH/GZ_SIM_RESOURCE_PATH set to gz_ws
    2. SITL reset:        <workspace>/reset_sim_v2.sh
    3. Run with the ardupilot venv python:
       ~/venvs/ardupilot/bin/python v2_benchmark.py

Mission: CONNECT -> WAIT READY -> GUIDED -> ARM -> TAKEOFF 1.5 m
-> spawn a Gazebo target model -> scenarios -> LAND -> DISARM.

Truth discipline: only TruthTargetAdapter and TruthSupervisor read
Gazebo poses. Guidance and the MAVLink adapter see exactly what they
saw in V1 (TargetEstimate / GuidanceCommand respectively).

Scenarios (target teleported relative to the UAV's current true pose):
    front_right : +20 deg initial yaw error -> converge <= 2 deg, APPROACH
    front_left  : -20 deg -> converge <= 2 deg, APPROACH
    ahead       : 0 deg, 20 s run -> supervisor stop-distance must gate
    invalid     : approach, then safe_for_control=False -> zero motion
    stale       : approach, then stale measurement -> SAFE_HOLD
"""

import json
import math
import subprocess
import sys
import time
from pathlib import Path

VISUAL_GUIDANCE_DIR = Path(__file__).resolve().parent.parent
MISSIONS_DIR = VISUAL_GUIDANCE_DIR.parent / "missions"
sys.path.insert(0, str(VISUAL_GUIDANCE_DIR))
sys.path.insert(0, str(MISSIONS_DIR))

from pymavlink import mavutil  # noqa: E402

import takeoff_forward_land_v5_trajectory as v5  # noqa: E402

from uav_guidance import GuidanceConfig, UAVGuidance  # noqa: E402
from uav_guidance.logging import GuidanceLogger  # noqa: E402
from v1_sitl.mavlink_adapter import GuidanceMavlinkAdapter  # noqa: E402
from v1_sitl.v1_sitl_guidance_test import (  # noqa: E402
    Telemetry,
    drain_telemetry,
    now_us,
)
from v2_gazebo.supervisor import TruthSupervisor  # noqa: E402
from v2_gazebo.truth_adapter import TruthTargetAdapter, enu_yaw_rad  # noqa: E402
from v2_gazebo.truth_source import GazeboPoseSource  # noqa: E402

LOOP_HZ = 10.0
LOOP_PERIOD_S = 1.0 / LOOP_HZ
DEG = math.pi / 180.0

WORLD = "iris_runway"
UAV_NAME = "iris_with_gimbal"
TARGET_NAME = "rm27_target"
STOP_DISTANCE_M = 2.0
SCENARIO_DISTANCE_M = 6.0

# (name, initial yaw error deg, [(duration_s, adapter_mode)])
SCENARIOS = [
    ("front_right", 20.0, [(12.0, "normal")]),
    ("front_left", -20.0, [(12.0, "normal")]),
    ("ahead", 0.0, [(20.0, "normal")]),
    ("invalid", 10.0, [(6.0, "normal"), (4.0, "unsafe"),
                       (2.0, "normal")]),
    ("stale", 10.0, [(6.0, "normal"), (4.0, "stale"),
                     (2.0, "normal")]),
]

TARGET_SDF = """<sdf version="1.9"><model name="{name}">
  <pose>0 0 0.25 0 0 0</pose>
  <link name="link">
    <inertial>
      <mass>0.5</mass>
      <inertia>
        <ixx>0.02</ixx><iyy>0.02</iyy><izz>0.02</izz>
      </inertia>
    </inertial>
    <collision name="c">
      <geometry><box><size>0.4 0.4 0.5</size></box></geometry>
    </collision>
    <visual name="v">
      <geometry><box><size>0.4 0.4 0.5</size></box></geometry>
      <material><diffuse>0.9 0.1 0.1 1</diffuse></material>
    </visual>
  </link>
</model></sdf>"""


def gz_service(service, reqtype, reptype, req):
    result = subprocess.run(
        ["gz", "service", "-s", service,
         "--reqtype", reqtype, "--reptype", reptype,
         "--timeout", "5000", "--req", req],
        capture_output=True, text=True, check=False,
    )
    if result.returncode != 0 or "data: true" not in result.stdout:
        raise RuntimeError(
            f"gz service {service} failed: rc={result.returncode} "
            f"stdout={result.stdout.strip()} stderr={result.stderr.strip()}"
        )


def spawn_target():
    # protobuf text format forbids multi-line string literals
    sdf = " ".join(TARGET_SDF.format(name=TARGET_NAME).split())
    gz_service(
        f"/world/{WORLD}/create", "gz.msgs.EntityFactory",
        "gz.msgs.Boolean",
        f'sdf: \'{sdf}\' name: "{TARGET_NAME}" allow_renaming: false',
    )


def teleport_target(x, y, z=0.25):
    gz_service(
        f"/world/{WORLD}/set_pose", "gz.msgs.Pose", "gz.msgs.Boolean",
        f'name: "{TARGET_NAME}" position {{ x: {x:.3f} y: {y:.3f} '
        f"z: {z:.3f} }}",
    )


def target_position_for(uav_pose, yaw_error_deg, distance_m):
    """World position of a target placed `yaw_error_deg` (vision sign:
    right positive) from the UAV's current heading at `distance_m`."""
    uav_yaw_enu = enu_yaw_rad(uav_pose["quat"])
    heading_err_enu = -yaw_error_deg * DEG
    bearing = uav_yaw_enu + heading_err_enu
    return (
        uav_pose["pos"][0] + distance_m * math.cos(bearing),
        uav_pose["pos"][1] + distance_m * math.sin(bearing),
    )


def run_scenario(master, mav_adapter, truth_adapter, supervisor, source,
                 tele, name, yaw_err_deg, phases, log_file,
                 guidance_logger):
    uav = source.poses()[UAV_NAME]
    tx, ty = target_position_for(uav, yaw_err_deg, SCENARIO_DISTANCE_M)
    teleport_target(tx, ty)
    time.sleep(0.5)  # let the pose topic reflect the teleport

    guidance = UAVGuidance(GuidanceConfig())  # fresh state per scenario
    records = []
    start_us = now_us()
    phase_plan = []
    cursor = 0.0
    for duration_s, mode in phases:
        phase_plan.append((cursor, cursor + duration_s, mode))
        cursor += duration_s

    while True:
        t_s = (now_us() - start_us) / 1e6
        if t_s >= cursor:
            break
        cycle_start = time.monotonic()
        mode = next(m for a, b, m in phase_plan if a <= t_s < b)
        t_us = now_us()

        estimate = truth_adapter.update(t_us, mode=mode)
        output = guidance.update(estimate, t_us)
        command, gated = supervisor.apply(output.command, t_us)
        mav_adapter.send(command)
        guidance_logger.log(t_us, estimate, output)

        drain_telemetry(master, tele)
        record = {
            "scenario": name,
            "t_s": round(t_s, 3),
            "phase_mode": mode,
            "state": output.state.name,
            "reject": output.reject_reason,
            "cmd_vx": command.vx_body_mps,
            "cmd_yaw_rate": command.yaw_rate_rad_s,
            "cmd_valid": command.valid,
            "supervisor_gated": gated,
            "est_yaw_rad": estimate.yaw_rad if estimate else None,
            "est_pitch_rad": estimate.pitch_rad if estimate else None,
            "distance_m": supervisor.distance_m(),
            "yaw_rad": tele.yaw_rad,
            "yawspeed_rad_s": tele.yawspeed_rad_s,
            "speed_mps": tele.horizontal_speed,
            "armed": tele.armed,
            "mode": tele.mode,
        }
        records.append(record)
        log_file.write(json.dumps(record) + "\n")
        log_file.flush()

        if tele.armed is False or (tele.mode not in (None, "GUIDED")):
            raise RuntimeError(
                f"unexpected state during {name}: armed={tele.armed} "
                f"mode={tele.mode}"
            )

        elapsed = time.monotonic() - cycle_start
        if elapsed < LOOP_PERIOD_S:
            time.sleep(LOOP_PERIOD_S - elapsed)

    return records


def mean(values):
    vals = [v for v in values if v is not None]
    return sum(vals) / len(vals) if vals else float("nan")


def approach_stats(records):
    """Stats over APPROACH-state records that are not supervisor-gated."""
    app = [r for r in records
           if r["state"] == "APPROACH" and not r["supervisor_gated"]]
    if not app:
        return None
    return {
        "n": len(app),
        "mean_abs_err_deg": math.degrees(mean(
            [abs(r["est_yaw_rad"]) for r in app])),
        "mean_cmd_vx": mean([r["cmd_vx"] for r in app]),
        "mean_speed": mean([r["speed_mps"] for r in app]),
        "max_abs_err_deg": math.degrees(max(
            abs(r["est_yaw_rad"]) for r in app)),
    }


def convergence_time_s(records, threshold_deg=2.0):
    for r in records:
        if r["est_yaw_rad"] is not None and \
                abs(r["est_yaw_rad"]) <= threshold_deg * DEG:
            return r["t_s"]
    return None


def evaluate(results):
    checks = []

    for name, expect_err in (("front_right", 20.0),
                             ("front_left", -20.0)):
        recs = results[name]
        first = next((r["est_yaw_rad"] for r in recs
                      if r["est_yaw_rad"] is not None), 0.0)
        conv = convergence_time_s(recs)
        stats = approach_stats(recs)
        entered = stats is not None and stats["n"] >= 10
        conv_txt = f"{conv:.1f}" if conv is not None else "never"
        if stats:
            detail = (
                f"initial={math.degrees(first):+.1f} deg, "
                f"converged in {conv_txt} s, "
                f"approach mean|err|={stats['mean_abs_err_deg']:.2f} deg "
                f"(max {stats['max_abs_err_deg']:.2f}), "
                f"cmd_vx={stats['mean_cmd_vx']:.2f}, "
                f"speed={stats['mean_speed']:.2f} m/s"
            )
        else:
            detail = (f"initial={math.degrees(first):+.1f} deg, "
                      f"converged in {conv_txt} s, no APPROACH phase")
        checks.append((
            f"{name}: {expect_err:+.0f} deg -> converge <=2 deg, APPROACH",
            math.copysign(1.0, first) == math.copysign(1.0, expect_err)
            and abs(math.degrees(first)) > 0.5 * abs(expect_err)
            and conv is not None and conv <= 8.0
            and entered
            and stats["mean_abs_err_deg"] <= 3.0
            and abs(stats["mean_cmd_vx"] - 0.30) < 1e-6
            and 0.15 < stats["mean_speed"] < 0.55,
            detail,
        ))

    ahead = results["ahead"]
    a_stats = approach_stats(ahead)
    a_gated = [r for r in ahead if r["supervisor_gated"]]
    a_min_dist = min((r["distance_m"] for r in ahead
                      if r["distance_m"] is not None), default=None)
    checks.append((
        "ahead: 0 deg -> APPROACH, supervisor stops at stop distance",
        a_stats is not None and a_stats["n"] >= 10
        and a_stats["mean_abs_err_deg"] <= 3.0
        and len(a_gated) >= 10
        and a_min_dist is not None
        and a_min_dist >= STOP_DISTANCE_M - 0.25,
        (f"approach mean|err|={a_stats['mean_abs_err_deg']:.2f} deg, "
         f"gated_cycles={len(a_gated)}, min_distance={a_min_dist:.2f} m")
        if a_stats and a_min_dist is not None
        else f"no APPROACH phase (gated={len(a_gated)})",
    ))

    for name, bad_mode in (("invalid", "unsafe"), ("stale", "stale")):
        recs = results[name]
        bad = [r for r in recs if r["phase_mode"] == bad_mode]
        recovery = [r for r in recs
                    if r["phase_mode"] == "normal" and r["t_s"] > 10.0]
        bad_zero = all(
            r["cmd_vx"] == 0.0 and r["cmd_yaw_rate"] == 0.0 for r in bad
        )
        checks.append((
            f"{name}: {bad_mode} phase -> zero motion, then recovery",
            len(bad) >= 20 and bad_zero
            and all(r["state"] == "SAFE_HOLD" for r in bad)
            and len(recovery) >= 5
            and any(r["state"] == "APPROACH" for r in recovery),
            f"bad_cycles={len(bad)}, zero_motion={bad_zero}, "
            f"recovery_approach="
            f"{any(r['state'] == 'APPROACH' for r in recovery)}",
        ))

    all_records = [r for recs in results.values() for r in recs]
    max_speed = max(
        (r["speed_mps"] for r in all_records if r["speed_mps"] is not None),
        default=0.0,
    )
    min_dist = min((r["distance_m"] for r in all_records
                    if r["distance_m"] is not None), default=None)
    armed_seen = [r["armed"] for r in all_records if r["armed"] is not None]
    mode_seen = [r["mode"] for r in all_records if r["mode"] is not None]
    cycles = len(all_records)
    checks.append((
        "Global: GUIDED+armed throughout, bounded speed, no contact",
        max_speed < 1.5
        and bool(armed_seen) and all(armed_seen)
        and bool(mode_seen) and all(m == "GUIDED" for m in mode_seen)
        and min_dist is not None and min_dist >= 1.0,
        f"max speed={max_speed:.2f} m/s, min distance={min_dist:.2f} m",
    ))
    checks.append((
        "Coverage: >= 100 closed-loop cycles",
        cycles >= 100,
        f"cycles={cycles}",
    ))
    return checks


def main():
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    log_dir = VISUAL_GUIDANCE_DIR / "logs"
    log_dir.mkdir(exist_ok=True)
    telemetry_path = log_dir / f"v2_sitl_{timestamp}.jsonl"
    guidance_path = log_dir / f"v2_guidance_{timestamp}.jsonl"

    master = None
    try:
        master = v5.connect_vehicle()
        v5.wait_vehicle_ready(master)
        v5.request_message_interval(
            master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20
        )

        print("\n[3] GUIDED")
        v5.set_mode(master, "GUIDED")
        v5.arm_vehicle(master)
        v5.takeoff(master)

        with GazeboPoseSource(world=WORLD) as source, \
                open(telemetry_path, "w", encoding="utf-8") as log_file, \
                GuidanceLogger(guidance_path) as guidance_logger:
            time.sleep(1.0)  # let the pose stream warm up
            if TARGET_NAME not in source.poses():
                print(f"[4] Spawning target model {TARGET_NAME}")
                spawn_target()
                time.sleep(1.0)

            adapter = GuidanceMavlinkAdapter(master)
            truth_adapter = TruthTargetAdapter(source)
            supervisor = TruthSupervisor(
                source, STOP_DISTANCE_M,
                uav_name=UAV_NAME, target_name=TARGET_NAME,
            )
            tele = Telemetry()

            results = {}
            for name, yaw_err_deg, phases in SCENARIOS:
                print(f"\n=== Scenario {name} "
                      f"({yaw_err_deg:+.0f} deg) ===")
                results[name] = run_scenario(
                    master, adapter, truth_adapter, supervisor, source,
                    tele, name, yaw_err_deg, phases, log_file,
                    guidance_logger,
                )

        print("\n[9] LAND")
        v5.land(master)

        print("\n========== V2 GAZEBO REPORT ==========")
        all_pass = True
        for title, ok, detail in evaluate(results):
            status = "PASS" if ok else "FAIL"
            all_pass &= ok
            print(f"[{status}] {title}\n         {detail}")
        print(f"telemetry log : {telemetry_path}")
        print(f"guidance log  : {guidance_path}")
        print(f"RESULT        : {'PASS' if all_pass else 'FAIL'}")
        print("======================================")
        return 0 if all_pass else 1

    except Exception:
        if master is not None and v5.vehicle_is_armed(master):
            print("ERROR during mission; attempting LAND...")
            try:
                v5.set_mode(master, "LAND")
            except Exception as land_error:
                print(f"WARNING: LAND failed: {land_error}")
        raise
    finally:
        if master is not None:
            try:
                master.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
