#!/usr/bin/env python3
"""V1 SITL integration test.

Chain under test:

    SyntheticTargetProducer -> TargetEstimate -> UAVGuidance
        -> GuidanceCommand -> GuidanceMavlinkAdapter -> ArduPilot SITL

Prerequisites:
    1. Gazebo running:    gz sim -v4 -r iris_runway.sdf   (or headless -s)
    2. SITL reset:        <workspace>/reset_sim_v2.sh
    3. Run this script with the ardupilot venv python:
       ~/venvs/ardupilot/bin/python v1_sitl_guidance_test.py

Mission: CONNECT -> WAIT READY -> GUIDED -> PREARM -> ARM -> TAKEOFF 1.5 m
-> scenarios A-E -> LAND -> DISARM. No force-arm, no disabled prearm checks.

Scenarios:
    A  fixed +10 deg synthetic target: yaw must rotate RIGHT at ~10 deg/s
    B  5 deg then 1 deg: commanded/actual yaw rate magnitude decreases
    C  centered target: APPROACH, forward speed converges near 0.30 m/s
    D  producer dropout: forward command zeroed, SAFE_HOLD after timeout
    E  re-acquire then unsafe_for_control: immediate zero motion
"""

import json
import math
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
from v1_sitl.synthetic_target import SyntheticTargetProducer  # noqa: E402

LOOP_HZ = 10.0
LOOP_PERIOD_S = 1.0 / LOOP_HZ
DEG = math.pi / 180.0

SCENARIOS = [
    ("A_align_right_10deg", [("bearing", 6.0, {"yaw_deg": 10.0})]),
    ("B_steps_5_then_1", [
        ("bearing", 2.0, {"yaw_deg": 5.0}),
        ("bearing", 2.0, {"yaw_deg": 1.0}),
    ]),
    ("C_approach_centered", [("bearing", 5.0, {"yaw_deg": 0.0})]),
    ("D_dropout", [("dropout", 2.0, {})]),
    ("E_reacquire_then_unsafe", [
        ("bearing", 2.0, {"yaw_deg": 0.0}),
        ("unsafe", 2.0, {}),
    ]),
]


class Telemetry:
    """Latest-value telemetry cache fed by non-blocking receives."""

    def __init__(self):
        self.yaw_rad = None
        self.yawspeed_rad_s = None
        self.vx = self.vy = self.vz = None
        self.armed = None
        self.mode = None

    @property
    def horizontal_speed(self):
        if self.vx is None or self.vy is None:
            return None
        return math.hypot(self.vx, self.vy)


def drain_telemetry(master, tele):
    while True:
        msg = master.recv_match(
            type=["ATTITUDE", "LOCAL_POSITION_NED", "HEARTBEAT"],
            blocking=False,
        )
        if msg is None:
            return
        if not v5.is_from_target(
            msg, master.target_system, master.target_component
        ):
            continue
        mtype = msg.get_type()
        if mtype == "ATTITUDE":
            tele.yaw_rad = msg.yaw
            tele.yawspeed_rad_s = msg.yawspeed
        elif mtype == "LOCAL_POSITION_NED":
            tele.vx, tele.vy, tele.vz = msg.vx, msg.vy, msg.vz
        elif mtype == "HEARTBEAT":
            tele.armed = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            inv = {v: k for k, v in master.mode_mapping().items()}
            tele.mode = inv.get(msg.custom_mode)


def run_scenario(master, adapter, guidance, name, steps, tele, log_file,
                 guidance_logger):
    """Run one scripted scenario at LOOP_HZ; return per-cycle records."""
    producer = SyntheticTargetProducer(steps, start_us=now_us())
    records = []

    while not producer.finished(now_us()):
        cycle_start = time.monotonic()
        t_us = now_us()

        estimate = producer.update(t_us)
        output = guidance.update(estimate, t_us)
        adapter.send(output.command)
        guidance_logger.log(t_us, estimate, output)

        drain_telemetry(master, tele)
        record = {
            "scenario": name,
            "t_us": t_us,
            "state": output.state.name,
            "cmd_vx": output.command.vx_body_mps,
            "cmd_yaw_rate": output.command.yaw_rate_rad_s,
            "cmd_valid": output.command.valid,
            "reject": output.reject_reason,
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


def now_us():
    return int(time.monotonic() * 1_000_000)


def mean(values):
    vals = [v for v in values if v is not None]
    return sum(vals) / len(vals) if vals else float("nan")


def evaluate(results):
    checks = []

    a = results["A_align_right_10deg"]
    a_yawspeed = mean([r["yawspeed_rad_s"] for r in a])
    a_yaws = [r["yaw_rad"] for r in a if r["yaw_rad"] is not None]
    a_delta_yaw = (a_yaws[-1] - a_yaws[0]) if a_yaws else 0.0
    checks.append((
        "A: target right -> vehicle yaws RIGHT (NED positive)",
        a_yawspeed > math.radians(3.0) and a_delta_yaw > math.radians(15.0),
        f"mean yawspeed={math.degrees(a_yawspeed):.1f} deg/s, "
        f"delta_yaw={math.degrees(a_delta_yaw):.1f} deg over 6 s",
    ))

    b = results["B_steps_5_then_1"]
    half = len(b) // 2
    b_cmd_first = mean([abs(r["cmd_yaw_rate"]) for r in b[:half]])
    b_cmd_second = mean([abs(r["cmd_yaw_rate"]) for r in b[half:]])
    b_act_first = mean([abs(r["yawspeed_rad_s"]) for r in b[:half]])
    b_act_second = mean([abs(r["yawspeed_rad_s"]) for r in b[half:]])
    checks.append((
        "B: 5 deg -> 1 deg shrinks yaw-rate command and response",
        b_cmd_first > b_cmd_second and b_act_first > b_act_second,
        f"cmd {math.degrees(b_cmd_first):.1f}->{math.degrees(b_cmd_second):.1f} deg/s, "
        f"actual {math.degrees(b_act_first):.1f}->{math.degrees(b_act_second):.1f} deg/s",
    ))

    c = results["C_approach_centered"]
    c_states = {r["state"] for r in c}
    c_speed = mean([r["speed_mps"] for r in c[len(c) // 2:]])
    c_cmd = mean([r["cmd_vx"] for r in c])
    checks.append((
        "C: centered target -> APPROACH at ~0.30 m/s",
        c_states == {"APPROACH"}
        and abs(c_cmd - 0.30) < 1e-6
        and 0.15 < c_speed < 0.55,
        f"states={sorted(c_states)}, cmd_vx={c_cmd:.2f}, "
        f"actual speed={c_speed:.2f} m/s",
    ))

    d = results["D_dropout"]
    d_vx = [r["cmd_vx"] for r in d]
    d_states = {r["state"] for r in d}
    checks.append((
        "D: dropout -> zero forward command, SAFE_HOLD within timeout",
        all(v == 0.0 for v in d_vx) and "SAFE_HOLD" in d_states,
        f"states={sorted(d_states)}, max cmd_vx={max(d_vx):.2f}",
    ))

    e = results["E_reacquire_then_unsafe"]
    half = len(e) // 2
    e_first_states = {r["state"] for r in e[:half]}
    e_second = e[half:]
    e_second_zero = all(
        r["cmd_vx"] == 0.0 and r["cmd_yaw_rate"] == 0.0 for r in e_second
    )
    checks.append((
        "E: unsafe_for_control during APPROACH -> immediate zero motion",
        "APPROACH" in e_first_states
        and e_second_zero
        and all(r["state"] == "SAFE_HOLD" for r in e_second),
        f"first-half states={sorted(e_first_states)}, "
        f"second-half states={sorted({r['state'] for r in e_second})}",
    ))

    all_records = [r for recs in results.values() for r in recs]
    max_speed = max(
        (r["speed_mps"] for r in all_records if r["speed_mps"] is not None),
        default=0.0,
    )
    armed_seen = [r["armed"] for r in all_records if r["armed"] is not None]
    mode_seen = [r["mode"] for r in all_records if r["mode"] is not None]
    checks.append((
        "Global: no runaway velocity, stayed GUIDED+armed",
        max_speed < 1.5
        and bool(armed_seen) and all(armed_seen)
        and bool(mode_seen) and all(m == "GUIDED" for m in mode_seen),
        f"max horizontal speed={max_speed:.2f} m/s",
    ))

    cycles = len(all_records)
    checks.append((
        "Coverage: >= 50 guidance/MAVLink update cycles",
        cycles >= 50,
        f"cycles={cycles}",
    ))

    return checks


def main():
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    log_dir = VISUAL_GUIDANCE_DIR / "logs"
    log_dir.mkdir(exist_ok=True)
    telemetry_path = log_dir / f"v1_sitl_{timestamp}.jsonl"
    guidance_path = log_dir / f"v1_guidance_{timestamp}.jsonl"

    master = None
    try:
        master = v5.connect_vehicle()
        v5.wait_vehicle_ready(master)
        # V1 also needs fast ATTITUDE for yaw-response verification.
        v5.request_message_interval(
            master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20
        )

        print("\n[3] GUIDED")
        v5.set_mode(master, "GUIDED")
        v5.arm_vehicle(master)
        v5.takeoff(master)

        adapter = GuidanceMavlinkAdapter(master)
        guidance = UAVGuidance(GuidanceConfig())
        tele = Telemetry()

        results = {}
        with open(telemetry_path, "w", encoding="utf-8") as log_file, \
                GuidanceLogger(guidance_path) as guidance_logger:
            for name, steps in SCENARIOS:
                print(f"\n=== Scenario {name} ===")
                results[name] = run_scenario(
                    master, adapter, guidance, name, steps, tele,
                    log_file, guidance_logger,
                )

        print("\n[9] LAND")
        v5.land(master)

        print("\n========== V1 SITL REPORT ==========")
        all_pass = True
        for title, ok, detail in evaluate(results):
            status = "PASS" if ok else "FAIL"
            all_pass &= ok
            print(f"[{status}] {title}\n         {detail}")
        print(f"telemetry log : {telemetry_path}")
        print(f"guidance log  : {guidance_path}")
        print(f"RESULT        : {'PASS' if all_pass else 'FAIL'}")
        print("=====================================")
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
