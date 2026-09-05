#!/usr/bin/env python3
"""M4-A micro-UAV takeoff/hover/land benchmark.

Flies the 180 g / 120 mm plant (models/quadrotor_m4a.xml) through the REAL
production path with the UNCHANGED M0 controller gains:

    TAKEOFF action -> FlightStateMachine profile -> position/SO(3)
    controller -> mixer -> motor thrusts -> MuJoCo physics -> LAND action

No qpos/qvel manipulation after the initial condition. This is the plant
experiment: any performance change vs M0 comes from the micro plant, not
from the autonomy stack.

Usage:

    python m4a_hover_benchmark.py
    python m4a_hover_benchmark.py --json m4a_hover_report.json
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path
from typing import Sequence

import numpy as np

from fly import positive_float, rotation_to_euler, wrap_angle
from ipc_protocol import CommandPacket, PROTOCOL_VERSION
from m4_params import M4A_NOMINAL, MicroUAVConfig
from simulator import FlightParameters, FlightState, QuadrotorSimulation


M4A_MODEL = Path(__file__).resolve().parent / "models" / "quadrotor_m4a.xml"

# M4-A engineering milestone gates (not final aircraft specs).
MAX_FINAL_ALTITUDE_ERROR = 0.15
MAX_HOVER_RMS_ERROR = 0.10
MAX_HORIZONTAL_DRIFT = 0.15
MAX_TILT_DEG = 8.0
MAX_SATURATION_FRACTION = 0.20
MIN_HOVER_ALTITUDE = 0.5
KEEPALIVE_PERIOD = 0.05


@dataclass(frozen=True)
class M4AHoverReport:
    model_file: str
    mass_kg: float
    wheelbase_m: float
    thrust_to_weight_max: float
    target_altitude: float
    takeoff_time: float
    max_altitude_overshoot: float
    final_altitude: float
    final_altitude_error: float
    hover_rms_error: float
    max_horizontal_drift: float
    max_horizontal_speed: float
    max_roll_deg: float
    max_pitch_deg: float
    max_yaw_error_deg: float
    max_motor_command: float
    saturation_fraction: float
    hover_duration_measured: float
    reached_hovering: bool
    landing_success: bool
    landed_altitude: float
    has_nan_or_inf: bool
    total_time: float
    passed: bool
    reasons: tuple[str, ...] = field(default_factory=tuple)

    def to_json_dict(self) -> dict:
        return asdict(self)


def run_m4a_hover_benchmark(
    target_height: float = 1.5,
    hover_duration: float = 5.0,
    max_total_time: float = 45.0,
    model_path: Path = M4A_MODEL,
    # Default nominal so the controller gets the inertia-scaled attitude
    # gains even when the caller only points at the M4-A XML.
    micro_config: MicroUAVConfig | None = M4A_NOMINAL,
) -> M4AHoverReport:
    """Take off to `target_height`, hover `hover_duration` s, then land."""

    config = micro_config or M4A_NOMINAL
    parameters = FlightParameters(takeoff_height=target_height)
    simulation = QuadrotorSimulation(
        Path(model_path), parameters, micro_config=micro_config
    )
    sequence = 0

    def send(action: str | None = None) -> None:
        nonlocal sequence
        simulation.accept_command(
            CommandPacket(
                PROTOCOL_VERSION, sequence, simulation.data.time, action,
                0.0, 0.0, 0.0, 0.0,
            ),
            simulation.data.time,
        )
        sequence += 1

    initial = simulation.controller.read_state(simulation.data)
    yaw0 = float(rotation_to_euler(initial.rotation_body_to_world)[2])

    takeoff_request_time = simulation.data.time
    send("takeoff")
    next_keepalive = 0.0
    hover_start_time: float | None = None
    hover_anchor_xy: np.ndarray | None = None
    land_sent = False
    has_nan_or_inf = False

    altitude_errors: list[float] = []
    horizontal_drifts: list[float] = []
    max_altitude = float(initial.position[2])
    max_horizontal_speed = 0.0
    max_roll_deg = 0.0
    max_pitch_deg = 0.0
    max_yaw_error_deg = 0.0
    max_motor_command = 0.0
    airborne_steps = 0
    saturated_steps = 0

    while simulation.data.time < max_total_time:
        sim_time = simulation.data.time
        if sim_time >= next_keepalive:
            send()
            next_keepalive = sim_time + KEEPALIVE_PERIOD

        vehicle = simulation.step(sim_time)

        values = np.concatenate(
            (vehicle.position, vehicle.velocity_world,
             simulation.last_motor_thrusts)
        )
        if not np.all(np.isfinite(values)):
            has_nan_or_inf = True
            break

        state = simulation.machine.state
        airborne = state not in (FlightState.DISARMED, FlightState.ARMED_IDLE)
        if airborne:
            airborne_steps += 1
            motors = simulation.last_motor_thrusts
            limits = simulation.controller
            if np.any(motors >= limits.maximum_thrusts - 1e-9) or np.any(
                motors <= limits.minimum_thrusts + 1e-9
            ):
                saturated_steps += 1
            max_motor_command = max(max_motor_command, float(np.max(motors)))

        if not land_sent:
            max_altitude = max(max_altitude, float(vehicle.position[2]))
            max_horizontal_speed = max(
                max_horizontal_speed,
                float(np.linalg.norm(vehicle.velocity_world[:2])),
            )
            euler_deg = np.degrees(
                rotation_to_euler(vehicle.rotation_body_to_world)
            )
            max_roll_deg = max(max_roll_deg, abs(float(euler_deg[0])))
            max_pitch_deg = max(max_pitch_deg, abs(float(euler_deg[1])))
            max_yaw_error_deg = max(
                max_yaw_error_deg,
                abs(math.degrees(wrap_angle(float(euler_deg[2]) - yaw0))),
            )

        if hover_start_time is None:
            if state == FlightState.HOVERING:
                hover_start_time = simulation.data.time
                hover_anchor_xy = vehicle.position[:2].copy()
            continue

        if not land_sent:
            altitude_errors.append(
                float(vehicle.position[2] - target_height)
            )
            horizontal_drifts.append(
                float(np.linalg.norm(vehicle.position[:2] - hover_anchor_xy))
            )
            if simulation.data.time - hover_start_time >= hover_duration:
                send("land")
                land_sent = True
        elif state == FlightState.DISARMED:
            # Landing complete: the state machine disarmed on ground contact.
            break

    final_state = simulation.controller.read_state(simulation.data)
    landed = simulation.machine.state == FlightState.DISARMED and land_sent
    landed_altitude = float(final_state.position[2])
    landing_success = landed and landed_altitude < 0.15

    # The hover window ends exactly when "land" is sent.
    hover_time = hover_duration if land_sent else 0.0
    takeoff_time = (
        (hover_start_time - takeoff_request_time)
        if hover_start_time is not None
        else math.inf
    )
    # Final altitude/error is measured at the END of the hover window
    # (before the land command), from the last hover sample.
    final_altitude_error = (
        abs(altitude_errors[-1]) if altitude_errors else math.inf
    )
    final_hover_altitude = (
        target_height + altitude_errors[-1] if altitude_errors else math.nan
    )
    hover_rms_error = (
        math.sqrt(sum(e * e for e in altitude_errors) / len(altitude_errors))
        if altitude_errors
        else math.inf
    )
    max_drift = max(horizontal_drifts, default=math.inf)
    saturation_fraction = (
        saturated_steps / airborne_steps if airborne_steps else math.inf
    )
    max_tilt = max(max_roll_deg, max_pitch_deg)

    reasons: list[str] = []
    if hover_start_time is None:
        reasons.append("takeoff never reached HOVERING")
    if not land_sent:
        reasons.append("hover window never completed")
    if not landing_success:
        reasons.append("landing did not disarm near ground")
    if has_nan_or_inf:
        reasons.append("state or motor thrust contains NaN/Inf")
    if final_altitude_error > MAX_FINAL_ALTITUDE_ERROR:
        reasons.append(
            f"final altitude error {final_altitude_error:.3f} m > "
            f"{MAX_FINAL_ALTITUDE_ERROR} m"
        )
    if hover_rms_error > MAX_HOVER_RMS_ERROR:
        reasons.append(
            f"hover RMS error {hover_rms_error:.3f} m > "
            f"{MAX_HOVER_RMS_ERROR} m"
        )
    if max_drift > MAX_HORIZONTAL_DRIFT:
        reasons.append(
            f"horizontal drift {max_drift:.3f} m > {MAX_HORIZONTAL_DRIFT} m"
        )
    if max_tilt > MAX_TILT_DEG:
        reasons.append(f"max tilt {max_tilt:.2f} deg > {MAX_TILT_DEG} deg")
    if saturation_fraction > MAX_SATURATION_FRACTION:
        reasons.append(
            f"saturation fraction {saturation_fraction:.2f} > "
            f"{MAX_SATURATION_FRACTION}"
        )

    return M4AHoverReport(
        model_file=Path(model_path).name,
        mass_kg=config.mass_kg,
        wheelbase_m=config.wheelbase_m,
        thrust_to_weight_max=config.thrust_to_weight_max,
        target_altitude=target_height,
        takeoff_time=takeoff_time,
        max_altitude_overshoot=max(0.0, max_altitude - target_height),
        final_altitude=final_hover_altitude,
        final_altitude_error=final_altitude_error,
        hover_rms_error=hover_rms_error,
        max_horizontal_drift=max_drift,
        max_horizontal_speed=max_horizontal_speed,
        max_roll_deg=max_roll_deg,
        max_pitch_deg=max_pitch_deg,
        max_yaw_error_deg=max_yaw_error_deg,
        max_motor_command=max_motor_command,
        saturation_fraction=saturation_fraction,
        hover_duration_measured=hover_time,
        reached_hovering=hover_start_time is not None,
        landing_success=landing_success,
        landed_altitude=landed_altitude,
        has_nan_or_inf=has_nan_or_inf,
        total_time=float(simulation.data.time),
        passed=not reasons,
        reasons=tuple(reasons),
    )


def format_report(report: M4AHoverReport) -> str:
    lines = [
        "========== M4-A MICRO-UAV HOVER REPORT ==========",
        f"Model                 : {report.model_file}",
        f"Mass / wheelbase / T/W: {report.mass_kg:.3f} kg / "
        f"{report.wheelbase_m * 1000:.0f} mm / {report.thrust_to_weight_max:.1f}",
        f"Target altitude       : {report.target_altitude:.2f} m",
        "",
        f"Takeoff time          : {report.takeoff_time:.2f} s",
        f"Max altitude overshoot: {report.max_altitude_overshoot:.3f} m",
        f"Final altitude error  : {report.final_altitude_error:.3f} m",
        f"Hover RMS error       : {report.hover_rms_error:.3f} m",
        "",
        f"Max horizontal drift  : {report.max_horizontal_drift:.3f} m",
        f"Max horizontal speed  : {report.max_horizontal_speed:.3f} m/s",
        f"Max roll              : {report.max_roll_deg:.2f} deg",
        f"Max pitch             : {report.max_pitch_deg:.2f} deg",
        f"Max yaw error         : {report.max_yaw_error_deg:.2f} deg",
        "",
        f"Max motor command     : {report.max_motor_command:.3f} N",
        f"Saturation fraction   : {report.saturation_fraction:.3f}",
        f"Landing success       : {report.landing_success}",
        f"Total time            : {report.total_time:.2f} s",
        "",
        f"RESULT                : {'PASS' if report.passed else 'FAIL'}",
    ]
    for reason in report.reasons:
        lines.append(f"  - {reason}")
    lines.append("==================================================")
    return "\n".join(lines)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--target-height", type=positive_float, default=1.5)
    parser.add_argument("--hover-duration", type=positive_float, default=5.0)
    parser.add_argument("--max-total-time", type=positive_float, default=45.0)
    parser.add_argument(
        "--json",
        type=Path,
        default=Path("m4a_hover_report.json"),
        help="where to write machine-readable metrics (default: %(default)s)",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    report = run_m4a_hover_benchmark(
        target_height=args.target_height,
        hover_duration=args.hover_duration,
        max_total_time=args.max_total_time,
    )
    print(format_report(report))
    args.json.write_text(json.dumps(report.to_json_dict(), indent=2) + "\n")
    print(f"JSON report written to {args.json}")
    return 0 if report.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
