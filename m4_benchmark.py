#!/usr/bin/env python3
"""M4-A micro-UAV benchmark: the M2 mission + M3 localization flown on the
180 g / 120 mm RM27-direction plant.

The autonomy stack is UNCHANGED from M3 (same mission, estimator, sensor
rates, truth-isolation). Only the plant differs, plus the inertia-scaled
attitude gains from m4_params.micro_controller_parameters (required for
discrete stability at 500 Hz; see that function's evidence note).

Case matrix:
    A0  nominal, ideal sensors
    A1  nominal, M3 nominal sensor noise
    A2  nominal, M3 combined localization realism
    A3/A4  mass 150/200 g sensitivity
    A5/A6  wheelbase 100/140 mm sensitivity
    A7/A8/A9  thrust-to-weight 2.0/2.5/3.0 sensitivity
    A10 combined localization, initial yaw 90 deg

Plus a one-factor-at-a-time sensor-rate study on the nominal plant:
IMU 125/250(base)/500 Hz, Flow 25/50(base)/100 Hz, ToF fixed at 50 Hz.

WARNING: 1103 9500 KV + 2"/2.5" + 2S/3S thrust/current/RPM performance is
NOT validated in M4-A. Max thrust comes from the assumed T/W ratio.

Usage:

    python m4_benchmark.py
    python m4_benchmark.py --json m4_micro_uav_report.json
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
from typing import Sequence

import mujoco
import numpy as np

from m2_benchmark import M0_MODEL
from m3_benchmark import (
    COMBINED_FLOW,
    COMBINED_IMU,
    COMBINED_TOF,
    M3Case,
    M3CaseReport,
    M3Gates,
    run_m3_case,
)
from m3_sensors import FlowConfig, IMUConfig, ToFConfig
from m4_params import (
    GRAVITY,
    M4A_NOMINAL,
    MicroUAVConfig,
    compute_inertia,
    provenance_table,
)


M4A_MODEL = Path(__file__).resolve().parent / "models" / "quadrotor_m4a.xml"

# M4-A mission gates: M3 forward/altitude/saturation gates, relaxed
# lateral (0.35 m) and tilt (10 deg nominal, 15 deg combined). The
# ground-penetration floor drops to 5 mm because the 12.5 mm-tall micro
# fuselage rests at ~0.011 m after touchdown (M3's 20 mm floor was sized
# for the 70 mm-tall 1 kg body).
M4_GATES_NOMINAL = M3Gates(lateral=0.35, tilt_deg=10.0, min_altitude_floor=0.005)
M4_GATES_COMBINED = M3Gates(lateral=0.35, tilt_deg=15.0, min_altitude_floor=0.005)

MICRO = M4A_NOMINAL


def _nominal_sensors(seed: int) -> dict:
    return {"imu": IMUConfig(), "flow": FlowConfig(), "tof": ToFConfig(),
            "seed": seed}


def _combined_sensors(seed: int = 27) -> dict:
    return {"imu": COMBINED_IMU, "flow": COMBINED_FLOW, "tof": COMBINED_TOF,
            "seed": seed}


M4A_CASES: tuple[M3Case, ...] = (
    M3Case("A0_ideal", "180 g / 120 mm / T/W 2.5, ideal sensors",
           M4A_MODEL, micro=MICRO, gates=M4_GATES_NOMINAL),
    M3Case("A1_nominal_noise", "nominal M3 sensor noise",
           M4A_MODEL, **_nominal_sensors(1),
           micro=MICRO, gates=M4_GATES_NOMINAL),
    M3Case("A2_combined", "M3 combined localization realism",
           M4A_MODEL, **_combined_sensors(),
           micro=MICRO, gates=M4_GATES_COMBINED),
    M3Case("A3_mass_150g", "mass sensitivity 150 g",
           M4A_MODEL, **_nominal_sensors(3),
           micro=MicroUAVConfig(mass_kg=0.150), gates=M4_GATES_NOMINAL),
    M3Case("A4_mass_200g", "mass sensitivity 200 g",
           M4A_MODEL, **_nominal_sensors(4),
           micro=MicroUAVConfig(mass_kg=0.200), gates=M4_GATES_NOMINAL),
    M3Case("A5_wheelbase_100mm", "wheelbase sensitivity 100 mm",
           M4A_MODEL, **_nominal_sensors(5),
           micro=MicroUAVConfig(wheelbase_m=0.100), gates=M4_GATES_NOMINAL),
    M3Case("A6_wheelbase_140mm", "wheelbase sensitivity 140 mm",
           M4A_MODEL, **_nominal_sensors(6),
           micro=MicroUAVConfig(wheelbase_m=0.140), gates=M4_GATES_NOMINAL),
    M3Case("A7_tw_2.0", "thrust-to-weight sensitivity 2.0",
           M4A_MODEL, **_nominal_sensors(7),
           micro=MicroUAVConfig(thrust_to_weight_max=2.0),
           gates=M4_GATES_NOMINAL),
    M3Case("A8_tw_2.5", "thrust-to-weight reference 2.5",
           M4A_MODEL, **_nominal_sensors(8),
           micro=MicroUAVConfig(thrust_to_weight_max=2.5),
           gates=M4_GATES_NOMINAL),
    M3Case("A9_tw_3.0", "thrust-to-weight sensitivity 3.0",
           M4A_MODEL, **_nominal_sensors(9),
           micro=MicroUAVConfig(thrust_to_weight_max=3.0),
           gates=M4_GATES_NOMINAL),
    M3Case("A10_yaw90_combined", "combined localization, initial yaw 90 deg",
           M4A_MODEL, **_combined_sensors(), initial_yaw_deg=90.0,
           micro=MICRO, gates=M4_GATES_COMBINED),
)

# One-factor-at-a-time sensor-rate study on the nominal micro plant.
# Baseline is A1 (IMU 250 Hz, Flow 50 Hz, ToF 50 Hz); ToF stays at 50 Hz.
M4_RATE_CASES: tuple[M3Case, ...] = (
    M3Case("rate_imu_125hz", "IMU rate 125 Hz (others baseline)",
           M4A_MODEL, imu=IMUConfig(rate_hz=125.0),
           flow=FlowConfig(), tof=ToFConfig(), seed=101,
           micro=MICRO, gates=M4_GATES_NOMINAL),
    M3Case("rate_imu_500hz", "IMU rate 500 Hz (others baseline)",
           M4A_MODEL, imu=IMUConfig(rate_hz=500.0),
           flow=FlowConfig(), tof=ToFConfig(), seed=102,
           micro=MICRO, gates=M4_GATES_NOMINAL),
    M3Case("rate_flow_25hz", "flow rate 25 Hz (others baseline)",
           M4A_MODEL, imu=IMUConfig(),
           flow=FlowConfig(rate_hz=25.0), tof=ToFConfig(), seed=103,
           micro=MICRO, gates=M4_GATES_NOMINAL),
    M3Case("rate_flow_100hz", "flow rate 100 Hz (others baseline)",
           M4A_MODEL, imu=IMUConfig(),
           flow=FlowConfig(rate_hz=100.0), tof=ToFConfig(), seed=104,
           micro=MICRO, gates=M4_GATES_NOMINAL),
)


def fast_dynamics_comparison() -> dict:
    """Analytic 1 kg generic vs 180 g micro comparison (no simulation).

    Torque ceilings use two motors per side at full vs zero thrust; angular
    acceleration = torque / inertia.
    """

    generic_model = mujoco.MjModel.from_xml_path(str(M0_MODEL))
    generic_body = mujoco.mj_name2id(
        generic_model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"
    )
    generic_mass = float(generic_model.body_mass[generic_body])
    generic_inertia = generic_model.body_inertia[generic_body].copy()
    generic_arm = 0.163  # models/quadrotor.xml rotor site |x| = |y|
    generic_max_thrust = float(generic_model.actuator_ctrlrange[0, 1])

    micro = M4A_NOMINAL
    micro_inertia = compute_inertia(micro)
    micro_arm = micro.motor_xy_m
    micro_max_thrust = micro.max_thrust_per_motor_n

    def authority(arm, max_thrust, yaw_coeff, inertia):
        roll_torque = 2.0 * arm * max_thrust
        yaw_torque = 2.0 * yaw_coeff * max_thrust
        return {
            "max_roll_pitch_torque_n_m": roll_torque,
            "max_yaw_torque_n_m": yaw_torque,
            "max_roll_alpha_rad_s2": roll_torque / float(inertia[0]),
            "max_pitch_alpha_rad_s2": roll_torque / float(inertia[1]),
            "max_yaw_alpha_rad_s2": yaw_torque / float(inertia[2]),
        }

    generic = authority(generic_arm, generic_max_thrust, 0.018, generic_inertia)
    micro_auth = authority(
        micro_arm, micro_max_thrust, micro.yaw_torque_per_thrust, micro_inertia
    )
    return {
        "mass_ratio": micro.mass_kg / generic_mass,
        "motor_arm_ratio": micro_arm / generic_arm,
        "inertia_ratio": [
            float(micro_inertia[i] / generic_inertia[i]) for i in range(3)
        ],
        "generic_hover_thrust_per_motor_n": generic_mass * GRAVITY / 4.0,
        "micro_hover_thrust_per_motor_n": micro.hover_thrust_per_motor_n,
        "generic_max_thrust_per_motor_n": generic_max_thrust,
        "micro_max_thrust_per_motor_n": micro_max_thrust,
        "generic_max_roll_alpha_rad_s2": generic["max_roll_alpha_rad_s2"],
        "micro_max_roll_alpha_rad_s2": micro_auth["max_roll_alpha_rad_s2"],
        "generic_max_pitch_alpha_rad_s2": generic["max_pitch_alpha_rad_s2"],
        "micro_max_pitch_alpha_rad_s2": micro_auth["max_pitch_alpha_rad_s2"],
        "generic_max_yaw_alpha_rad_s2": generic["max_yaw_alpha_rad_s2"],
        "micro_max_yaw_alpha_rad_s2": micro_auth["max_yaw_alpha_rad_s2"],
        "generic": generic,
        "micro": micro_auth,
    }


def _summary_row(report: M3CaseReport) -> str:
    return (
        f"{report.name:<20s} {report.mission_result:<6s} "
        f"fwd {report.truth_forward_at_land:5.2f}/"
        f"{report.est_forward_at_land:5.2f} m "
        f"lat {report.truth_lateral_at_land:+5.2f} m "
        f"posRMS {report.horizontal_position_rms_error:5.2f} m "
        f"altRMS {report.altitude_rms_error:5.2f} m "
        f"tilt {max(report.max_roll_deg, report.max_pitch_deg):5.1f} deg "
        f"sat {report.saturation_fraction:4.2f} "
        f"{'PASS' if report.passed else 'FAIL'}"
    )


def run_m4_benchmark(
    cases: Sequence[M3Case] = M4A_CASES,
    rate_cases: Sequence[M3Case] = M4_RATE_CASES,
) -> tuple[list[M3CaseReport], list[M3CaseReport]]:
    reports = [run_m3_case(case) for case in cases]
    rate_reports = [run_m3_case(case) for case in rate_cases]
    return reports, rate_reports


def build_report_dict(
    reports: list[M3CaseReport], rate_reports: list[M3CaseReport]
) -> dict:
    nominal = asdict(M4A_NOMINAL)
    nominal["derived"] = {
        "motor_radius_m": M4A_NOMINAL.motor_radius_m,
        "motor_xy_m": M4A_NOMINAL.motor_xy_m,
        "hover_thrust_per_motor_n": M4A_NOMINAL.hover_thrust_per_motor_n,
        "max_thrust_per_motor_n": M4A_NOMINAL.max_thrust_per_motor_n,
        "inertia_kg_m2": [float(v) for v in compute_inertia(M4A_NOMINAL)],
    }
    return {
        "milestone": "M4-A",
        "warning": (
            "1103 9500 KV + 2\"/2.5\" + 2S/3S thrust/current/RPM is NOT "
            "validated in M4-A; max thrust comes from assumed T/W ratios "
            "(ENGINEERING_ESTIMATE)."
        ),
        "nominal_config": nominal,
        "provenance": provenance_table(M4A_NOMINAL),
        "fast_dynamics_comparison": fast_dynamics_comparison(),
        "cases": [asdict(report) for report in reports],
        "rate_study": [asdict(report) for report in rate_reports],
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--json",
        type=Path,
        default=Path("m4_micro_uav_report.json"),
        help="where to write machine-readable metrics (default: %(default)s)",
    )
    args = parser.parse_args(argv)

    reports, rate_reports = run_m4_benchmark()

    print("========== M4-A MICRO-UAV MISSION MATRIX ==========")
    print(f"{'case':<20s} {'result':<6s} metrics...")
    for report in reports:
        print(_summary_row(report))
        for reason in report.reasons:
            print(f"    - {reason}")
    print()
    print("========== M4-A SENSOR-RATE STUDY (one factor at a time) ==========")
    for report in rate_reports:
        print(_summary_row(report))
        for reason in report.reasons:
            print(f"    - {reason}")

    args.json.write_text(
        json.dumps(build_report_dict(reports, rate_reports), indent=2) + "\n"
    )
    print(f"\nJSON report written to {args.json}")

    # The A-case matrix is the M4-A gate; the rate study is an experiment
    # whose per-case PASS/FAIL is reported above.
    all_passed = all(report.passed for report in reports)
    print(
        f"M4-A RESULT: {'PASS' if all_passed else 'FAIL'} "
        f"({sum(r.passed for r in reports)}/{len(reports)} A cases)"
    )
    return 0 if all_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
