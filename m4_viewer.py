#!/usr/bin/env python3
"""M4-A interactive mission viewer (visualization only).

Watch the verified 180 g / 120 mm micro-UAV fly the M2 mission on the M3
localization stack, rendered by MuJoCo's interactive viewer:

    python m4_viewer.py --case ideal
    python m4_viewer.py --case nominal
    python m4_viewer.py --case combined
    python m4_viewer.py --case combined --yaw 90

The mission stack is IDENTICAL to m4_benchmark.py: the same canonical
M4A_CASES definitions and the same build_mission_stack() construction
(M4-A plant -> M3 sensors -> M3 estimator -> M2 mission -> controller ->
mixer -> MuJoCo). This file adds only rendering, real-time pacing, and
terminal telemetry. It never writes qpos/qvel for visualization.

Controls: SPACE pause/resume, R restart mission, close window / ESC exits.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field, replace
import math
import time
from typing import Sequence

import numpy as np

from fly import SimulationSafetyError, rotation_to_euler, wrap_angle
from m2_mission import MissionPhase, MissionStateMachine
from m3_benchmark import (
    HEALTH_GATED_PHASES,
    HEALTH_STARTUP_GRACE_S,
    MAX_TOTAL_TIME,
    M3Case,
    MissionStack,
    build_mission_stack,
)
from m4_benchmark import M4A_CASES


CASE_ALIASES = {
    "ideal": "A0_ideal",
    "nominal": "A1_nominal_noise",
    "combined": "A2_combined",
}

TELEMETRY_PERIOD_S = 0.15  # ~7 Hz terminal telemetry
TRAIL_PERIOD_S = 0.05  # trajectory marker sampling

TRUTH_RGBA = (0.1, 0.9, 0.9, 0.9)  # cyan trail = physical truth
EST_RGBA = (1.0, 0.2, 0.9, 0.9)  # magenta trail = estimator belief
START_RGBA = (0.2, 0.9, 0.2, 0.9)  # green = mission start
TARGET_RGBA = (1.0, 0.6, 0.1, 0.9)  # orange = 5 m target


def resolve_case(name: str, yaw_deg: float | None = None) -> M3Case:
    """Map a CLI alias (or canonical name) to the canonical M4A_CASES entry."""

    canonical = {case.name: case for case in M4A_CASES}
    canonical_name = CASE_ALIASES.get(name, name)
    if canonical_name not in canonical:
        raise ValueError(
            f"unknown case {name!r}; expected one of "
            f"{sorted(CASE_ALIASES)} or canonical names {sorted(canonical)}"
        )
    case = canonical[canonical_name]
    if yaw_deg is not None:
        case = replace(case, initial_yaw_deg=float(yaw_deg))
    return case


def local_to_world_xy(
    points_xy: np.ndarray, start_xy: np.ndarray, yaw0: float
) -> np.ndarray:
    """Estimator local-nav-frame XY -> MuJoCo world XY (scoring/render only)."""

    points = np.asarray(points_xy, dtype=float)
    cos0, sin0 = math.cos(yaw0), math.sin(yaw0)
    rotation = np.array([[cos0, -sin0], [sin0, cos0]])
    return points @ rotation.T + np.asarray(start_xy, dtype=float)


def format_telemetry(stack: MissionStack, truth) -> str:
    """One telemetry block. Pure formatting; reads state, never writes."""

    simulation = stack.simulation
    estimated = simulation.last_estimated_state
    est_state = simulation.last_measured_state
    t = float(simulation.data.time)

    truth_forward, truth_lateral = stack.truth_frame.project(truth.position[:2])
    cos0, sin0 = math.cos(stack.truth_yaw0), math.sin(stack.truth_yaw0)
    truth_vel_local = np.array(
        [
            cos0 * truth.velocity_world[0] + sin0 * truth.velocity_world[1],
            -sin0 * truth.velocity_world[0] + cos0 * truth.velocity_world[1],
            truth.velocity_world[2],
        ]
    )
    truth_mission_pos = np.array(
        [truth_forward, truth_lateral, truth.position[2]]
    )
    est_pos = est_state.position
    est_vel = est_state.velocity_world
    position_error = float(
        np.linalg.norm(est_pos[:2] - truth_mission_pos[:2])
    )
    velocity_error = float(np.linalg.norm(est_vel - truth_vel_local))

    est_yaw = float(rotation_to_euler(est_state.rotation_body_to_world)[2])
    truth_yaw_rel = wrap_angle(
        float(rotation_to_euler(truth.rotation_body_to_world)[2])
        - stack.truth_yaw0
    )
    yaw_error = math.degrees(abs(wrap_angle(est_yaw - truth_yaw_rel)))

    pipeline = stack.pipeline
    motors = simulation.last_motor_thrusts

    def vec3(values) -> str:
        return f"({values[0]:+6.2f},{values[1]:+6.2f},{values[2]:+6.2f})"

    lines = [
        "─" * 64,
        f"t={t:6.2f}s  phase={stack.mission.phase.value:<8s} "
        f"estimator={'HEALTHY' if estimated.healthy else 'UNHEALTHY'}",
        f"sensors: IMU age {estimated.imu_age * 1000:4.0f}ms | "
        f"Flow age {estimated.flow_age * 1000:4.0f}ms "
        f"({pipeline.flow_samples} samples, {pipeline.flow_invalid} invalid) | "
        f"ToF age {estimated.tof_age * 1000:4.0f}ms "
        f"({pipeline.tof_samples} samples, {pipeline.tof_invalid} invalid)",
        f"position  truth {vec3(truth_mission_pos)}  "
        f"est {vec3(est_pos)}  err {position_error:5.3f} m",
        f"velocity  truth {vec3(truth_vel_local)}  "
        f"est {vec3(est_vel)}  err {velocity_error:5.3f} m/s",
        f"yaw rel   truth {math.degrees(truth_yaw_rel):+7.2f} deg  "
        f"est {math.degrees(est_yaw):+7.2f} deg  err {yaw_error:5.2f} deg",
        f"forward   truth {truth_forward:+5.2f} m   est {est_pos[0]:+5.2f} m",
        f"lateral   truth {truth_lateral:+5.2f} m   est {est_pos[1]:+5.2f} m",
        "motors [N] FL FR RR RL: "
        + " ".join(f"{value:5.3f}" for value in motors),
    ]
    return "\n".join(lines)


def build_marker_commands(
    stack: MissionStack,
    truth_points: Sequence[np.ndarray],
    est_points: Sequence[np.ndarray],
    max_geoms: int = 900,
) -> list[tuple]:
    """Visualization-only marker descriptors (spheres and line segments).

    Pure function: produces a list the GUI applies to viewer.user_scn.
    Nothing here touches MjData.
    """

    frame = stack.truth_frame
    config = stack.mission_config
    spawn_z = float(
        stack.simulation.model.qpos0[stack.simulation.controller.qpos_address + 2]
    )
    start_world = np.array(
        [frame.origin_xy[0], frame.origin_xy[1], spawn_z]
    )
    target_xy = local_to_world_xy(
        np.array([[config.forward_distance, 0.0]]), frame.origin_xy, frame.yaw
    )[0]
    target_world = np.array(
        [target_xy[0], target_xy[1], config.takeoff_altitude]
    )

    commands: list[tuple] = [
        ("sphere", start_world, 0.05, START_RGBA),
        ("sphere", target_world, 0.07, TARGET_RGBA),
        (
            "line",
            np.array([target_world[0], target_world[1], 0.0]),
            target_world,
            1.5,
            TARGET_RGBA,
        ),
    ]

    def trail(points, rgba):
        if len(points) < 2:
            return
        # Subsample so both trails fit inside the scene's geom budget.
        stride = max(
            1, math.ceil(2 * (len(points) - 1) / max(1, max_geoms - 4))
        )
        sampled = points[::stride]
        for first, second in zip(sampled, sampled[1:]):
            commands.append(("line", first, second, 2.0, rgba))

    trail(list(truth_points), TRUTH_RGBA)
    trail(list(est_points), EST_RGBA)
    return commands


@dataclass
class ViewerMissionResult:
    phase: str
    failure_reason: str | None
    total_time: float
    final_time: float
    truth_final_z: float
    truth_forward: float
    est_forward: float
    truth_lateral: float
    est_lateral: float


def run_viewer_mission(
    stack: MissionStack,
    on_step=None,
    max_total_time: float = MAX_TOTAL_TIME,
) -> ViewerMissionResult:
    """The benchmark's mission loop, optionally with a per-step callback.

    Identical construction and stepping to run_m3_case: simulation.step ->
    estimator health gate -> mission.update -> send -> phase/timeout checks.
    `on_step(stack, truth)` may return "abort" (used for pause/restart/close
    in the GUI); with on_step=None this is the deterministic headless path
    used by tests.
    """

    simulation = stack.simulation
    mission = stack.mission
    command = mission.start(simulation.last_measured_state,
                            simulation.data.time)
    stack.send(command)

    abort_reason: str | None = None
    while True:
        try:
            truth = simulation.step(simulation.data.time)
        except SimulationSafetyError as error:
            abort_reason = f"SIMULATION_SAFETY: {error}"
            break
        t = float(simulation.data.time)
        estimated = simulation.last_estimated_state

        if (
            mission.phase in HEALTH_GATED_PHASES
            and t > HEALTH_STARTUP_GRACE_S
            and not estimated.healthy
        ):
            abort_reason = f"ESTIMATOR_UNHEALTHY in {mission.phase.value}"
            break

        command = mission.update(
            simulation.last_measured_state,
            simulation.machine.state.value,
            t,
        )
        stack.send(command)

        if on_step is not None and on_step(stack, truth) == "abort":
            abort_reason = "viewer aborted (restart or window closed)"
            break
        if mission.phase in (MissionPhase.DONE, MissionPhase.FAILED):
            break
        if t > max_total_time:
            abort_reason = "BENCHMARK_TIMEOUT"
            break

    truth = simulation.controller.read_state(simulation.data)
    truth_forward, truth_lateral = stack.truth_frame.project(
        truth.position[:2]
    )
    est_final = simulation.last_measured_state.position
    log = mission.log
    total_time = float(
        log.get("done_time", math.nan) - log.get("takeoff_entry_time", math.nan)
    )
    return ViewerMissionResult(
        phase=mission.phase.value,
        failure_reason=mission.failure_reason or abort_reason,
        total_time=total_time,
        final_time=float(simulation.data.time),
        truth_final_z=float(truth.position[2]),
        truth_forward=float(truth_forward),
        est_forward=float(est_final[0]),
        truth_lateral=float(truth_lateral),
        est_lateral=float(est_final[1]),
    )


def reset_stack(stack: MissionStack) -> None:
    """R-key restart: full deterministic reset of the mission stack."""

    stack.simulation.reset_model()
    stack.pipeline = stack.simulation.localization
    stack.mission = MissionStateMachine(stack.mission_config)
    stack.sequence = 0


def _apply_markers(scn, commands: list[tuple]) -> None:
    """Copy marker descriptors into the viewer's user scene (render only)."""

    import mujoco

    scn.ngeom = 0
    for command in commands:
        if scn.ngeom >= scn.maxgeom:
            break
        geom = scn.geoms[scn.ngeom]
        if command[0] == "sphere":
            _, position, radius, rgba = command
            mujoco.mjv_initGeom(
                geom,
                mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([radius, 0.0, 0.0]),
                np.asarray(position, dtype=float),
                np.eye(3).flatten(),
                np.asarray(rgba, dtype=float),
            )
        else:
            _, first, second, width, rgba = command
            mujoco.mjv_initGeom(
                geom,
                mujoco.mjtGeom.mjGEOM_LINE,
                np.zeros(3),
                np.zeros(3),
                np.eye(3).flatten(),
                np.asarray(rgba, dtype=float),
            )
            mujoco.mjv_connector(
                geom,
                mujoco.mjtGeom.mjGEOM_LINE,
                width,
                np.asarray(first, dtype=float),
                np.asarray(second, dtype=float),
            )
        scn.ngeom += 1


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--case",
        default="ideal",
        help="ideal | nominal | combined | canonical M4 case name",
    )
    parser.add_argument(
        "--yaw",
        type=float,
        default=None,
        help="initial true yaw in degrees (overrides the case default)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="real-time factor (1.0 = watch in real time)",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    case = resolve_case(args.case, args.yaw)
    stack = build_mission_stack(case)

    import mujoco.viewer

    gui = {"paused": False, "restart": False}

    def key_callback(keycode: int) -> None:
        if keycode == ord(" "):
            gui["paused"] = not gui["paused"]
        elif keycode == ord("R"):
            gui["restart"] = True

    print(f"Case: {case.name} — {case.description}")
    print(f"Initial true yaw: {case.initial_yaw_deg:.1f} deg")
    print("Controls: SPACE pause/resume, R restart, close window to exit.")
    print("Markers: green=start, orange=5 m target, cyan=truth, "
          "magenta=estimator belief\n")

    with mujoco.viewer.launch_passive(
        stack.simulation.model, stack.simulation.data,
        key_callback=key_callback,
    ) as viewer:
        state = {
            "wall0": None,
            "sim0": 0.0,
            "last_telemetry": -math.inf,
            "last_trail": -math.inf,
            "truth_trail": [],
            "est_trail": [],
        }

        def on_step(stack: MissionStack, truth):
            simulation = stack.simulation
            t = float(simulation.data.time)
            if state["wall0"] is None:
                state["wall0"] = time.perf_counter()
                state["sim0"] = t

            # Real-time pacing (viewer only; benchmarks run unthrottled).
            target_wall = (
                state["wall0"] + (t - state["sim0"]) / max(args.speed, 1e-6)
            )
            delay = target_wall - time.perf_counter()
            if delay > 0.0:
                time.sleep(min(delay, 0.1))

            if t - state["last_trail"] >= TRAIL_PERIOD_S:
                state["last_trail"] = t
                state["truth_trail"].append(truth.position.copy())
                est_xy_world = local_to_world_xy(
                    np.array([simulation.last_measured_state.position[:2]]),
                    stack.truth_frame.origin_xy,
                    stack.truth_yaw0,
                )[0]
                state["est_trail"].append(
                    np.array(
                        [
                            est_xy_world[0],
                            est_xy_world[1],
                            simulation.ground_height
                            + simulation.last_measured_state.position[2],
                        ]
                    )
                )

            if t - state["last_telemetry"] >= TELEMETRY_PERIOD_S:
                state["last_telemetry"] = t
                print(format_telemetry(stack, truth))

            _apply_markers(
                viewer.user_scn,
                build_marker_commands(
                    stack, state["truth_trail"], state["est_trail"]
                ),
            )
            viewer.sync()

            while (
                gui["paused"]
                and viewer.is_running()
                and not gui["restart"]
            ):
                time.sleep(0.05)
                viewer.sync()
            if gui["restart"] or not viewer.is_running():
                return "abort"
            return None

        while viewer.is_running():
            result = run_viewer_mission(stack, on_step=on_step)
            print("\n" + "=" * 64)
            print(
                f"MISSION {result.phase}"
                + (f" ({result.failure_reason})" if result.failure_reason else "")
            )
            print(
                f"truth forward {result.truth_forward:5.2f} m | "
                f"est forward {result.est_forward:5.2f} m | "
                f"total time {result.total_time:.2f} s"
            )
            print("=" * 64 + "\nPress R to fly again, or close the window.")

            while viewer.is_running() and not gui["restart"]:
                time.sleep(0.05)
                viewer.sync()
            if not viewer.is_running():
                break
            gui["restart"] = False
            reset_stack(stack)
            state.update(
                wall0=None,
                last_telemetry=-math.inf,
                last_trail=-math.inf,
                truth_trail=[],
                est_trail=[],
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
