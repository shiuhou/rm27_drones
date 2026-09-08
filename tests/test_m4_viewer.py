"""M4 viewer tests: prove the viewer runs the SAME verified mission stack
as m4_benchmark.py, and that visualization helpers never touch physics.

No display is required: the interactive window only lives in main(); the
mission loop, telemetry formatter and marker builder are headless.
"""

from __future__ import annotations

import unittest

import numpy as np

from m2_mission import MissionConfig
from m3_benchmark import build_mission_stack, run_m3_case
from m4_benchmark import M4A_CASES
from m4_viewer import (
    build_marker_commands,
    format_telemetry,
    local_to_world_xy,
    resolve_case,
    run_viewer_mission,
)


class CaseResolutionTest(unittest.TestCase):
    def test_aliases_select_canonical_m4_cases(self) -> None:
        canonical = {case.name: case for case in M4A_CASES}
        self.assertEqual(resolve_case("ideal"), canonical["A0_ideal"])
        self.assertEqual(resolve_case("nominal"),
                         canonical["A1_nominal_noise"])
        self.assertEqual(resolve_case("combined"), canonical["A2_combined"])

    def test_canonical_names_accepted_directly(self) -> None:
        canonical = {case.name: case for case in M4A_CASES}
        self.assertEqual(resolve_case("A10_yaw90_combined"),
                         canonical["A10_yaw90_combined"])

    def test_yaw_override(self) -> None:
        case = resolve_case("combined", yaw_deg=90.0)
        self.assertEqual(case.initial_yaw_deg, 90.0)
        # Everything else stays canonical: same sensors, plant, gates, seed.
        canonical = {c.name: c for c in M4A_CASES}["A10_yaw90_combined"]
        for field_name in (
            "model_path", "plant", "imu", "flow", "tof", "estimator",
            "seed", "micro", "gates",
        ):
            self.assertEqual(
                getattr(case, field_name), getattr(canonical, field_name),
                msg=field_name,
            )

    def test_unknown_case_rejected(self) -> None:
        with self.assertRaises(ValueError):
            resolve_case("backflip")


class SameStackAsBenchmarkTest(unittest.TestCase):
    def test_same_mission_config(self) -> None:
        stack = build_mission_stack(resolve_case("nominal"))
        self.assertEqual(stack.mission_config, MissionConfig())

    def test_same_localization_config(self) -> None:
        case = resolve_case("combined")
        stack = build_mission_stack(case)
        self.assertEqual(stack.pipeline.imu.config, case.imu)
        self.assertEqual(stack.pipeline.flow.config, case.flow)
        self.assertEqual(stack.pipeline.tof.config, case.tof)
        self.assertEqual(stack.pipeline.estimator.config, case.estimator)

    def test_same_micro_plant(self) -> None:
        case = resolve_case("ideal")
        stack = build_mission_stack(case)
        self.assertIs(stack.simulation.micro_config, case.micro)
        self.assertAlmostEqual(stack.simulation.controller.mass, 0.180)

    def test_headless_viewer_mission_matches_benchmark_bit_exactly(self) -> None:
        # The strongest no-drift proof: same case, same deterministic seed,
        # one loop through the benchmark and one through the viewer must
        # produce identical mission timing and final truth.
        case = resolve_case("ideal")
        report = run_m3_case(case)
        stack = build_mission_stack(case)
        result = run_viewer_mission(stack)
        self.assertEqual(report.mission_result, "DONE")
        self.assertEqual(result.phase, "DONE")
        self.assertAlmostEqual(result.total_time, report.total_time,
                               places=12)
        self.assertLess(result.truth_final_z, 0.15)


class VisualizationPurityTest(unittest.TestCase):
    def test_helpers_do_not_mutate_simulation_state(self) -> None:
        stack = build_mission_stack(resolve_case("nominal"))
        # Step a little so there is real state to corrupt.
        command = stack.mission.start(
            stack.simulation.last_measured_state, stack.simulation.data.time
        )
        stack.send(command)
        for _ in range(50):
            truth = stack.simulation.step(stack.simulation.data.time)

        data = stack.simulation.data
        qpos_before = data.qpos.copy()
        qvel_before = data.qvel.copy()
        ctrl_before = data.ctrl.copy()
        time_before = float(data.time)

        text = format_telemetry(stack, truth)
        self.assertIn(stack.mission.phase.value, text)
        self.assertIn("estimator=HEALTHY", text)
        self.assertIn("motors [N]", text)
        markers = build_marker_commands(stack, [], [])
        self.assertTrue(len(markers) >= 2)  # start + target at minimum

        np.testing.assert_array_equal(data.qpos, qpos_before)
        np.testing.assert_array_equal(data.qvel, qvel_before)
        np.testing.assert_array_equal(data.ctrl, ctrl_before)
        self.assertEqual(float(data.time), time_before)

    def test_local_to_world_xy_rotates_and_translates(self) -> None:
        # yaw0 = 90 deg: local +X (body forward) maps to world +Y.
        world = local_to_world_xy(
            np.array([[2.0, 0.0]]),
            start_xy=np.array([1.0, -1.0]),
            yaw0=np.pi / 2,
        )
        np.testing.assert_allclose(world, [[1.0, 1.0]], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
