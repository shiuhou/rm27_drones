"""M4-A benchmark tests.

The full A0-A10 mission matrix + sensor-rate study is flown by
`python m4_benchmark.py` (final regression). Here we pin the machinery:

* the case matrix exists with the required sensitivity coverage;
* every M4 case runs on the micro plant (micro config present) while the
  M3 cases stay on the generic plant (micro config absent);
* the fast-dynamics comparison is physically consistent;
* the ideal-sensor case A0 actually completes the mission end to end.
"""

from __future__ import annotations

import unittest

from m3_benchmark import M3_CASES, run_m3_case
from m4_benchmark import (
    M4_RATE_CASES,
    M4A_CASES,
    fast_dynamics_comparison,
)


class M4CaseMatrixTest(unittest.TestCase):
    def test_required_cases_exist(self) -> None:
        names = [case.name for case in M4A_CASES]
        for expected in (
            "A0_ideal",
            "A1_nominal_noise",
            "A2_combined",
            "A3_mass_150g",
            "A4_mass_200g",
            "A5_wheelbase_100mm",
            "A6_wheelbase_140mm",
            "A7_tw_2.0",
            "A8_tw_2.5",
            "A9_tw_3.0",
            "A10_yaw90_combined",
        ):
            self.assertIn(expected, names)

    def test_every_m4_case_uses_micro_plant(self) -> None:
        for case in (*M4A_CASES, *M4_RATE_CASES):
            self.assertIsNotNone(case.micro, msg=case.name)

    def test_m3_cases_stay_on_generic_plant(self) -> None:
        for case in M3_CASES:
            self.assertIsNone(case.micro, msg=case.name)

    def test_rate_study_is_one_factor_at_a_time(self) -> None:
        names = [case.name for case in M4_RATE_CASES]
        self.assertIn("rate_imu_125hz", names)
        self.assertIn("rate_imu_500hz", names)
        self.assertIn("rate_flow_25hz", names)
        self.assertIn("rate_flow_100hz", names)


class FastDynamicsComparisonTest(unittest.TestCase):
    def test_micro_is_faster_despite_shorter_arms(self) -> None:
        comparison = fast_dynamics_comparison()
        self.assertAlmostEqual(comparison["mass_ratio"], 0.180)
        # ~212x smaller roll inertia beats the ~3.85x shorter arm:
        # roll angular acceleration authority must INCREASE.
        self.assertGreater(comparison["micro_max_roll_alpha_rad_s2"],
                           comparison["generic_max_roll_alpha_rad_s2"])
        self.assertGreater(comparison["micro_max_yaw_alpha_rad_s2"],
                           comparison["generic_max_yaw_alpha_rad_s2"])


class M4IdealCaseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        case = next(c for c in M4A_CASES if c.name == "A0_ideal")
        cls.report = run_m3_case(case)

    def test_ideal_case_completes_mission(self) -> None:
        self.assertEqual(self.report.mission_result, "DONE",
                         msg=self.report.failure_reason)
        self.assertTrue(self.report.passed,
                        msg="; ".join(self.report.reasons))

    def test_report_echoes_micro_config(self) -> None:
        self.assertIsNotNone(self.report.micro)
        self.assertAlmostEqual(self.report.micro["mass_kg"], 0.180)


if __name__ == "__main__":
    unittest.main()
