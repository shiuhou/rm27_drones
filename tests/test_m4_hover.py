"""M4-A hover benchmark gate.

The 180 g / 120 mm micro plant must fly takeoff -> hover 5 s -> land with
the UNCHANGED M0 controller gains (M4-A changes the plant, not the
autonomy stack). Gates are engineering milestone gates, looser than M0:

* final altitude error <= 0.15 m
* hover RMS altitude error <= 0.10 m
* horizontal drift <= 0.15 m
* max roll/pitch <= 8 deg
* physical landing with disarm near ground
* no NaN/Inf, no permanent actuator saturation
"""

from __future__ import annotations

import unittest

from m4a_hover_benchmark import run_m4a_hover_benchmark


class M4AHoverBenchmarkTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report = run_m4a_hover_benchmark()

    def test_nominal_hover_passes_gates(self) -> None:
        self.assertTrue(
            self.report.passed, msg="; ".join(self.report.reasons)
        )

    def test_takeoff_reached_hover_and_landed(self) -> None:
        self.assertTrue(self.report.reached_hovering)
        self.assertTrue(self.report.landing_success)
        self.assertGreater(self.report.takeoff_time, 0.0)
        self.assertLess(self.report.takeoff_time, 15.0)

    def test_actuator_authority_not_permanently_saturated(self) -> None:
        self.assertLess(self.report.saturation_fraction, 0.20)
        # Hover at 0.44145 N with a 1.103625 N ceiling: some headroom must
        # remain even at the peak command.
        self.assertLess(self.report.max_motor_command, 1.103625)


if __name__ == "__main__":
    unittest.main()
