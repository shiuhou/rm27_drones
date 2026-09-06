"""M4-B hardware-measurement ingestion tests (synthetic fixtures only).

These tests exercise analyze_hardware_measurements.py against synthetic
CSV data written into temporary directories. No real thrust-stand data is
fabricated into hardware_measurements/ -- missing combinations must come
back as NEEDS_MEASUREMENT, and raw files must never be modified.
"""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from analyze_hardware_measurements import (
    GRAM_FORCE_TO_NEWTON,
    MeasurementValidationError,
    analyze_combination,
    analyze_directory,
    fit_monotonic_curve,
    load_airframe_csv,
    load_thrust_csv,
)


# Synthetic 2S + 2" curve: thrust_gf = 400 * t^2 on a 0..1 throttle sweep.
THROTTLE_STEPS = np.linspace(0.0, 1.0, 21)


def synthetic_rows(throttle=THROTTLE_STEPS):
    t = np.asarray(throttle, dtype=float)
    return {
        "throttle": t,
        "voltage_v": np.full_like(t, 7.4),
        "current_a": 8.0 * t**1.8,
        "thrust_gf": 400.0 * t**2,
        "rpm": [""] * len(t),  # rpm column present but blank
    }


def write_thrust_csv(path: Path, rows: dict) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["throttle", "voltage_v", "current_a", "thrust_gf",
                         "rpm"])
        for i in range(len(rows["throttle"])):
            writer.writerow(
                [
                    rows["throttle"][i],
                    rows["voltage_v"][i],
                    rows["current_a"][i],
                    rows["thrust_gf"][i],
                    rows["rpm"][i],
                ]
            )


def write_airframe_csv(path: Path, auw_kg: float = 0.180) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["parameter", "value", "unit", "notes"])
        writer.writerow(["auw_kg", auw_kg, "kg", "synthetic fixture"])
        writer.writerow(["wheelbase_m", 0.120, "m", "synthetic fixture"])


class ThrustCsvValidationTest(unittest.TestCase):
    def test_valid_csv_loads_and_normalizes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "thrust_2s_2in.csv"
            write_thrust_csv(path, synthetic_rows())
            measurement = load_thrust_csv(path)
        self.assertEqual(len(measurement.throttle), 21)
        self.assertAlmostEqual(float(measurement.throttle.max()), 1.0)
        # rpm column was blank everywhere -> NaN, not an error.
        self.assertTrue(np.all(np.isnan(measurement.rpm)))

    def test_percent_throttle_is_normalized_to_fraction(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x.csv"
            write_thrust_csv(path, synthetic_rows(throttle=np.linspace(0, 100, 11)))
            measurement = load_thrust_csv(path)
        self.assertAlmostEqual(float(measurement.throttle.max()), 1.0)

    def test_missing_column_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x.csv"
            path.write_text("throttle,voltage_v\n0.1,7.4\n")
            with self.assertRaises(MeasurementValidationError):
                load_thrust_csv(path)

    def test_non_numeric_thrust_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x.csv"
            rows = synthetic_rows()
            write_thrust_csv(path, rows)
            text = path.read_text().replace("16.0", "oops", 1)
            path.write_text(text)
            with self.assertRaises(MeasurementValidationError):
                load_thrust_csv(path)

    def test_negative_current_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x.csv"
            rows = synthetic_rows()
            rows["current_a"] = rows["current_a"].copy()
            rows["current_a"][3] = -0.5
            write_thrust_csv(path, rows)
            with self.assertRaises(MeasurementValidationError):
                load_thrust_csv(path)

    def test_ambiguous_throttle_range_rejected(self) -> None:
        # Max throttle 1.2 could be a broken 0-1 or a broken 0-100 scale;
        # the loader must refuse to guess.
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x.csv"
            write_thrust_csv(path, synthetic_rows(throttle=np.linspace(0, 1.2, 7)))
            with self.assertRaises(MeasurementValidationError):
                load_thrust_csv(path)

    def test_header_only_file_is_not_valid_data(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x.csv"
            path.write_text("throttle,voltage_v,current_a,thrust_gf,rpm\n")
            with self.assertRaises(MeasurementValidationError):
                load_thrust_csv(path)


class UnitConversionTest(unittest.TestCase):
    def test_gf_to_newton_uses_standard_gravity(self) -> None:
        self.assertAlmostEqual(GRAM_FORCE_TO_NEWTON, 0.00980665)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x.csv"
            rows = synthetic_rows()
            rows["thrust_gf"] = np.full_like(rows["throttle"], 500.0)
            write_thrust_csv(path, rows)
            measurement = load_thrust_csv(path)
        np.testing.assert_allclose(measurement.thrust_n, 500.0 * 0.00980665)

    def test_electrical_power_is_voltage_times_current(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x.csv"
            rows = synthetic_rows()
            rows["voltage_v"] = np.full_like(rows["throttle"], 7.4)
            rows["current_a"] = np.full_like(rows["throttle"], 5.0)
            write_thrust_csv(path, rows)
            measurement = load_thrust_csv(path)
        np.testing.assert_allclose(measurement.power_w, 37.0)


class CombinationAnalysisTest(unittest.TestCase):
    def _analyze(self, auw_kg=0.180):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "thrust_2s_2in.csv"
            write_thrust_csv(path, synthetic_rows())
            measurement = load_thrust_csv(path)
            return analyze_combination(measurement, auw_kg=auw_kg)

    def test_max_thrust_and_efficiency(self) -> None:
        result = self._analyze()
        self.assertEqual(result["status"], "OK")
        self.assertAlmostEqual(result["max_thrust_gf"], 400.0)
        self.assertAlmostEqual(result["max_thrust_n"], 400.0 * 0.00980665)
        self.assertAlmostEqual(result["throttle_at_max"], 1.0)
        self.assertAlmostEqual(result["current_at_max_a"], 8.0)
        self.assertAlmostEqual(result["power_at_max_w"], 59.2)
        expected_n_per_w = (400.0 * 0.00980665) / 59.2
        self.assertAlmostEqual(result["thrust_per_power_n_per_w"],
                               expected_n_per_w)
        self.assertAlmostEqual(result["thrust_per_current_n_per_a"],
                               (400.0 * 0.00980665) / 8.0)

    def test_thrust_to_weight_uses_four_motors_and_auw(self) -> None:
        result = self._analyze(auw_kg=0.180)
        weight = 0.180 * 9.80665
        expected = 4.0 * 400.0 * 0.00980665 / weight
        self.assertAlmostEqual(result["thrust_to_weight_max"], expected,
                               places=6)

    def test_hover_throttle_from_interpolation(self) -> None:
        # hover per motor = 0.180 * 9.80665 / 4 N = 45.015 gf
        # synthetic curve: t = sqrt(thrust_gf / 400) -> t = 0.3355
        result = self._analyze(auw_kg=0.180)
        self.assertAlmostEqual(result["hover_thrust_per_motor_n"],
                               0.180 * 9.80665 / 4.0, places=6)
        self.assertAlmostEqual(result["hover_throttle"], 0.3355, places=2)
        # current at hover from 8 * t^1.8 at t=0.3355 -> ~1.12 A
        expected_current = 8.0 * 0.3355**1.8
        self.assertAlmostEqual(result["hover_current_a"], expected_current,
                               delta=0.08)

    def test_hover_beyond_measured_range_is_not_extrapolated(self) -> None:
        result = self._analyze(auw_kg=5.0)  # needs 12.3 N/motor >> measured
        self.assertIsNone(result["hover_throttle"])
        self.assertIn("outside measured range", result["hover_note"])

    def test_missing_auw_leaves_hover_and_tw_unmeasured(self) -> None:
        result = self._analyze(auw_kg=None)
        self.assertEqual(result["thrust_to_weight_max"], "NEEDS_MEASUREMENT")
        self.assertEqual(result["hover_throttle"], "NEEDS_MEASUREMENT")
        self.assertEqual(result["hover_current_a"], "NEEDS_MEASUREMENT")


class MonotonicFitTest(unittest.TestCase):
    def test_fit_removes_non_monotonic_dip(self) -> None:
        throttle = np.linspace(0.0, 1.0, 21)
        thrust = 400.0 * throttle**2
        thrust[10] -= 30.0  # measurement dip
        fit_t, fit_y = fit_monotonic_curve(throttle, thrust)
        self.assertTrue(np.all(np.diff(fit_y) >= -1e-12))
        np.testing.assert_allclose(fit_t, throttle)

    def test_fit_is_exact_on_already_monotonic_data(self) -> None:
        throttle = np.linspace(0.0, 1.0, 11)
        thrust = 100.0 * throttle**2
        _, fit_y = fit_monotonic_curve(throttle, thrust)
        np.testing.assert_allclose(fit_y, thrust)


class DirectoryAnalysisTest(unittest.TestCase):
    def test_missing_combinations_stay_needs_measurement(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_thrust_csv(root / "thrust_2s_2in.csv", synthetic_rows())
            write_airframe_csv(root / "airframe_measurements.csv")
            report = analyze_directory(root, make_plots=False)
        combos = report["combinations"]
        self.assertEqual(combos["2s_2in"]["status"], "OK")
        for name in ("2s_2p5in", "3s_2in", "3s_2p5in"):
            self.assertEqual(combos[name]["status"], "NEEDS_MEASUREMENT")

    def test_header_only_file_counts_as_unmeasured(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "thrust_2s_2in.csv").write_text(
                "throttle,voltage_v,current_a,thrust_gf,rpm\n"
            )
            report = analyze_directory(root, make_plots=False)
        self.assertEqual(report["combinations"]["2s_2in"]["status"],
                         "NEEDS_MEASUREMENT")

    def test_raw_files_are_never_modified(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            raw = root / "thrust_2s_2in.csv"
            write_thrust_csv(raw, synthetic_rows())
            write_airframe_csv(root / "airframe_measurements.csv")
            before = {
                p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                for p in root.glob("*.csv")
            }
            analyze_directory(root, make_plots=False)
            after = {
                p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                for p in root.glob("*.csv")
            }
        self.assertEqual(before, after)

    def test_outputs_land_in_analysis_subdir(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_thrust_csv(root / "thrust_2s_2in.csv", synthetic_rows())
            write_airframe_csv(root / "airframe_measurements.csv")
            report = analyze_directory(root, make_plots=False)
            analysis_dir = root / "analysis"
            self.assertTrue((analysis_dir / "hardware_analysis.json").is_file())
            payload = json.loads(
                (analysis_dir / "hardware_analysis.json").read_text()
            )
            self.assertEqual(
                payload["combinations"]["2s_2in"]["status"], "OK"
            )
            self.assertEqual(report["airframe"]["auw_kg"], 0.180)

    def test_plots_written_when_matplotlib_available(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_thrust_csv(root / "thrust_2s_2in.csv", synthetic_rows())
            analyze_directory(root, make_plots=True)
            plot = root / "analysis" / "thrust_2s_2in_curve.png"
            self.assertTrue(plot.is_file())
            self.assertGreater(plot.stat().st_size, 1000)


class AirframeCsvTest(unittest.TestCase):
    def test_airframe_values_parse(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "airframe_measurements.csv"
            write_airframe_csv(path, auw_kg=0.165)
            values = load_airframe_csv(path)
        self.assertAlmostEqual(values["auw_kg"]["value"], 0.165)
        self.assertEqual(values["wheelbase_m"]["unit"], "m")


if __name__ == "__main__":
    unittest.main()
