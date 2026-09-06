#!/usr/bin/env python3
"""M4-B hardware-measurement ingestion and analysis.

Reads raw thrust-stand / airframe CSVs from hardware_measurements/ and
produces a machine-readable summary (JSON) plus plots in
hardware_measurements/analysis/. Raw measurement files are NEVER modified
or overwritten; missing combinations are reported as NEEDS_MEASUREMENT
and never fabricated.

Expected thrust CSV columns (one motor, one prop, one battery):

    throttle     command fraction 0..1 (or percent 0..100, auto-normalized)
    voltage_v    measured pack voltage under load [V]
    current_a    measured motor current [A]
    thrust_gf    measured thrust [gram-force]
    rpm          measured RPM (column required, values may be blank)

Expected airframe CSV columns:

    parameter,value,unit,notes   (e.g. auw_kg, 0.180, kg, "scale + props")

Nothing here feeds the MuJoCo plant yet; this is the M4-B data-ingestion
step only.

Usage:

    python analyze_hardware_measurements.py
    python analyze_hardware_measurements.py --dir hardware_measurements
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Sequence

import numpy as np


# 1 gram-force = 1 g * standard gravity (exact definition).
GRAM_FORCE_TO_NEWTON = 9.80665e-3
STANDARD_GRAVITY = 9.80665

NEEDS_MEASUREMENT = "NEEDS_MEASUREMENT"

EXPECTED_THRUST_FILES = {
    "2s_2in": "thrust_2s_2in.csv",
    "2s_2p5in": "thrust_2s_2p5in.csv",
    "3s_2in": "thrust_3s_2in.csv",
    "3s_2p5in": "thrust_3s_2p5in.csv",
}
AIRFRAME_FILE = "airframe_measurements.csv"

REQUIRED_THRUST_COLUMNS = ("throttle", "voltage_v", "current_a",
                           "thrust_gf", "rpm")
AIRFRAME_COLUMNS = ("parameter", "value", "unit")


class MeasurementValidationError(ValueError):
    """Raised when a raw measurement CSV fails validation."""


@dataclass(frozen=True)
class ThrustMeasurement:
    """Validated, normalized thrust-stand sweep (one motor/prop/battery)."""

    source_file: str
    throttle: np.ndarray       # 0..1, strictly increasing
    voltage_v: np.ndarray
    current_a: np.ndarray
    thrust_gf: np.ndarray
    rpm: np.ndarray            # NaN where the raw cell was blank

    @property
    def thrust_n(self) -> np.ndarray:
        return self.thrust_gf * GRAM_FORCE_TO_NEWTON

    @property
    def power_w(self) -> np.ndarray:
        return self.voltage_v * self.current_a


def _parse_float(text: str, *, column: str, row: int,
                 allow_blank: bool = False) -> float:
    text = text.strip()
    if text == "":
        if allow_blank:
            return math.nan
        raise MeasurementValidationError(
            f"row {row}: column '{column}' is blank"
        )
    try:
        value = float(text)
    except ValueError as error:
        raise MeasurementValidationError(
            f"row {row}: column '{column}' is not numeric: {text!r}"
        ) from error
    if not math.isfinite(value):
        raise MeasurementValidationError(
            f"row {row}: column '{column}' is not finite: {text!r}"
        )
    return value


def _normalize_throttle(throttle: np.ndarray) -> np.ndarray:
    """Accept a 0..1 fraction or a 0..100 percent sweep; refuse ambiguity."""

    if np.any(throttle < 0.0):
        raise MeasurementValidationError("throttle must be non-negative")
    peak = float(throttle.max())
    if peak > 1.5:
        return throttle / 100.0
    if peak > 1.0 + 1e-9:
        raise MeasurementValidationError(
            f"ambiguous throttle range (max {peak}): expected 0..1 or 0..100"
        )
    return throttle


def load_thrust_csv(path: Path) -> ThrustMeasurement:
    """Load and validate one thrust-stand CSV."""

    with Path(path).open(newline="") as handle:
        reader = csv.reader(handle)
        rows = [row for row in reader if any(cell.strip() for cell in row)]
    if not rows:
        raise MeasurementValidationError(f"{path}: empty file")
    header = [cell.strip() for cell in rows[0]]
    missing = [c for c in REQUIRED_THRUST_COLUMNS if c not in header]
    if missing:
        raise MeasurementValidationError(
            f"{path}: missing columns {missing}; "
            f"expected {list(REQUIRED_THRUST_COLUMNS)}"
        )
    data_rows = rows[1:]
    if len(data_rows) < 2:
        raise MeasurementValidationError(
            f"{path}: need at least 2 data rows, found {len(data_rows)}"
        )

    index = {name: header.index(name) for name in REQUIRED_THRUST_COLUMNS}
    columns: dict[str, list[float]] = {name: [] for name in index}
    for row_number, row in enumerate(data_rows, start=2):
        for name, column_index in index.items():
            cell = row[column_index] if column_index < len(row) else ""
            columns[name].append(
                _parse_float(cell, column=name, row=row_number,
                             allow_blank=(name == "rpm"))
            )

    throttle = _normalize_throttle(np.asarray(columns["throttle"]))
    voltage = np.asarray(columns["voltage_v"])
    current = np.asarray(columns["current_a"])
    thrust_gf = np.asarray(columns["thrust_gf"])
    rpm = np.asarray(columns["rpm"])

    if np.any(voltage <= 0.0):
        raise MeasurementValidationError(f"{path}: voltage_v must be > 0")
    if np.any(current < 0.0):
        raise MeasurementValidationError(f"{path}: current_a must be >= 0")
    if np.any(thrust_gf < 0.0):
        raise MeasurementValidationError(f"{path}: thrust_gf must be >= 0")
    if np.any(np.isfinite(rpm) & (rpm <= 0.0)):
        raise MeasurementValidationError(f"{path}: rpm must be > 0 when given")

    # Sort by throttle and consolidate duplicate commands (mean of samples).
    order = np.argsort(throttle, kind="stable")
    throttle, voltage, current, thrust_gf, rpm = (
        values[order]
        for values in (throttle, voltage, current, thrust_gf, rpm)
    )
    unique_throttle, inverse, counts = np.unique(
        throttle, return_inverse=True, return_counts=True
    )
    if len(unique_throttle) < 2:
        raise MeasurementValidationError(
            f"{path}: need at least 2 distinct throttle settings"
        )
    if len(unique_throttle) != len(throttle):
        def consolidate(values):
            sums = np.zeros_like(unique_throttle)
            np.add.at(sums, inverse, values)
            return sums / counts

        voltage = consolidate(voltage)
        current = consolidate(current)
        thrust_gf = consolidate(thrust_gf)
        rpm = consolidate(np.where(np.isfinite(rpm), rpm, np.nan))
        throttle = unique_throttle

    return ThrustMeasurement(
        source_file=Path(path).name,
        throttle=throttle,
        voltage_v=voltage,
        current_a=current,
        thrust_gf=thrust_gf,
        rpm=rpm,
    )


def load_airframe_csv(path: Path) -> dict:
    """Load parameter,value,unit,notes airframe measurements."""

    with Path(path).open(newline="") as handle:
        reader = csv.reader(handle)
        rows = [row for row in reader if any(cell.strip() for cell in row)]
    if not rows:
        raise MeasurementValidationError(f"{path}: empty file")
    header = [cell.strip() for cell in rows[0]]
    missing = [c for c in AIRFRAME_COLUMNS if c not in header]
    if missing:
        raise MeasurementValidationError(
            f"{path}: missing columns {missing}; expected {list(AIRFRAME_COLUMNS)}"
        )
    index = {name: header.index(name) for name in header}
    values: dict[str, dict] = {}
    for row_number, row in enumerate(rows[1:], start=2):
        name = row[index["parameter"]].strip()
        if not name:
            continue
        values[name] = {
            "value": _parse_float(
                row[index["value"]], column="value", row=row_number
            ),
            "unit": row[index["unit"]].strip(),
            "notes": (
                row[index["notes"]].strip()
                if "notes" in index and index["notes"] < len(row)
                else ""
            ),
        }
    return values


def _pava(values: np.ndarray) -> np.ndarray:
    """Pool Adjacent Violators: unweighted isotonic (non-decreasing) fit."""

    blocks: list[list[float]] = []  # [level, count]
    for value in values:
        blocks.append([float(value), 1])
        while len(blocks) >= 2 and blocks[-2][0] > blocks[-1][0]:
            level2, count2 = blocks.pop()
            level1, count1 = blocks.pop()
            total = count1 + count2
            blocks.append(
                [(level1 * count1 + level2 * count2) / total, total]
            )
    return np.concatenate(
        [np.full(count, level) for level, count in blocks]
    )


def fit_monotonic_curve(
    throttle: np.ndarray, values: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Isotonic (non-decreasing) fit of `values` against `throttle`.

    A thrust-vs-command curve must be monotonic to be usable as a mixer
    reference; measurement noise can locally violate that, so we pool the
    violating neighbors (PAVA) rather than deleting data.
    """

    throttle = np.asarray(throttle, dtype=float)
    values = np.asarray(values, dtype=float)
    order = np.argsort(throttle, kind="stable")
    sorted_throttle = throttle[order]
    fitted = _pava(values[order])
    return sorted_throttle, fitted


def analyze_combination(
    measurement: ThrustMeasurement, auw_kg: float | None = None
) -> dict:
    """Derived metrics for one motor/prop/battery combination."""

    thrust_n = measurement.thrust_n
    power_w = measurement.power_w
    peak = int(np.argmax(thrust_n))

    fit_throttle, fit_thrust_n = fit_monotonic_curve(
        measurement.throttle, thrust_n
    )

    result: dict = {
        "status": "OK",
        "source_file": measurement.source_file,
        "sample_count": int(len(measurement.throttle)),
        "max_thrust_gf": float(measurement.thrust_gf[peak]),
        "max_thrust_n": float(thrust_n[peak]),
        "throttle_at_max": float(measurement.throttle[peak]),
        "current_at_max_a": float(measurement.current_a[peak]),
        "power_at_max_w": float(power_w[peak]),
        "thrust_per_current_n_per_a": float(
            thrust_n[peak] / measurement.current_a[peak]
        )
        if measurement.current_a[peak] > 0.0
        else None,
        "thrust_per_power_n_per_w": float(thrust_n[peak] / power_w[peak])
        if power_w[peak] > 0.0
        else None,
        "thrust_per_power_gf_per_w": float(
            measurement.thrust_gf[peak] / power_w[peak]
        )
        if power_w[peak] > 0.0
        else None,
        "rpm_available": bool(np.any(np.isfinite(measurement.rpm))),
        "monotonic_curve": {
            "throttle": [float(v) for v in fit_throttle],
            "thrust_n": [float(v) for v in fit_thrust_n],
        },
    }

    if auw_kg is None:
        result["thrust_to_weight_max"] = NEEDS_MEASUREMENT
        result["hover_thrust_per_motor_n"] = NEEDS_MEASUREMENT
        result["hover_throttle"] = NEEDS_MEASUREMENT
        result["hover_current_a"] = NEEDS_MEASUREMENT
        result["hover_note"] = "AUW not measured"
        return result

    hover_n = auw_kg * STANDARD_GRAVITY / 4.0
    result["thrust_to_weight_max"] = float(
        4.0 * thrust_n[peak] / (auw_kg * STANDARD_GRAVITY)
    )
    result["hover_thrust_per_motor_n"] = float(hover_n)

    if not (fit_thrust_n[0] <= hover_n <= fit_thrust_n[-1]):
        result["hover_throttle"] = None
        result["hover_current_a"] = None
        result["hover_note"] = (
            f"hover thrust {hover_n:.3f} N outside measured range "
            f"[{fit_thrust_n[0]:.3f}, {fit_thrust_n[-1]:.3f}] N; "
            "refusing to extrapolate"
        )
        return result

    # Invert the fitted monotonic thrust curve to get the hover command,
    # then read current off the measured current-vs-throttle samples.
    hover_throttle = float(
        np.interp(hover_n, fit_thrust_n, fit_throttle)
    )
    hover_current = float(
        np.interp(hover_throttle, measurement.throttle, measurement.current_a)
    )
    hover_voltage = float(
        np.interp(hover_throttle, measurement.throttle, measurement.voltage_v)
    )
    result["hover_throttle"] = hover_throttle
    result["hover_current_a"] = hover_current
    result["hover_power_w"] = hover_current * hover_voltage
    result["hover_note"] = "interpolated from fitted monotonic curve"
    return result


def _plot_combination(measurement: ThrustMeasurement, result: dict,
                      output_dir: Path) -> str | None:
    """Write a 2-panel thrust/current plot; returns the PNG filename."""

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    name = Path(measurement.source_file).stem
    figure, (ax_thrust, ax_current) = plt.subplots(
        2, 1, figsize=(7, 7), sharex=True
    )
    ax_thrust.plot(measurement.throttle, measurement.thrust_n, "o",
                   markersize=4, label="measured")
    ax_thrust.plot(
        result["monotonic_curve"]["throttle"],
        result["monotonic_curve"]["thrust_n"],
        "-", label="monotonic fit",
    )
    if isinstance(result.get("hover_throttle"), float):
        ax_thrust.axvline(result["hover_throttle"], color="gray",
                          linestyle="--", linewidth=1,
                          label=f"hover throttle {result['hover_throttle']:.2f}")
    ax_thrust.set_ylabel("thrust [N]")
    ax_thrust.set_title(
        f"{name}: max {result['max_thrust_n']:.2f} N "
        f"({result['max_thrust_gf']:.0f} gf)"
    )
    ax_thrust.legend()
    ax_thrust.grid(True, alpha=0.3)

    ax_current.plot(measurement.throttle, measurement.current_a, "s-",
                    markersize=4, color="tab:red", label="current")
    ax_current.set_xlabel("throttle command [0-1]")
    ax_current.set_ylabel("current [A]")
    ax_power = ax_current.twinx()
    ax_power.plot(measurement.throttle, measurement.power_w, "^-",
                  markersize=3, color="tab:orange", label="power")
    ax_power.set_ylabel("electrical power [W]")
    ax_current.grid(True, alpha=0.3)

    figure.tight_layout()
    output_path = output_dir / f"{name}_curve.png"
    figure.savefig(output_path, dpi=120)
    plt.close(figure)
    return output_path.name


def analyze_directory(
    measurements_dir: Path,
    output_dir: Path | None = None,
    make_plots: bool = True,
) -> dict:
    """Analyze every expected combination; write JSON (+ plots).

    Writes only into `output_dir` (default: <measurements_dir>/analysis).
    Raw CSVs are opened read-only and never modified.
    """

    measurements_dir = Path(measurements_dir)
    output_dir = (
        Path(output_dir)
        if output_dir is not None
        else measurements_dir / "analysis"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    airframe: dict = {"status": NEEDS_MEASUREMENT, "auw_kg": None}
    airframe_path = measurements_dir / AIRFRAME_FILE
    if airframe_path.is_file():
        try:
            values = load_airframe_csv(airframe_path)
            auw = values.get("auw_kg")
            airframe = {
                "status": "OK" if auw is not None else NEEDS_MEASUREMENT,
                "auw_kg": auw["value"] if auw is not None else None,
                "parameters": values,
            }
        except MeasurementValidationError as error:
            airframe = {
                "status": "INVALID",
                "auw_kg": None,
                "error": str(error),
            }

    combinations: dict[str, dict] = {}
    for combo, filename in EXPECTED_THRUST_FILES.items():
        path = measurements_dir / filename
        if not path.is_file():
            combinations[combo] = {"status": NEEDS_MEASUREMENT,
                                   "expected_file": filename}
            continue
        try:
            measurement = load_thrust_csv(path)
        except MeasurementValidationError as error:
            if "at least 2 data rows" in str(error):
                # Header-only template: no data yet, not a malformed file.
                combinations[combo] = {
                    "status": NEEDS_MEASUREMENT,
                    "expected_file": filename,
                    "note": "file present but contains no data rows",
                }
                continue
            raise
        result = analyze_combination(measurement, auw_kg=airframe["auw_kg"])
        if make_plots:
            plot_name = _plot_combination(measurement, result, output_dir)
            if plot_name is not None:
                result["plot"] = plot_name
        combinations[combo] = result

    report = {
        "milestone": "M4-B data ingestion (not yet used by the plant)",
        "measurements_dir": measurements_dir.name,
        "gram_force_to_newton": GRAM_FORCE_TO_NEWTON,
        "airframe": airframe,
        "combinations": combinations,
        "missing": [
            combo
            for combo, entry in combinations.items()
            if entry["status"] == NEEDS_MEASUREMENT
        ],
    }
    (output_dir / "hardware_analysis.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    return report


def format_summary(report: dict) -> str:
    lines = ["========== HARDWARE MEASUREMENT ANALYSIS =========="]
    airframe = report["airframe"]
    if airframe.get("auw_kg") is not None:
        lines.append(f"AUW                   : {airframe['auw_kg']:.3f} kg")
    else:
        lines.append(f"AUW                   : {airframe['status']}")
    for combo, entry in report["combinations"].items():
        if entry["status"] != "OK":
            lines.append(f"{combo:<22s}: {entry['status']}")
            continue
        lines.append(
            f"{combo:<22s}: max {entry['max_thrust_n']:.3f} N "
            f"({entry['max_thrust_gf']:.0f} gf) @ "
            f"throttle {entry['throttle_at_max']:.2f}, "
            f"{entry['current_at_max_a']:.2f} A, "
            f"{entry['power_at_max_w']:.1f} W, "
            f"{entry['thrust_per_power_gf_per_w']:.2f} gf/W"
        )
        tw = entry["thrust_to_weight_max"]
        hover = entry["hover_throttle"]
        tw_text = f"{tw:.2f}" if isinstance(tw, float) else tw
        hover_text = f"{hover:.3f}" if isinstance(hover, float) else str(hover)
        lines.append(
            f"{'':<22s}  T/W {tw_text}, hover throttle {hover_text}"
        )
    lines.append("===================================================")
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--dir",
        type=Path,
        default=Path(__file__).resolve().parent / "hardware_measurements",
        help="directory holding the raw CSVs (default: %(default)s)",
    )
    parser.add_argument(
        "--no-plots", action="store_true", help="skip PNG generation"
    )
    args = parser.parse_args(argv)

    report = analyze_directory(args.dir, make_plots=not args.no_plots)
    print(format_summary(report))
    print(f"\nJSON written to {args.dir / 'analysis' / 'hardware_analysis.json'}")
    # Exit 0 even with missing data: NEEDS_MEASUREMENT is an expected state.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
