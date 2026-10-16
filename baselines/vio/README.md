# VIO baseline handoff

## Purpose

This line owns Camera + IMU visual-inertial odometry, localization contracts,
calibration evidence and estimator-facing interfaces. Monocular SLAM research
is kept in its separate component; neither line owns PX4 control.

## Current implementation

- VIO repository: `components/rm27_vio/` on `main`; current estimator line is
  OpenVINS.
- SLAM repository: `components/rm27_slam/` on `main`; it is the independent
  MaixCAM2 monocular research line.
- Root integration interfaces: `rm27/perception/vision/`,
  `rm27/perception/camera/` and `rm27/perception/guidance/`.

## Configuration, checks and evidence

- Configurations and contracts: component `config/` or experiment inputs,
  `components/rm27_vio/rm27/perception/vision/docs/`, and root `configs/`.
- Component checks: `components/rm27_vio/tests/` and
  `components/rm27_slam/tests/`.
- Root checks: `tests/test_localization_contract.py` and related estimator and
  deployment checks.
- Evidence: VIO/SLAM component reports and root `artifacts/slam-*` or
  estimator-related run directories when their metadata is present.

## Verified and unresolved

- Verified: software contracts, synthetic/replay paths and selected estimator
  interfaces have explicit tests and reports.
- Unresolved: VIO-P/S0 remain partial until their original synchronization,
  completeness, clock and observation-model gates are met. Replay or synthetic
  pose does not prove camera/IMU calibration, online tracking, drift or flight.
- Preserve timestamps, frames, calibration provenance, scale semantics and
  backend state. Never silently restore the archived SLAM fallback to VIO main.

## Start here

Read `components/rm27_vio/AGENTS.md`, `HANDOFF.md` and the Camera/IMU data
contract. For monocular SLAM work, enter `components/rm27_slam/` and read its
own `AGENTS.md`, `README.md`, `docs/STATUS.md` and `HANDOFF.md`.
