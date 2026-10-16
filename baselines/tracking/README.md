# Tracking baseline handoff

## Purpose

This line owns target observation and guidance: green-light and armor
recognition, target tracks, dart interception and the tracking-to-guidance
handoff. It produces observations; PX4 or the root mission decides how those
observations may be used for control.

## Current implementation

- Primary Tracking repository: `rm27/perception/vision/maixcam2_dart_vision`,
  GitHub `blademaster679/maixcam2_dart_vision`, branch `slam_drone`.
- The same repository is consumed as the nested Tracking submodule of
  `components/rm27_drone_px4/` and is pinned by the root integration repo.
- Root guidance and experiment code: `rm27/experiments/dart_interception/`
  and `research/Darts/`.
- Root perception adapters: `rm27/perception/vision/`.

## Configuration, checks and evidence

- Tracking configurations: `rm27/perception/vision/maixcam2_dart_vision/config/`
  and root `configs/dart_interception.json`,
  `configs/isolated_red_dart.json`.
- Tracking checks: `rm27/perception/vision/maixcam2_dart_vision/tests/` and
  root `tests/test_dart_interception.py`, `tests/test_isolated_red_dart.py`.
- Evidence: the Tracking repo's local `reports/`, `recordings/` and
  `artifacts/` inputs when available, plus root `docs/DART_INTERCEPTION.md`,
  `docs/ISOLATED_RED_DART.md` and matching root artifacts.

## Verified and unresolved

- Verified: host-side detector, tracker, JSON contract and selected replay or
  simulation paths have code-level checks; the repository preserves explicit
  fail-closed output states.
- Unresolved: target geometry, camera calibration, exposure/clock semantics,
  model artifacts, real-camera timing and the final 15–25 m field campaign are
  not established by code or unlabelled frame rate.
- Keep `npu.enabled=false`, `target_geometry.pose_enabled=false` and
  `safe_for_control=false` until the required calibration and model evidence
  exists. Do not call visual or recorded tracking a complete mission result.

## Start here

Read the Tracking repo's `AGENTS.md`, `README.md`,
`docs/GREEN_DETECTION_PLAN_FULL180.md` and
`docs/IMPLEMENTATION_STATUS_V0.2.md`. For root mission behavior, also read
`docs/INTEGRATION.md` and the relevant dart-interception report.
