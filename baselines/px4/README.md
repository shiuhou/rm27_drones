# PX4 baseline handoff

## Purpose

This line owns firmware-facing simulation, PX4 migration and closed-loop SITL
integration. It does not turn SITL or MuJoCo results into real-aircraft
calibration or flight qualification.

## Current implementation

- Primary repository: `components/rm27_drone_px4/` on
  `codex/px4-v1.17-migration`.
- Root integration: `rm27/autopilot/`, `rm27/integration/` and the root mission
  launchers.
- PX4's device-side Tracking dependency is the nested
  `rm27/perception/vision/maixcam2_dart_vision` submodule.

## Configuration, checks and evidence

- Configurations: `components/rm27_drone_px4/configs/`, root `configs/`, and
  the reviewed `configs/contact_phase4/` and `configs/contact_phase5/` studies.
- Component checks: `components/rm27_drone_px4/tests/`.
- Root checks: `tests/` for scene, flight-interface, contact and deployment
  contracts.
- Evidence: component `artifacts/` and the PX4 migration, SITL, image and
  contact reports under `components/rm27_drone_px4/docs/` and root `docs/`.

## Verified and unresolved

- Verified: selected PX4/SITL and headless simulation paths have reproducible
  reports with explicit limits; the parent and PX4 repositories resolve the
  same Tracking submodule commit.
- Unresolved: SITL does not prove board calibration, sensor timing, hardware
  imaging, real motor response or flight safety. Report headless, live-view,
  closed-loop SITL and hardware results separately.
- Firmware, parameters, ULogs and board identity must remain provenance-bearing
  inputs. Do not write hardware parameters or arm a vehicle as part of a
  software task.

## Start here

Read `components/rm27_drone_px4/AGENTS.md`, then its `README.md`,
`docs/INTEGRATION.md` and the relevant migration report. For root integration,
read `docs/INTEGRATION.md` and `docs/REAL_HARDWARE.md` before changing a
cross-component interface.
