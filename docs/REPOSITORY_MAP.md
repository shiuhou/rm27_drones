# Repository map

This repository is one RM27 research project. The root integration package is
the main runtime entry point; the other directories provide research, external
components, source evidence, or generated results. This map is an orientation
guide, not a statement that the working tree is clean.

| Path | Responsibility | Owner | Independent Git repository | Movable? |
|---|---|---|---|---|
| `rm27/` | Unified root integration package: `integration/`, `simulation/`, `autopilot/`, `perception/` and `experiments/` | RM27 root | No | Yes, only with package/import updates |
| `baselines/` | Four development entry points: RL, PX4, VIO and Tracking | RM27 root | No | Documentation boundary; migrate code only with verified import/config updates |
| `components/` | Project-level submodules for the independent RM27 repositories | RM27 root | Yes, each child | Keep paths stable; update the parent commit when a component revision changes |
| `configs/` | Reviewed scene, vehicle, target, hardware and experiment configuration | RM27 root | No | Yes, with config path updates |
| `components/rm_mjcf/` | Reusable MuJoCo runtime assets: current Ver2 visual/dynamic aircraft, docks, bases, darts and fields; model conversion and experiments remain in root `rm27/` | `CosmosMount/rm_mjcf` submodule | Yes | Keep the pinned parent revision and update root config references when assets move |
| `tests/` | Root unit and integration tests | RM27 root | No | Yes, with test/config path updates |
| `docs/` | Current contracts, reports, milestones and historical archive | RM27 root | No | Yes |
| `rm27/perception/` | Complete RM27 sensing layer: vision (SLAM, camera calibration and target recognition), target estimates, guidance handoff and simulator producers | RM27 root | No | Yes, with import updates |
| `data/references/cad_raw/` | Raw STEP/SLDASM inputs used by reviewed integration configs | RM27 root | No | Yes, preserve hashes and update config paths |
| `data/flight_logs/data/` | Read-only historical and real-flight logs, grouped by source | RM27 root | No | Yes, preserve original files and hashes |
| `/home/shiuhou/Projects/archive/nested_legacy_workspaces_20261016/rm27_drones_legacy/` | Archived PX4, old RM27 and standalone ArduPilot reference workspace | Historical/reference owner | Contains independent repositories | Archive only; not a current runtime path |
| `/home/shiuhou/Projects/archive/nested_legacy_workspaces_20261016/rm27_drones_slam*/` | Archived SLAM worktree snapshots | Historical/reference owner | Independent snapshots | Archive only; use `components/rm27_slam` or `components/rm27_vio` |
| `research/astra/` | Observation-only terminal-guidance research | Astra research | No | Yes, as a whole research package |
| `research/Darts/` | Simplified dart trajectory and CAD collision-path experiment | Darts research | No | Yes, as a whole research package |
| `research/se3_radio_starter/` | SE3 controller, MuJoCo and radio-input reproduction package | SE3 research | No | Yes, as a whole research package |
| `rm27/perception/vision/` | Camera-based perception: M3C/OS04A10 calibration, monocular localization/SLAM, green-light and armor/target recognition | RM27 perception | Contains the `maixcam2_dart_vision` Git submodule | Move only as one perception unit; preserve nested Git history |
| `rm27/autopilot/vendor/` | External ArduPilot and Gazebo plugin checkouts used by the root launchers | RM27 autopilot | Yes, Git submodules | Move only with `.gitmodules` and launcher path updates |
| `rm27/autopilot/baselines/` | Frozen G1 ArduPilot–MuJoCo baseline and its historical evidence | RM27 autopilot | Yes, Git submodule | Keep pinned; do not mix with the current vehicle model |
| `rm27/simulation/field/` | Pinned upstream battlefield model used by `configs/scene.json` | RM27 simulation | Yes, Git submodule | Move only with scene and asset path updates |
| `rm27/simulation/legacy/` | Source-only M0–M4 simulator and older controller experiments | Historical/reference owner | Yes, Git submodule | Keep isolated from current simulation interfaces |
| `rm27/experiments/legacy_sitl/` | Historical direct pymavlink mission scripts moved from the M600 lab | RM27 experiments | No | Yes, as an archive; do not use as current mission entry |
| `data/` | Local references, read-only flight logs and patches grouped by type | RM27 root | No | Keep grouped by type; preserve originals and hashes |
| `artifacts/` | Ignored generated models, renders, trajectories and reports; currently contains many experiment output directories | RM27 root | No | Do not delete or bulk-move; classify outputs first |

## Ownership rule

Current RM27 behavior belongs in `rm27/`, `configs/`, `tests/` and `docs/`.
Research packages remain separate when they are independent studies. The
M3C/OS04A10, camera-calibration, SLAM and target-recognition work is part of
`rm27/perception/vision/` because it defines the project's camera sensing path.
`rm27/perception/camera/` documents the producer boundary; it is not a separate
research project. Third-party source and
historical results keep their original version boundaries.

## Safe reorganization order

1. Update this map and the directory READMEs.
2. Make output paths configurable and index existing artifacts.
3. Migrate reusable code into `rm27/` with tests and provenance records.
4. Move or archive old workspaces only after commands and submodule paths pass.

The detailed code migration sequence is maintained in
[`MIGRATION_PLAN.md`](MIGRATION_PLAN.md).

## Current orientation (2026-10-16)

- **Work here first:** `rm27/`, `configs/`, `tests/`, `docs/`, `research/` and
  the reviewed runtime assets in `components/rm_mjcf/`.
- **Project-level submodules:** `components/rm27_slam`,
  `components/rm27_vio`, `components/rm27_RL`, `components/rm27_drone_px4`,
  `components/rm_uav_mujoco` and `components/rm_mjcf` mirror the independent
  GitHub repositories used by the integration layer. Their pinned commits are
  recorded by the parent repository.
- **Generated evidence:** `artifacts/` is ignored by Git and has roughly 355
  output directories in this checkout. Names alone do not establish which
  runs are disposable; read each run's metadata before removal. See the
  [read-only artifact index](ARTIFACT_INDEX.md).
- **Historical or independent workspaces:** `rm27_drones_legacy/` remains a
  local historical tree. Retired worktrees and mixed VIO experiments are now
  under `/home/shiuhou/Projects/archive/` and are outside this repository.
- **Working-tree state:** there are existing root modifications and untracked
  files. Any future cleanup must distinguish those from changes made by the
  cleanup task.

### Root-level files

- `README.md`, `START_HERE.md` and `AGENTS.md` are navigation and operating
  rules.
- `requirements-integration.txt` and `requirements-contact-research.txt` are
  environment inputs; keep them at the root because commands reference them.
- `artifacts/_local/MUJOCO_LOG.TXT` is a local GLFW diagnostic log, not a project
  specification or test result.
- `artifacts/_local/pelican_bicycle.html` is a standalone local visual artifact
  and is not part of the RM27 runtime or documentation path.

Do not infer a clean repository from this map. Check root and nested Git status
separately before moving or deleting anything.
