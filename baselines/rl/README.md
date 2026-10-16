# RL baseline handoff

## Purpose

This line develops learned maneuver and recovery policies. It owns teacher
trajectories, behavior cloning, history/corrective policies and distillation.
It does not own PX4 firmware, VIO, camera calibration or real-flight claims.

## Current implementation

- Primary repository: `components/rm27_RL/` (`refactor/repository-separation`).
- Root integration and experiment adapters: `rm27/experiments/` and
  `rm27/integration/` where a policy is exercised by the RM27 plant.
- Main research material: `components/rm27_RL/packages/`, `configs/` and the
  component's `.agents/docs/SPEC.md` and `.agents/docs/DESIGN.md`.

## Configuration, checks and evidence

- Configurations: `components/rm27_RL/configs/` and root
  `configs/contact_phase3/`, `configs/contact_teacher*.json`.
- Component checks: `components/rm27_RL/tests/`.
- Root integration checks: `tests/test_contact_*.py`, policy and deployment
  tests that explicitly consume the learned-policy interfaces.
- Generated evidence: component `artifacts/` and root
  `artifacts/distill-*`, `artifacts/teacher-*`, `artifacts/phase3-*`.

## Verified and unresolved

- Verified: the repository has reproducible software and simulation paths for
  the recorded RL studies; each report identifies its own run and evaluation
  set.
- Unresolved: learned-policy results remain simulation evidence. They do not
  establish PX4 deployment, camera/VIO robustness, motor identification or
  real-aircraft safety.
- Keep checkpoint provenance, seed, train/evaluation split, plant revision and
  truth-versus-estimated inputs with every new result.

## Start here

Read `components/rm27_RL/AGENTS.md`, then its `SPEC.md` and `DESIGN.md`. For a
root mission integration, read `docs/INTEGRATION.md` and the relevant current
experiment report before changing the adapter.
