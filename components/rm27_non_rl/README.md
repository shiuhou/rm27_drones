# RM27 Non-RL

Traditional model-based planning, guidance, state estimation and recovery for
RM27. ASM2 names identify the aircraft and existing experiments; the research
line is **RM27 Non-RL**.

```text
components/
├── rm27_non_rl/   # Planning, guidance, estimation, recovery and experiments
├── rm27_sim/      # Native physics, scenes, sensor simulation, scoring and replay
└── rm27_RL/       # RL training and evaluation; unchanged
```

`rm27_sim` never imports Non-RL for its shared native/sensor/replay APIs. The
Non-RL runtime consumes synthetic measurements; MuJoCo truth stays in sensor
generation and independent scoring. Existing M3 benchmark imports and historical
`rm27_sim.experiments.asm2_*` entries forward to the single canonical Non-RL
implementation. The old component path `../rm_uav_mujoco` is a compatibility
symlink for frozen evidence and RL consumers, not another implementation.

## Setup and entry points

Use the existing environment appropriate for the recorded aircraft/model. The
ASM2 research baseline uses MuJoCo 3.12.0; newer field experiments have separate
requirements. Do not upgrade a historical environment merely to install this
source layout. Install the three local packages without resolving new versions:

```bash
cd /home/shiuhou/Projects/rm27_drones
/home/shiuhou/venvs/mujoco/bin/python -m pip install --no-deps --no-build-isolation \
  -e components/rm27_sim/packages/physics \
  -e components/rm27_sim \
  -e components/rm27_non_rl
cd components/rm27_non_rl
PYTHONDONTWRITEBYTECODE=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27_non_rl --help
PYTHONDONTWRITEBYTECODE=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27_non_rl.asm2_estimation_replay --help
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest -q
```

The integrated entry takes explicit `--config`, `--case` and a new `--output`.
Task configs live here. Shared aircraft/target/field assets are resolved by
`rm27_sim.paths.ROOT`; sensor-generation configs stay with the simulator.
`data/` and `artifacts/` are links to shared inputs and preserved evidence, not
copies. New research outputs must use a new named directory; existing results
remain immutable. This component has not been assigned a new Git remote.
Installation is qualified for editable source checkouts. Portable wheels,
packaged model/config assets and deployment have not been qualified.

Read [SPEC](.agents/docs/SPEC.md), [DESIGN](.agents/docs/DESIGN.md) and
[ROADMAP](.agents/docs/ROADMAP.md) for the current contracts. Progress and the
remaining trial budget live only in the active plan's §0, as linked in AGENTS.
