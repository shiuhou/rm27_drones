# RM27 development baselines

`baselines/` is the handoff and navigation layer for the four development
lines. It does not contain a second copy of the implementation. Each baseline
README records the current code locations, configuration, checks, evidence,
verified results and open assumptions across the root integration repository
and its independent components.

| Baseline | Handoff | Primary implementation boundary |
|---|---|---|
| RL | [rl/README.md](rl/README.md) | `components/rm27_RL/` plus root integration adapters |
| PX4 | [px4/README.md](px4/README.md) | `components/rm27_drone_px4/` plus root `rm27/autopilot/` integration |
| VIO | [vio/README.md](vio/README.md) | `components/rm27_vio/`, `components/rm27_slam/` and root perception interfaces |
| Tracking | [tracking/README.md](tracking/README.md) | Tracking vision submodule plus root target/guidance experiments |

Use the baseline README to decide where to begin work. Use `rm27/` for shared
integration, `components/` for independently versioned repositories,
`configs/` for reviewed configuration, `tests/` for root checks and
`artifacts/` for ignored generated evidence. A baseline entry does not imply
that its files should be moved into `baselines/`.

Every result in these handoffs must distinguish observed verification from
engineering estimates and unresolved assumptions.
