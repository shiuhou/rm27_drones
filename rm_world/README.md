# Native RMUC2026 field + verified ArduPilot UAV

G1' bounded validation passed: native field collision probes, real SITL
ARM → 1.5 m takeoff → ten-second hover → 0.3 m translation → controlled deck
contact. No Gazebo dependency. See [REPORT.md](REPORT.md) for evidence and limits.

The sibling `../rmuc2026_battlefield_mjcf` must remain available at audited
commit `dcf1202f04e1ba31ab5dffac6b2c1c26f1129453`. Assets are referenced externally;
no license file was found, and no assets are vendored. Field coordinates and
collision geometry are preserved. **This field is not a complete aerial
collision envelope:** much of the upstream collision was designed for Z≤1 m.

Run from the `mujoco_uav_ardupilot` worktree root:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m pip install -r rm_world/requirements.txt
/home/shiuhou/venvs/mujoco/bin/python -m rm_world.validate_upstream
/home/shiuhou/venvs/mujoco/bin/python -m rm_world.probes
/home/shiuhou/venvs/mujoco/bin/python -m pytest tests ardupilot_bridge/tests rm_world/tests -q
/home/shiuhou/venvs/mujoco/bin/python -m rm_world.flight \
  --instance 21 --output rm_world/artifacts/new-field-run
```

The flight output directory must be new. Add `--hover-only` to stop after hover.
Default flight includes only the fixed validation experiment: translate over
the northwest deck, then descend slowly onto it. It does not implement navigation,
perception, target attack, or G2. The launcher stops its own SITL after the
post-contact observation; it leaves existing simulator processes alone.

```bash
/home/shiuhou/venvs/mujoco/bin/python -m ardupilot_bridge.launch \
  --instance 22 --output rm_world/artifacts/new-standalone-run
/home/shiuhou/venvs/mujoco/bin/python -m rm_world.performance \
  --field-run rm_world/artifacts/new-field-run \
  --baseline-run rm_world/artifacts/new-standalone-run
/home/shiuhou/venvs/mujoco/bin/python -m rm_world.replay rm_world/artifacts/new-field-run
```

Source responsibilities:

- `field_adapter.py`: deterministic XML composition, source asset references,
  safe spawn, wire-frame origin, contact and step-time logging.
- `flight.py`: fixed low-speed experiment using the existing bridge and runner.
- `probes.py`: full-field sphere contacts and throughput samples.
- `performance.py`, `replay.py`: measured timing, baseline comparison, determinism.
- `validate_upstream.py`, `ray_compat.hpp`: unchanged upstream C++ checks with
  an optional-output compatibility shim for the installed MuJoCo 3.12 ray API.
- `tests/`: compiled-geometry preservation, frame/spawn and physical probes.
- `evidence/`: compact reports and plot; `artifacts/`: ignored raw logs/builds.

Only `ardupilot_bridge/bridge.py` and `launch.py` gain optional dependency
injection / post-hover hooks and local-origin guard support. Their default
behavior remains the standalone hover benchmark; its physics hash was unchanged.
