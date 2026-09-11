# ArduPilot SITL + MuJoCo M3

A parallel JSON external-physics backend for the unchanged verified M3 plant.
Actual normal ARM → 1.5 m TAKEOFF → 10 s HOVER passed twice with identical physics.

Read [the integration report](REPORT.md) for audit, conventions, results and
limitations. [AUDIT.md](AUDIT.md) is the pre-implementation design record.

From this worktree's root:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m pip install -r ardupilot_bridge/requirements.txt
/home/shiuhou/venvs/mujoco/bin/python -m pytest tests ardupilot_bridge/tests -q
/home/shiuhou/venvs/mujoco/bin/python -m ardupilot_bridge.launch \
  --ardupilot ../ardupilot --instance 20 --speedup 5 \
  --output ardupilot_bridge/artifacts/new-hover-run
```

Requires the local built ArduCopter SITL binary. Use a fresh output directory.
The launcher checks parameters, leaves existing instance 0 alone, writes every
physics frame and MAVLink observations, then stops its own SITL after hover.

Physics is 500 Hz; the autopilot loop is 250 Hz to satisfy the checked-out
version's IMU rate arming requirement. The first packet advertises 1200 Hz before
JSON timestamp negotiation; subsequent packets are required to advertise 500 Hz.
