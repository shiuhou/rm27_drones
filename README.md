# RM27 autonomous micro-UAV development

## 進度與工作入口

[report.md](report.md) 是可單獨分享給 ChatGPT 的跨倉庫研發快照：包含各組件進度、
已驗證結果、失敗、工程假設、來源與本地／GitHub 差距。里程碑變更時更新同一檔案。
它整理本地研發成果，**不表示本分支及全部 submodule pins 已包含這些成果**。

| 要做的工作 | 工作位置／入口 |
|---|---|
| 跨組件契約、版本釘選、總進度 | 本 repo；[倉庫地圖](docs/REPOSITORY_MAP.md)、[總覽](report.md) |
| 共用物理、場景、完整任務與 viewer | [rm27_sim](https://github.com/shiuhou/rm27_sim)；本地沿用 `components/rm_uav_mujoco/` 名稱 |
| RL 訓練、reward、policy、Docker | [rm27_RL](https://github.com/shiuhou/rm27_RL) |
| PX4 固件／SITL 整合 | [rm27_drone_px4](https://github.com/shiuhou/rm27_drone_px4) |
| VIO／單目 SLAM | [rm27_vio](https://github.com/shiuhou/rm27_vio) ／ [rm27_slam](https://github.com/shiuhou/rm27_slam) |
| 可重用機型與場地資產 | [rm_mjcf](https://github.com/CosmosMount/rm_mjcf) |
| Tracking | [maixcam2_dart_vision](https://github.com/blademaster679/maixcam2_dart_vision)，在 `rm27/perception/vision/` 下 |

新仿真開發歸 Sim 的 `rm27_sim/`；根 `rm27/` 保有整合、歷史研究與遷移兼容責任。
本分支下方的 180 g／RMUC2026 命令與結果屬既有 root 基線；新 Ver2／2027 場地
應依對應 Sim 版本的 README 操作，不能把不同機型或導航來源的成功混為完整任務驗收。
父 repo 記錄固定 gitlink，`branch = main` 不會自動更新它。此文件修訂沒有更換 pins
或 `.gitmodules`；舊 simulator URL 與新版獨立 repo 的差異見 [總覽 §13](report.md)。

日常啟動與查看指令：[START_HERE.md](START_HERE.md)（繁體中文速查表）。

RoboMaster 2027 simulation and research workspace. The current priority is one
vehicle repeatedly completing dock → blue base armor impact → recovery → assigned
dock slot round trips. Quantity expands only after single-vehicle acceptance;
see [the milestone gates](docs/MILESTONES.md). The green light above
the armor is the 520 nm dart-guidance lamp used as a localization marker. The
research mission includes stabilizing after a potentially greater-than-10 m/s
armor-normal impact component and returning to dock; this value comes from the
RMUC 2026 42 mm detection rule, while the final RM2027 aerial-robot damage
threshold remains unknown. Interception is a later milestone.

The root project owns integration and experiment configuration; existing
simulator and upstream repositories remain independently versioned. Read the
[integration contract and status](docs/INTEGRATION.md) first. The full directory
ownership and migration map is in [docs/REPOSITORY_MAP.md](docs/REPOSITORY_MAP.md).
Evidence: [dock validation](docs/DOCK_VALIDATION.md) and
[preliminary base CAD inventory](docs/BASE_AUDIT.md).
Competition constraints and source hashes are tracked in the
[rulebook baseline](docs/RULEBOOK_BASELINE.md).
The base audit also includes commands for a separate static CAD preview.
Current real-aircraft parts and pending measurements are tracked in the
[real-hardware baseline](docs/REAL_HARDWARE.md). `docs/archive/chatgptre.md` is retained as
historical conversation context only and is not a current specification.

## Published root baseline (180 g / RMUC2026)

- `configs/scene.json`: field revision, named H platform, dock placement and
  sequential mission contract.
- `configs/drone.json`: user-reported 1103 9600KV, 2-inch props, approximately
  180 g; provisional 120 × 120 mm envelope. Unmeasured dynamics are labeled.
- `configs/real_hardware.json`: current MicoAir flight controller and sensor
  inventory, with unresolved installation/log fields left explicit.
- `rm27/`: CAD audit, dock conversion, scene composition, preview and single-vehicle flight gate.
- `tests/`: dock geometry, four-vehicle settling, isolated flight-interface and
  live-view state-isolation, calibration, contact and route checks (31 passing).
- `artifacts/`: generated models, renders and local evidence (ignored).

The dock is placed on the red H platform by default. Four separate physical
vehicle proxies occupy its slots. A separate [single-vehicle flight gate](docs/MICRO_FLIGHT.md)
uses real ArduPilot SITL for takeoff, hover, return and landing, with simulation
truth assisting precision return. The scene builder itself is a static preview.
This root baseline does not qualify base-contact missions or four simultaneous
ArduPilot instances. Later component/local studies have separate evidence in
[report.md](report.md).
A separate [provisional base target](docs/BASE_TARGET.md) adds a green marker
and collision test patch to the existing field base without replacing it.
The [S0 batch report](docs/S0_REPORT.md) records ten independent passing dock
flights. [Calibration workflow](docs/CALIBRATION_WORKFLOW.md) provides explicit
simulation dynamics profiles, read-only DataFlash intake and contact probes.
The [staging-route experiment](docs/CORRIDOR_REPORT.md) now reaches a point
in front of the base and returns to the dock using truth-assisted SITL guidance.
It does not include armor impact or visual navigation.

```bash
python -m pip install -r requirements-integration.txt
python -m rm27 build-dock --output artifacts/my-dock-scene
python -m rm27 view artifacts/my-dock-scene/scene.xml
python -m rm27.integration.check_scene artifacts/my-dock-scene/scene.xml
MUJOCO_GL=egl python -m rm27.simulation.render artifacts/my-dock-scene/scene.xml
python -m rm27 micro-flight artifacts/my-dock-scene/scene.xml --output artifacts/my-flight --instance 31
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests -q
```

Add `--view` to `micro-flight` for a live MuJoCo window with flight status.
Viewing defaults to requested 1× speed; headless runs default to 5×.
Use `--speedup` to override. Closing the live window cancels the test.

Use a new build output directory. This machine uses
`/home/shiuhou/venvs/mujoco/bin/python`. CAD source expected at
`references/cad_raw/dock.STEP`; raw CAD is local and excluded from Git.
Generated XML references sibling field assets by absolute path and must be
rebuilt after moving the workspace. The converter deliberately rejects
unreviewed geometry changes rather than silently reusing obsolete colliders.

## Components and baselines

| Path | Role |
|---|---|
| `rm27/autopilot/baselines/mujoco_uav_ardupilot` | Verified G1 ArduPilot–MuJoCo baseline, commit `2369d7d` |
| `rm27/simulation/field/rmuc2026_battlefield_mjcf` | Upstream field; development scene pins `67c1327` |
| `rm27/simulation/legacy/mujoco_uav` | Standalone M0–M4 vehicle/control experiments, source-only |
| `research/astra` | Independent observation-only terminal-guidance research |
| `rm27/perception/` | Unified sensing layer: visual SLAM/localization, camera calibration, target recognition, guidance and simulator producers |
| `rm27/perception/vision/` | M3C/OS04A10 camera/SLAM work and device-side MaixCAM2 green-light and armor recognition |
| `docs/MIGRATION_PROVENANCE.md` | Source commits and migration records for code now under `rm27/` |
| `rm27/experiments/legacy_sitl` | Historical standalone SITL mission scripts; current root missions live under `rm27/` |

G1's historical field was `dcf1202`; its results do not certify the updated
field or the new 180 g vehicle. Each asset/controller version requires its own
regression evidence. The canonical ArduPilot dependency is under
`rm27/autopilot/vendor/`; archived PX4 and historical workspaces live outside
the repository root in `../rm27_drones_legacy/`.

For a new checkout, initialize needed components with `git submodule update --init`
and install the integration dependencies above. Ensure the field checkout is
at the revision pinned by `configs/scene.json`; builds fail on a mismatch.
Raw CAD and the rules,
photo and video supplied locally are separate source inputs; do not assume a
clone alone includes them. No hardware actuation is provided by the root CLI.

Existing research: [Astra](research/astra/README.md), [research journal](research/astra/docs/research_log.md),
[hardware evidence](research/astra/docs/hardware.md), [real experiment protocol](research/astra/docs/real_experiments.md).
