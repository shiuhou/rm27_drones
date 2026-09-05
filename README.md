# MuJoCo Quadrotor with Independent Pygame Controller

這是一個最小、可驗證的 MuJoCo 四旋翼 baseline。MuJoCo 物理模擬與 Pygame 飛行控制器是兩個獨立 process、兩個獨立視窗，透過 localhost UDP 傳送高層飛行命令與遙測。

Pygame 是唯一飛行輸入視窗。MuJoCo Viewer 只負責顯示模擬與相機操作，沒有註冊 WASD、Q/E、R/F、T、L 等飛行 callback。Pygame 不會修改 MuJoCo state，也不會發送四個馬達推力。

## 實驗室脈絡（RM UAV Lab）

本 repo 是 RM UAV Lab 工作區中的 **MuJoCo 自主飛行堆疊子專案**。工作區
內還存在另一套模擬工作（刻意**不**收進本 repo）：

- **ArduPilot SITL**（`~/rm_uav_lab/ardupilot/`）：未修改的上游 checkout
  （6.6 GB，origin 為 ArduPilot/ardupilot.git），隨時可重新 clone，因此
  不複製進版控。
- **Gazebo 工作區**（`~/rm_uav_lab/gz_ws/`）：ArduPilot SITL 搭配的
  Gazebo 模擬環境。
- **pymavlink 任務腳本**（`~/rm_uav_lab/missions/`）：SITL 上的
  takeoff → forward → land 系列，演進到 **V5 軌跡腳本**
  （`takeoff_forward_land_v5_trajectory.py`）。這條 V5 軌跡是本專案的
  最終重現目標之一；M0 的原始目的就是在重現它之前先建立可信、可測的
  MuJoCo baseline。
- 其他：`reset_sim_v2.sh`、`params/`、`notes/`、`tools/`。

排除它們的原因：ardupilot/ 是公開上游程式碼且無本地修改；gz_ws 含有
大量第三方 vendored 套件；GitHub 單檔 100 MB 上限。這些 SITL/Gazebo
產物日後適合獨立成一個 `rm_uav_sitl` repo。

## Milestone 進程（M0–M4-A 已驗證）

| Milestone | 內容 | 驗證 |
|---|---|---|
| M0 | 理想剛體 plant + 位置/SO(3) 控制器 baseline；物理 sanity tests（自由落體、懸停推力 m·g/4、mixer 方向）；1.5 m 起飛 + 5 s 懸停 benchmark | `m0_benchmark.py` PASS |
| M1 | 溫和 Sim-to-Real robustness：馬達一階延遲（40 ms）、流體阻力、質量/慣量 ±10%、馬達效率不一致、seeded 量測雜訊；z 軸小積分項 | `m1_benchmark.py` PASS（10 cases） |
| M2 | 自主任務：起飛 → 前進 5 m（相對起飛航向）→ 煞車 → 懸停 → 降落；**mission 只看 measured state，ground truth 只用於評分** | `m2_benchmark.py` PASS（12 cases） |
| M3 | GPS-denied 定位：IMU（250 Hz）+ optical-flow-like 速度（50 Hz）+ ToF 高度（50 Hz）→ complementary predict/correct estimator → EstimatedState 驅動 mission 與 controller；truth 結構性隔離 | `m3_benchmark.py` PASS（11 cases） |
| M4-A | RM27 方向微型機 plant：180 g / 120 mm X quad，質量/幾何/慣量/推力餘裕（T/W 2.5 假設）縮放；autonomy stack 不變；attitude 增益依慣量比縮放（必要，見下文） | `m4_benchmark.py` PASS（11 A cases + 4 rate cases） |

驗證狀態：`python -m pytest -q` → **167 passed**；上述五個 benchmark
全部 PASS。Git 歷史即里程碑封存：M0 `5631a1f` → M1 `ad152a7` →
M2 `7789a81` → M3 `2be0029`（M4-A 待 review 後封存）。

下一步（M4-B，待 review 後開始）：馬達/槳/電池/COM 細節 realism——
馬達延遲掃描、2S vs 3S 推力包線、2" vs 2.5" 推力曲線、COM offset、
微型機 drag fitting。這些需要真實量測數據，M4-A 全部標為
NEEDS_MEASUREMENT。

## 專案結構

```text
.
├── models/
│   ├── quadrotor.xml          # M0 理想模型：1 kg X 型四旋翼與四個 site actuators
│   ├── quadrotor_m1.xml       # M1 模型：filterexact 馬達延遲 + MuJoCo 流體阻力
│   └── quadrotor_m4a.xml      # M4-A 微型機模型：180 g / 120 mm X quad（T/W 2.5 假設）
├── fly.py                     # 核心位置/SO(3)姿態控制器與相容 hover CLI
├── simulator.py               # MuJoCo、flight state machine、IPC server
├── pygame_controller.py       # 獨立 Pygame UI、鍵盤焦點與 IPC client
├── pygame_fly.py              # pygame_controller.py 的相容入口
├── ipc_protocol.py            # 封包、JSON encode/decode 與驗證
├── launch.py                  # 啟動並清理兩個子程序
├── m0_benchmark.py            # M0 起飛/懸停 headless benchmark（輸出報告與 JSON）
├── m1_realism.py              # M1 plant perturbation（質量/慣量/馬達效率/drag）與量測雜訊層
├── m1_benchmark.py            # M1 robustness 矩陣 benchmark（輸出 m1_robustness_report.json）
├── m2_mission.py              # M2 任務狀態機（READY→TAKEOFF→…→DONE/FAILED）與 heading-relative frame
├── m2_benchmark.py            # M2 自主任務 benchmark（輸出 m2_mission_report.json）
├── m3_sensors.py              # M3 感測器模型：IMU / optical-flow-like 速度 / ToF 高度（非 datasheet 數值）
├── m3_estimator.py            # M3 定位估計器：IMU 預測 + flow/ToF 修正（complementary，非 Kalman）
├── m3_benchmark.py            # M3 定位任務 benchmark（輸出 m3_localization_report.json）
├── m4_params.py               # M4-A micro-UAV 參數 + provenance + 慣量元件模型 + 慣量比增益縮放
├── m4a_hover_benchmark.py     # M4-A 起飛/懸停/降落 benchmark（輸出 m4a_hover_report.json）
├── m4_benchmark.py            # M4-A 任務矩陣 + 感測器速率實驗（輸出 m4_micro_uav_report.json）
├── requirements.txt
├── pytest.ini
└── tests/
    ├── test_protocol.py
    ├── test_flight_sequence.py
    ├── test_controller_safety.py
    ├── test_udp_integration.py
    ├── test_smoke.py
    ├── test_m0_physics.py     # 自由落體 / 懸停推力 / mixer 方向的物理 sanity tests
    ├── test_m0_benchmark.py   # M0 驗收：1.5 m 起飛 + 5 s 懸停
    ├── test_m1_motor_dynamics.py  # 一階馬達延遲步階響應
    ├── test_m1_measurements.py    # 量測雜訊：seed 重現性、零雜訊、合法旋轉
    ├── test_m1_robustness.py      # 質量/慣量/效率 perturbation、drag、不對稱暫態
    ├── test_m1_benchmark.py       # M1 驗收：10 個 case 的 robustness 矩陣
    ├── test_m2_mission.py         # M2 任務狀態機：phase 轉移、timeout、measured/truth 分離
    ├── test_m2_frames.py          # M2 heading-relative frame 投影
    ├── test_m2_benchmark.py       # M2 驗收：12 個 case 的任務矩陣 + heading 不變性
    ├── test_m3_sensors.py         # M3 感測器：符號慣例、body-frame 轉換、seed、rate、latency、dropout
    ├── test_m3_estimator.py       # M3 估計器：積分、scale/bias 漂移、dropout health、quaternion 合法性
    ├── test_m3_truth_isolation.py # M3 結構性 truth 隔離：stub 估計器證明 mission/controller 只吃估計值
    ├── test_m3_benchmark.py       # M3 驗收：11 個 case + 長斷訊失敗測試
    ├── test_m4_micro_physics.py   # M4-A 物理：質量/幾何/慣量/mixer 方向/馬達順序/增益縮放
    ├── test_m4_hover.py           # M4-A 驗收：起飛 + 5 s 懸停 + 降落
    └── test_m4_benchmark.py       # M4-A 驗收：case 矩陣結構 + A0 全程任務
```

## 建議閱讀順序

如果你想從「模型如何產生推力」一路讀到「鍵盤如何讓飛機移動」，建議依下列順序查閱：

1. `models/quadrotor.xml`：先認識機體座標、質量/慣量、旋翼位置與 actuator。
2. `fly.py`：理解位置控制、SO(3) 姿態控制與四旋翼 mixer。
3. `ipc_protocol.py`：查看 Pygame 與 simulator 之間允許傳送的資料格式。
4. `simulator.py`：追蹤飛行狀態機、起降軌跡、手動 target 積分與 MuJoCo stepping。
5. `pygame_controller.py`：理解鍵盤焦點保護、按鍵映射、命令發送與遙測畫面。
6. `launch.py`：查看兩個獨立 process 如何一起啟動與結束。
7. `tests/`：用測試案例確認封包邊界、狀態轉移、安全條件與完整飛行流程。

一個持續按住 `W` 的命令會沿著這條資料流前進：

```text
pygame.key.get_pressed()
    → pressed_axes() 產生 normalized forward=1
    → ControllerLink.send() 編碼 CommandPacket
    → localhost UDP
    → LocalUdpTransport.receive_commands() 解碼與驗證
    → QuadrotorSimulation.accept_command() 過濾舊 sequence
    → FlightStateMachine.integrate_manual() 更新世界座標 target
    → QuadrotorController.step_controller() 計算四槳推力
    → MuJoCo mj_step()
    → TelemetryPacket 經 UDP 回到 Pygame 畫面
```

程式內的中文註解集中解釋「為什麼這樣做」、座標/符號約定與安全邊界；函式名稱與型別則保留英文，方便對照 MuJoCo 與控制理論文件。

## 系統需求與安裝

- Python 3.10+
- Linux、macOS 或 Windows
- MuJoCo Viewer 模式需要可用的 OpenGL/display
- `pygame-ce` distribution；module 名稱仍是 `import pygame`

Linux/macOS：

```bash
cd ~/rm_uav_lab/mujoco_uav
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

Windows PowerShell：

```powershell
cd path\to\rm_uav_lab\mujoco_uav
py -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

若同一環境曾安裝傳統 `pygame`，先執行 `python -m pip uninstall pygame`，避免它與 `pygame-ce` 覆蓋同一個 module。

## 一次啟動兩個視窗

```bash
cd ~/rm_uav_lab/mujoco_uav
source .venv/bin/activate
python launch.py
```

啟動後會看到：

```text
MuJoCo Viewer
    只顯示 3D 模擬並提供相機操作。

Pygame Controller
    唯一接收飛行鍵盤與按鈕輸入的視窗。
```

先點擊 Pygame Controller，使畫面右上方顯示 `CONTROL FOCUS ACTIVE`，再使用 WASD。若顯示 `NO CONTROL FOCUS`，所有連續命令均為零。

macOS 若 passive viewer 要求主執行緒 wrapper：

```bash
python launch.py --sim-python mjpython
```

也可以設定：

```bash
export MUJOCO_SIM_PYTHON=mjpython
python launch.py
```

`launch.py` 在任一子程序結束或收到 Ctrl+C 時會終止另一個子程序，避免留下孤兒 process。

## 分別啟動

終端 1：

```bash
python simulator.py
```

終端 2：

```bash
python pygame_controller.py
```

控制器可以先開；沒有 simulator 時顯示 `DISCONNECTED`，但不會崩潰。Simulator 也可以在沒有控制器時持續運作。

## IPC 設計

只接受 loopback address：

```text
Pygame → simulator commands:  127.0.0.1:14560，約 50 Hz
simulator → Pygame telemetry:  127.0.0.1:14561，約 25 Hz
```

兩個 UDP socket 都不阻塞 simulation/render loop。封包包含 `version`、單調遞增 `sequence` 與 monotonic `timestamp`；malformed JSON、NaN/Inf、未知 action、非 localhost 來源與超出 ±1 的 command 都會被拒絕。舊 sequence 會被丟棄，因此重複 UDP action 不會重複觸發。

命令是 normalized 高層輸入：

```json
{
  "version": 1,
  "sequence": 123,
  "timestamp": 12.345,
  "action": null,
  "forward": 0.0,
  "right": 0.0,
  "up": 0.0,
  "yaw": 0.0
}
```

`action` 只允許 `arm`、`disarm`、`takeoff`、`hover`、`land`、`reset`、`emergency_stop`。IPC 不包含單槳命令。

遙測包含 connection、flight state、位置、速度、roll/pitch/yaw、target、四槳推力與 command age。Pygame 會全部顯示。

## Pygame 按鍵

| 按鍵 | 功能 |
|---|---|
| W / S | 沿目前 yaw 前進 / 後退 |
| A / D | 沿目前 yaw 左移 / 右移 |
| R / F | 上升 / 下降 |
| Q / E | 左偏航 / 右偏航 |
| T | 平滑起飛；會自動 ARM |
| L | 受控降落 |
| H / Space | HOVER/BRAKE，捕捉並保持目前 pose target |
| M | ARM |
| N | DISARM；只允許低速地面狀態 |
| Backspace | RESET；只允許地面或 EMERGENCY_STOP |
| Shift + Escape | EMERGENCY STOP |
| Escape | 送出 neutral command 並關閉 Pygame，不會直接切馬達 |

Pygame 也提供 ARM、DISARM、TAKE OFF、HOVER、LAND、RESET、EMERGENCY STOP 按鈕。按鈕會依目前 state disabled；EMERGENCY STOP 必須在兩秒內點擊兩次確認。

## 焦點與卡鍵保護

每個 Pygame frame 都檢查 `pygame.key.get_focused()`，連續控制使用 `pygame.key.get_pressed()`，不依賴 OS key repeat，也不使用全域 keyboard hook。

發生失焦、最小化、切換到 MuJoCo Viewer 或退出時：

1. 本地 axes 立即清零。
2. 立即發出 neutral UDP command。
3. UI 顯示 `NO CONTROL FOCUS`。
4. 回到 Pygame 後，必須先完全鬆開 W/S/A/D/R/F/Q/E，才會重新顯示 `CONTROL FOCUS ACTIVE`。

這避免按住 W 切換視窗後命令卡住。

## 手動飛行語義

輸入由 simulator 依無人機當前 yaw 轉成 heading-relative 世界座標 target velocity，再以 simulation timestep 積分成位置 target：

```text
forward max speed:    1.0 m/s
vertical max speed:   0.6 m/s
yaw max rate:         60 deg/s
horizontal target:    ±10 m
altitude target:      0.15–5.0 m
yaw target:           [-π, π)
```

放開按鍵後停止積分並保留最後 target，原位置與 SO(3) 姿態控制器會在該處懸停。

## 飛行狀態機

```text
DISARMED
    ↓ ARM / TAKEOFF
ARMED_IDLE
    ↓ TAKEOFF
TAKING_OFF
    ↓ target reached and stable for 0.4 s
HOVERING
    ↔ MANUAL
    ↓ LAND
LANDING
    ↓ ground stable for 0.4 s
    ↓ motor ramp-down
LANDED
    ↓ automatic safe disarm
DISARMED
```

任何 powered state 都可以進入 `EMERGENCY_STOP`。只有在地面、低速且姿態合理時才能 RESET 回到 `DISARMED`。

### 起飛

- 驗證地面高度、線速度、角速度與姿態。
- 自動 ARM，捕捉目前 x/y/yaw。
- 以 3 秒 smoothstep 將高度 target 升到預設 1.2 m。
- profile 峰值上升速度受 duration 限制。
- `|z error| < 0.08 m` 且 `|vz| < 0.12 m/s` 持續 0.4 秒後進入 HOVERING。
- TAKING_OFF 期間忽略普通 manual axes，只接受 LAND 與 EMERGENCY STOP。

### HOVER / BRAKE

清除 manual axes、捕捉目前 x/y/z/yaw 並設成新 target，完全透過 controller 減速；不修改 MuJoCo pose 或 velocity。

### 降落

- 捕捉開始時的 x/y/yaw 並固定。
- smoothstep 峰值下降速度限制為約 0.32 m/s，接近地面自動減速。
- 綜合高度、垂直速度與 roll/pitch，接地條件需連續成立 0.4 秒。
- 接地後用 0.5 秒把現有 motor thrust 線性降到 0。
- 經 LANDED 自動轉成 DISARMED。
- 高度仍足夠時可按 H/Space 中止降落並懸停。

### ARM、DISARM 與 EMERGENCY

- ARM 只允許安全地面狀態。
- 飛行中普通 DISARM 會被 simulator 拒絕。
- EMERGENCY STOP 是唯一立即將馬達歸零的飛行 action；空中觸發會掉落。
- EMERGENCY_STOP 只能在確認落地後 RESET。

## 失聯保護

Simulator 使用本地 monotonic receive time，而不是相信封包 timestamp。

- 有效 command 超過 0.5 秒未收到：axes 歸零、捕捉目前位置與 yaw、進入 HOVERING。
- DISARMED/LANDED 失聯：保持 motor 0。
- LANDING 失聯：繼續已開始的受控降落。
- 最小版本不做自動降落；長時間失聯維持懸停。
- Pygame 關閉不會讓 simulator 崩潰或立即切馬達。

## CLI

Simulator：

```bash
python simulator.py
python simulator.py --headless
python simulator.py --headless --scenario takeoff_hover --duration 8
python simulator.py --headless --scenario takeoff_land --duration 14
python simulator.py --command-port 14560 --telemetry-port 14561
```

Controller：

```bash
python pygame_controller.py
python pygame_controller.py --host 127.0.0.1
python pygame_controller.py --command-port 14560 --telemetry-port 14561
```

Launcher：

```bash
python launch.py
python launch.py --headless
python launch.py --sim-python mjpython
python launch.py --command-port 14560 --telemetry-port 14561
```

舊的 `fly.py` 保留 core smoke/hover 相容性，但它的 viewer 沒有飛行 keyboard mode：

```bash
python fly.py --headless --trajectory hover --duration 6
python fly.py --trajectory hover
```

## 自動化驗收

```bash
python -m pytest tests/test_protocol.py -q
python -m pytest -q
python simulator.py --headless --scenario takeoff_hover --duration 8
python simulator.py --headless --scenario takeoff_land --duration 14
python m0_benchmark.py
python m1_benchmark.py
python m2_benchmark.py
python m3_benchmark.py
```

測試涵蓋：

- command/telemetry encode、decode 與 round-trip
- malformed JSON、NaN/Inf、out-of-range command
- 重複 action sequence 去重
- takeoff → hover 與 takeoff → land → disarm
- heading-relative forward command
- 飛行中 DISARM 拒絕與 EMERGENCY 零推力
- 0.5 秒 command timeout 回到 HOVERING
- Pygame focus loss、最小化與 key-release re-arm
- 真 localhost UDP command/telemetry round-trip
- 原始 mixer、位置、yaw 與 hover regression
- M0 物理 sanity tests：自由落體加速度、懸停推力 m·g/4、推力方向、
  roll/pitch/yaw mixer 產生的力矩符號
- M0 benchmark：起飛到 1.5 m 並懸停 5 秒的完整驗收門檻
- M2 任務狀態機單元測試：phase 轉移、連續穩定計時、timeout、
  measured-state 決策與 ground-truth 評分結構性分離、heading 投影
- M2 benchmark：起飛 → 前進 5 m → 煞車 → 懸停 → 降落的 12 case 驗收
- M3：IMU/flow/ToF 感測器模型、complementary 估計器、結構性 truth 隔離、
  11 case 定位任務驗收 + 長斷訊 loud-failure 測試

## M3 GPS-denied 定位：估計導航狀態

M0 驗證理想 plant/controller；M1 加入非理想物理與「truth+白噪聲」量測；
M2 用 measured state 完成自主任務；M3 則**移除直接位置/速度/姿態真值**，
改成一條最小可理解的 GPS-denied 定位鏈：

```text
MuJoCo truth ──→ IMU (gyro + specific force, 250 Hz)
             ──→ optical-flow-like body 速度 (50 Hz)
             ──→ ToF 高度 (50 Hz)
                     ↓
             LocalizationEstimator（IMU 預測 + flow/ToF 修正）
                     ↓
             EstimatedState → mission + flight controller

MuJoCo truth ──→ 評分 / 安全檢查 / 落地接觸判定（ONLY）
```

**M3 的感測器數值是工程測試值，不是 MicoAir MTF-02P 的 datasheet
性能。** 之後的 milestone 才會用量測過的硬體數據替換。

### 感測器模型（m3_sensors.py）

- **IMU**：ω_meas = ω_true_body + bias + 白噪聲（σ=0.002 rad/s）；
  specific force f = R_world→body · (a_world − g_world) + bias +
  白噪聲（σ=0.05 m/s²）。靜止水平時 accelerometer 讀到 body +Z 的
  +9.81 m/s²（量測的是支撐力而非重力本身），此符號慣例由測試鎖定。
  a_world 取自 free joint 的 qacc[0:3]（mj_objectAcceleration 在此模型
  不可用：沒有 force sensor 時 cacc 不會被計算，會靜默回傳零）。
- **Optical-flow-like**：輸出 body frame 水平速度 scale·v_body_xy +
  bias + 白噪聲（σ=0.03 m/s），不含任何絕對位置；**不是**影像級
  optical flow（無相機渲染、無 feature tracking）。支援 scale error、
  bias、dropout、固定 latency（timestamped queue，不用 sleep）。
- **ToF**：body 原點對地面（z=0 plane）的高度 + bias + 白噪聲
  （σ=0.01 m），有效範圍 0.05–8.0 m，範圍外 / dropout → invalid。
  注意落地後高度 < 0.05 m 時 ToF 合理失明，因此 estimator health 只在
  TAKEOFF..HOVER 階段作為 benchmark gate。

所有感測器以 physics step counter 決定取樣（IMU 每 2 步、flow/ToF
每 10 步，dt=0.002 s），seed 固定時 bit-identical 可重現。

### 估計器（m3_estimator.py）

刻意不用 Kalman filter；是可讀的 predict/correct：

```text
q ← q ⊗ exp(ω_meas·dt/2)                      # 姿態傳播（quaternion 正規化）
a_nav = R(q_est)·f_body + g_world             # 比力轉導航系
v += a_nav·dt ; p += v·dt + ½·a_nav·dt²       # 慣性預測
v_xy ← (1−α_f)·v_xy + α_f·(R(q_est)·v_flow_body)_xy   # α_f = 0.35
z   ← (1−α_t)·z + α_t·h_tof                            # α_t = 0.5
```

Estimator 有自己的 LOCAL navigation frame：啟動時 x=y=0、yaw=0
（local +X = 起飛時機頭方向），即使 MuJoCo world yaw 是 90°。沒有
磁力計 → gyro z bias 會造成真實 yaw drift；沒有水平位置感測器 →
x/y 只能靠積分估計速度得到，flow scale/bias 因此會累積成**真實的
位置誤差**。若任何時刻 x/y 與 truth bit-identical，代表 truth 洩漏，
M3 失敗。

Health：IMU 失效 > 0.05 s、flow > 0.5 s、ToF > 0.5 s 或內部狀態
NaN/Inf → healthy=False；benchmark 中止為 ESTIMATOR_UNHEALTHY，
**絕不 fallback 到 truth**。短斷訊（0.25 s）由慣性預測橋接。

### M2 與 M3 的差異

M2 的「measured state」本質仍是 truth + 白噪聲——每個通道都有真值
為中心。M3 的 EstimatedState 由感測器模型動態生成，**會漂移**：
estimator 以為的 5 m 與物理實際的 5 m 之差成為核心指標
（例：case2 flow scale +2% → est 5.099 m vs truth 5.011 m）。

### M3 benchmark

```bash
python m3_benchmark.py          # 11 個 case，輸出 m3_localization_report.json
```

Case 0 理想感測器（證明估計器架構本身正確）；case 1 nominal 噪聲；
case 2/3 flow scale/bias；case 4/5 flow/ToF 短斷訊 0.25 s；case 6
gyro z bias 0.1°/s；case 7 flow+ToF latency 20 ms；case 8 全部疊加
（含 M1 combined plant，seed 27）；case 9/10 初始 yaw=90°。

每個 case 的通過門檻（刻意容忍漂移，不要求完美 5.00 m）：
mission DONE、LAND 前 truth 前進位移 4.5–5.5 m、|truth 橫向|
< 0.30 m、est-vs-truth 高度 RMS < 0.15 m、最大傾角 < 15°、飽和
< 20%、無 NaN/Inf、無穿地、estimator 在主動飛行階段全程 healthy、
成功物理降落。

## M4-A 微型機 plant：質量 / 幾何 / 慣量 / 推力餘裕縮放

M0 驗證理想 plant/controller；M1 加 plant realism；M2 做有限任務；
M3 用估計導航狀態飛全程。M4-A 則把 **plant 本身**換成 RM27 方向的
微型機：**180 g、120 mm 對角 wheelbase、X 配置**
（`models/quadrotor_m4a.xml` + `m4_params.py`），而 M2 mission、
M3 estimator、感測器速率（IMU 250 / Flow 50 / ToF 50 Hz）、truth
隔離、高層速度介面全部不變——這是 controlled experiment：性能差異
只能來自 plant。

> **警告：1103 9500 KV + 2"/2.5" 槳 + 2S/3S 的推力 / 電流 / 轉速
> 在 M4-A 尚未經實測驗證。** 最大推力來自假設的 thrust-to-weight
> ratio（nominal 2.5，sensitivity 2.0 / 3.0），屬
> ENGINEERING_ESTIMATE。KV × 電壓 **不是** 實際帶槳轉速。

### 參數 provenance

`m4_params.provenance_table()` 為每個重要參數標註 MEASURED /
DATASHEET / CAD_DERIVED / CALCULATED / ENGINEERING_ESTIMATE /
USER_PROVIDED / DESIGN_CHOICE 與信心等級（DATASHEET 僅用於 repo 內
真的有 datasheet/規格來源的參數）；完整表隨 benchmark 寫入
`m4_micro_uav_report.json`。核心數字：

| 參數 | 值 | provenance | 說明 |
|---|---|---|---|
| mass | 0.180 kg | ENGINEERING_ESTIMATE | RM27 目標 150–200 g，AUW 未秤 |
| wheelbase | 0.120 m | ENGINEERING_ESTIMATE | 100–140 mm 設計範圍中值 |
| hover thrust/motor | 0.44145 N | CALCULATED | m·g/4 |
| max thrust/motor | 1.103625 N | ENGINEERING_ESTIMATE | T/W=2.5 假設 × m·g/4，**非馬達實測** |
| inertia | (8.50, 10.97, 17.93)e-5 kg·m² | CALCULATED | 元件法（見下），輸入皆估計 |
| motor KV | 9500 RPM/V | USER_PROVIDED | 使用者選定 1103，repo 內無 datasheet 佐證；KV 不決定推力 |
| prop / battery | 2.0–2.5 in / 2S–3S | DESIGN_CHOICE | 候選方向，M4-A 僅為 metadata |

### 慣量模型（非線性縮放舊值）

中央 60×40×25 mm 方塊（0.148 kg = 總重 − 4×8 g 馬達）+ 四個馬達
point mass 於 (±a, ±a, 0)，a = wheelbase/(2√2) = 0.04243 m。方塊用
I = m(d₁²+d₂²)/12，馬達用 parallel-axis point mass。縮放律
I ∝ m·L² 由測試鎖定（geometrically_scaled）。

### 控制器：唯一必要的更動（有失敗證據）

第一次 M4-A hover benchmark 用原增益**直接炸掉**：80 ms 內姿態發散、
衝過 10 m 安全上限。根因不是物理或座標（M4 物理測試全過），而是
離散穩定性：M0 attitude_kd=0.18 作用在 Ixx=8.5e-5 kg·m²、500 Hz 下，
顯式阻尼 pole 1 − kd·dt/I = **−3.24**（|pole|>1，每步放大 3.24 倍）。

修正是 **analytical inertia-based gain scaling**（model-derived gain
rescaling）：`micro_controller_parameters()` 把 attitude_kp /
attitude_kd / max_torque 依慣量比 I_micro/I_M0 縮放，閉迴路頻寬與阻尼
與 M0 **數學上完全相同**；位置迴路增益不變（thrust=m·a 對質量不變）。
舊預設增益不動，M0–M3 regression 全部維持 PASS。

### 感測器速率壓力實驗（one-factor-at-a-time）

微型機動態更快，原本可能暴露取樣率不足。實驗：IMU 125/250/500 Hz、
Flow 25/50/100 Hz（ToF 固定 50 Hz）。結果四個變體全部 DONE/PASS，
posRMS 皆 ~0.02 m —— **250/50/50 Hz 對 180 g plant 仍然足夠**，
M3 預設值不變。

### 結果與報告

11 個 A case（nominal / noise / combined / 質量 150–200 g /
wheelbase 100–140 mm / T/W 2.0–3.0 / yaw 90°）全部 DONE + PASS。
因為閉迴路動態被慣量比縮放保留，任務指標與 M3 通用機幾乎一致
（A1: fwd 5.09/5.09 m, posRMS 0.014 m, tilt 3.9°）；差異主要出現在
**馬達指令**（M3 case1 峰值 2.68 N → M4-A A1 0.48 N ≈ 質量比）。
Combined case 的 flow scale 1.01 造成預期的 est長/truth短 分歧
（A2: est 5.09 m vs truth 4.94 m）。機讀報告：
`m4_micro_uav_report.json`；hover benchmark：`m4a_hover_benchmark.py`
（起飛 4.47 s、overshoot 0.143 m、hover RMS 0.013 m、落地成功）。

## M1 非理想 robustness

M0 是理想剛體/控制 baseline；M1 問的是：「當 plant 與量測不再完美時，
同一套控制器與任務架構是否仍然穩定？」M1 只加入溫和、可理解的非理想
因素，**不是**最終 RM 微型無人機的驗證物理模型，而是 ideal simulation
與日後 system identification / Sim-to-Real 之間的 robustness bridge。

### 非理想因素

- **馬達一階延遲**（`models/quadrotor_m1.xml`）：`<motor>` shortcut 換成
  等價 `<general dyntype="filterexact" dynprm="0.040 ...">`，實際推力遵循
  Ḟ=(u−F)/τ，τ=0.040 s（測試另覆蓋 0.020/0.080）。名稱、site、gear、
  ctrlrange、推力單位（牛頓）與 M0 完全相同。
- **空氣阻力**（M1 XML 內建，`density=1.2`、`viscosity=1.8e-5`）：
  MuJoCo 內建流體交互以等效橢球近似對機體 geom 施加與速度反向的
  阻力/黏性項；不含槳葉尾流、地面效應或升力。`PlantConfig(drag=...)`
  可在任一模型上開關。
- **質量/慣量不確定性**（`PlantConfig(mass_scale=, inertia_scale=)`）：
  ±10%。物理模型改變，但 controller 永遠使用 nominal 1.0 kg 與
  (0.018, 0.018, 0.030)——這正是「不確定性」的意義。
- **馬達效率不一致**（`PlantConfig(motor_effectiveness=)`，順序為
  FL/FR/RR/RL）：實際推力 = effectiveness × 指令推力，以 actuator
  fixed gain 實現。M1 nominal 不對稱 case 依馬達名稱明確定義為
  **FL=0.97、FR=1.00、RR=0.99、RL=1.02**（tuple `(0.97, 1.00, 0.99, 1.02)`
  對應 actuator 順序 FL/FR/RR/RL），會在起飛時產生可量測的 pitch/yaw
  擾動，由控制器回授吸收。tuple 順序由測試依 actuator 名稱鎖定，
  不可只依賴文字描述。
- **量測雜訊**（`NoiseConfig` + seeded `MeasurementModel`）：位置
  σ=0.010 m、速度 σ=0.020 m/s、姿態以小旋轉向量擾動 σ=0.2°
  （保證仍是合法旋轉，不直接對 quaternion 分量加噪）、角速度
  σ=0.01 rad/s。雜訊只進入控制器看到的 measured state；ground truth
  保留給狀態機、安全檢查與 metrics。相同 seed 完全可重現。
- **高度積分項**（M1 對控制器的唯一修改）：純 PD 在 ±10% 質量下有
  恆定誤差 e=Δm·g/kp_z=0.196 m（實測），導致永遠無法進入 HOVERING。
  因此在垂直通道加入小積分（ki_z=1.5，積分貢獻限幅 ±2.0 m/s²），
  消除恆定力偏差；M0 全部測試與 benchmark 維持通過。

### M1 benchmark

```bash
python m1_benchmark.py          # 10 個 case，輸出 m1_robustness_report.json
```

每個 case 固定一組 plant 參數跑完整 production path（action → 狀態機 →
控制器 → mixer → actuator → MuJoCo → measurement → 控制器），起飛到
1.5 m 後懸停 5 秒。Case 0 為 M0-like 參照；case 9 為全部非理想因素
疊加（seed 27，完全可重現）。

每個 case 的通過門檻：final altitude error < 0.15 m、hover RMS < 0.10 m、
水平漂移 < 0.15 m、起飛暫態後 roll/pitch < 7°、懸停水平速度 < 0.30 m/s、
懸停滿 5 s、無 NaN/Inf、無穿地、無持續 actuator 飽和（< 20%）。

## M2 自主任務：起飛 → 前進 5 m → 煞車 → 懸停 → 降落

M0 驗證理想 plant/controller，M1 驗證非理想 robustness；M2 問的是：
「同一條 production path 能否完成一段有限距離的自主任務？」任務圖：

```text
READY → TAKEOFF → SETTLE → FORWARD → BRAKE → HOVER → LAND → DONE
                          （任何階段 timeout / 異常 → FAILED，附明確原因）
```

### Measured-state 決策 vs ground-truth 評分

M2 的核心規則：**mission 狀態機只看 measured state**（含 M1 量測雜訊），
ground truth 只用於 benchmark 評分、安全檢查與 NaN/runaway 偵測。
起飛完成、前進 5 m 到達、煞車完成、懸停穩定都由量測值判定——
MissionStateMachine 的 API 結構上不接受 truth state。因此報表刻意分開
記錄「飛機以為的位置」與「實際位置」（例如 combined case 在 LAND 前：
measured 5.122 m vs truth 5.113 m）。這個差距是日後 estimator 工作的
核心指標，不可合併成單一「距離」。

### Heading-relative 任務座標

「前進 5 m」定義為**任務起始時機頭方向的 5 m**，不是世界 +X。任務開始
時由 measured state 記錄 p0_xy 與 yaw0，定義：

```text
h = [cos yaw0, sin yaw0]   # 起始航向單位向量
l = [-sin yaw0, cos yaw0]  # 橫向單位向量
forward_progress = dot(p_xy - p0_xy, h)
lateral_error    = dot(p_xy - p0_xy, l)
```

yaw0 = 0° 時世界軌跡沿 +X；yaw0 = 90°（case10/11）時沿 +Y，
兩者的 projected forward displacement 都 ≈ 5 m。

### 速度命令剖面

FORWARD 依 measured 剩餘距離產生距離感知的速度命令：
remaining > slowdown_distance（2.0 m）時巡航 0.8 m/s；進入減速段後
v = max_speed × remaining / slowdown_distance，並以 minimum_approach_speed
（0.05 m/s）做下限。到達門檻後命令歸零進入 BRAKE，要求 measured
水平速度 < 0.20 m/s 連續 0.4 s 才算煞車完成，之後懸停 1.0 s 再降落。

注意：本專案的高層介面是 position-mode（速度命令被積分成位置 target
再由控制器追蹤），target 會「領先」機身約 v × τ_cascade。因此減速段
必須在機身到達前足夠早開始（slowdown_distance 需大於巡航領先量
≈ 1.1 m，實測 1.0 m 會衝到 5.46 m），minimum_approach_speed 也不能
太大（0.15 m/s 時 target 凍結後機身仍多走約 0.44 m，實測超出
5.3 m 門檻）。這兩個參數是任務層修正，控制器本身維持 M1 驗證值。

### M2 benchmark 與通過門檻

```bash
python m2_benchmark.py          # 12 個 case，輸出 m2_mission_report.json
```

Case 0 為 nominal 參照；case 1–9 逐一疊加 M1 非理想因素（馬達延遲、
質量/慣量 ±10%、drag、量測雜訊、馬達 mismatch，case 9 為全部疊加、
seed 27）；case 10/11 驗證 yaw0 = 90° 的 heading 不變性。每個 case
的通過門檻：mission DONE、truth 起飛高度誤差 < 0.15 m、LAND 前
truth 前進位移 4.7–5.3 m、|truth 橫向位移| < 0.20 m、進入 HOVER 時
truth 水平速度 < 0.30 m/s、最大傾角 < 10°（robust case < 15°）、
飽和比例 < 20%、無 NaN/Inf、無穿地。

M2 仍然不是最終 RM 微型無人機的物理模型；它是 estimator / 定位層
（M3）之前的最後一個 ground-truth-scored 任務里程碑。

## M0 驗證

M0 的目的是在沒有任何學習演算法之前，先證明「模型物理 + 控制器 +
狀態機」是可信的 baseline。驗證分兩層：

1. `tests/test_m0_physics.py`：直接對 MuJoCo plant 施力，驗證自由落體
   加速度等於 −g、總推力 m·g（每槳 m·g/4 = 2.4525 N）時加速度為零、
   增大總推力向上加速、以及 roll/pitch/yaw 差動推力產生的角加速度
   符號與大小符合 allocation matrix 的解析預測（τ = I·α）。
2. `python m0_benchmark.py`：只透過 TAKEOFF action 與控制器閉迴路到達
   1.5 m，狀態機宣告 HOVERING 後量測 5 秒懸停，輸出報告與
   `m0_hover_report.json`。驗收門檻：final altitude error ≤ 0.10 m、
   hover RMS error ≤ 0.10 m、水平漂移 ≤ 0.10 m、roll/pitch ≤ ±5°、
   無墜毀、無 NaN/Inf、懸停滿 5 s。pytest 中的
   `tests/test_m0_benchmark.py` 會重跑同一個 benchmark 作為 milestone gate。

## 高層動作介面（vx, vy, vz, yaw_rate）

IPC 上的四個 normalized 軸 `forward / right / up / yaw` 刻意設計成與
「期望速度」一一對應，而不是馬達命令。換算（ψ 為當前 yaw，世界座標
+Z 向上）：

```text
vx       = 1.0 m/s  * (forward * cos ψ + right * sin ψ)
vy       = 1.0 m/s  * (forward * sin ψ - right * cos ψ)
vz       = 0.6 m/s  * up
yaw_rate = 60 deg/s * yaw          # 正號為逆時針（向左轉）
```

反向換算（例如未來的 trajectory controller 或 policy 輸出速度命令）：

```text
forward = (vx * cos ψ + vy * sin ψ) / 1.0
right   = (vx * sin ψ - vy * cos ψ) / 1.0
up      = vz / 0.6
yaw     = yaw_rate / (60 deg/s)
```

語義上這是 position-mode：simulator 把 vx/vy/vz/yaw_rate 以 simulation
timestep 積分成位置/航向 target，再交給位置 + SO(3) 姿態控制器追蹤；
放開輸入（全零）時 target 停在原地懸停。未來的軌跡控制器、神經網路
policy 或 companion computer 只需要輸出 (vx, vy, vz, yaw_rate)，經過
上表正規化後即可沿用同一條 IPC 與控制管線，永遠不需要直接碰四個馬達。

## 目前限制

- M4-A 微型機的馬達/槳/電池性能是**假設值**（T/W ratio），不是
  1103 9500 KV + 2"/2.5" + 2S/3S 的實測數據；真實推力包線、電流、
  馬達響應、COM offset 屬 M4-B（NEEDS_MEASUREMENT）。
- M3 已有最小定位估計器（IMU 積分 + flow/ToF 修正），但不是 EKF：
  沒有 bias 估計、沒有磁力計/航向觀測、感測器模型不含真實 optical
  flow 的紋理/光照依賴或 ToF 的表面材質依賴。
- 馬達以一階 lag 近似，沒有真實槳葉氣動、ground effect 或電池電壓
  衰退；drag 為 MuJoCo 等效橢球流體近似。
- UDP 僅供同一台機器的簡單控制，不是 MAVLink，也沒有加密或遠端網路支援。
- 失聯策略是懸停，不會自動返航或自動降落。
- 這是易懂、可擴充的 simulation baseline，不代表實機飛行安全性。
- 不依賴 ROS、ArduPilot、PX4 或其他大型飛控框架。
