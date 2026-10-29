# RM27 全倉庫研發進度與研究背景

研發快照日期：2026-10-29（Asia/Shanghai）；下文版本／dirty 狀態是文件發布前的研發工作區，不是這個文件 PR 的乾淨 checkout。用途：可單獨上傳給 ChatGPT 的跨倉庫背景與進度快照；也是本專案固定的總進度入口。

本報告整理主倉庫、六個獨立 components、Tracking、相關歷史基線、正式報告及本地活動計畫。它不是逐行源碼審計，也不是重新執行所有歷史實驗。核心結論已寫入正文；讀者無須取得本地計畫才能理解進度。文末路徑供追溯，不代表附件包含那些檔案。

## 1. 先讀這一頁

我們要做的是單架微型無人機：**從機庫起飛 → 接近基地 → 指定底部接觸件接觸裝甲 → 脫離與恢復 → 返回指定機庫位置並降落停槳**。先完成單機可靠閉環，再擴展多機；飛鏢攔截是另一條研究支線。

目前不是一套已完成真機驗證的系統，而是多條研發線各自取得有限證據：

| 研發線 | 已有能力／結果 | 還不能宣稱 |
|---|---|---|
| 新場地 + Ver2 仿真 | 零推力起步、原生機庫起飛、完整場地繞行，到裝甲前保持前飛的交接狀態；三個名義速度通過 | 在這個新場地已完成撞擊、恢復、返航與落庫 |
| 局部撞擊任務 | 局部試驗台上連續起飛、接觸、恢復、返回、降落與停轉，軟／硬兩組共 140/140 | 競賽原生機庫與完整場地同樣通過，或真機可靠性 |
| RL | 有自主 CTBR 學習、到達環境、PPO/SAC 與多輪對照研究；部分舊任務配置成功 | 新 Ver2 真實接觸任務已訓練完成；所有 seed 都穩健 |
| PX4 | 真實 PX4 v1.17 固件 SITL；舊代理機型完整圖像引導任務；Ver2 短程起降任務各有成功例 | Ver2 在新場地完整任務、真實 VIO 或實機部署已完成 |
| VIO | OpenVINS 路線、資料鏈路與時間診斷、可重現 PC 候選 | 相機–IMU 時序、標定和完整定位驗收已過 |
| SLAM | 獨立單目起步包、主機資料集路線與板端前端測量 | 板端完整 SLAM 實時、穩定尺度與飛行定位 |
| Tracking | 綠燈／裝甲管線、180 fps 全視野採集與估計輸出測量 | 180 Hz 有效目標觀測、帶真值準確率、可安全控制 |
| MJCF | 飛機／機庫／場地資產、碰撞重建、可動基地 | 可見幾何全部具有經驗證碰撞，或材料參數已實測 |

**最重要的閱讀原則：不同飛機、場地、控制器、導航輸入、驗收門檻的成功不能相加成完整能力。** 仿真真值不是相機定位；錄製輸入精確回放不是另一次獨立閉環飛行；動畫不是物理驗證。

## 2. 證據標記與本次整理範圍

- **本次核對**：讀取現有檔案、Git 狀態或結果 JSON；沒有重跑該實驗。
- **既有紀錄**：原報告或活動計畫記載的結果，本次整理引用；不是本次重新驗證。
- **工程假設**：為建模或研究指定的值，不是實測。
- **未完成／未知**：沒有相應證據，或原驗收仍未通過。

本次只做文件整理。未訓練、未執行飛行、未刷固件、未寫真機參數，也未重跑各倉庫測試。下文測試數量均屬各自歷史版本，不能合計為目前工作區的測試通過數。

## 3. 倉庫架構與在哪裡工作

工作區根目錄為 `/home/shiuhou/Projects/rm27_drones`。以下相對路徑都以此為基準。

| 本地位置 | GitHub repository | 職責 |
|---|---|---|
| 根目錄 | `shiuhou/rm27_drones` | 跨組件契約、版本釘選、需求、整合與歷史研究 |
| `components/rm27_RL/` | `shiuhou/rm27_RL` | 訓練環境封裝、reward、policy、訓練／評估、RL Docker |
| `components/rm_uav_mujoco/` | `shiuhou/rm27_sim` | 共用物理、場景組合、完整任務、viewer、回放、仿真整合 |
| `components/rm27_drone_px4/` | `shiuhou/rm27_drone_px4` | PX4 固件介面、SITL、估計器與飛行任務整合 |
| `components/rm27_vio/` | `shiuhou/rm27_vio` | OpenVINS、相機／IMU、時序、定位驗證 |
| `components/rm27_slam/` | `shiuhou/rm27_slam` | 單目 SLAM 獨立研究 |
| `components/rm_mjcf/` | `CosmosMount/rm_mjcf` | 可重用 MJCF 與 runtime meshes／場地資產 |
| `rm27/perception/vision/maixcam2_dart_vision/` | `blademaster679/maixcam2_dart_vision` | Tracking 上游綠燈／裝甲視覺管線 |

`components/` 是獨立版本庫的引入位置。`rm27/` 是主倉庫自己的整合／研究命名空間，並保留歷史 vendor、baseline 與兼容入口。兩者不是兩套相同的主倉庫。

2026-10-21 起，場景組合、任務執行和 viewer 的實作歸 `components/rm_uav_mujoco/rm27_sim/`。根目錄部分 `rm27` 模組是兼容轉接、部分 config 是 symlink；不要再在根目錄複製一套新的仿真實作。既有歷史接觸研究仍保有原契約與重現用途。

共用物理套件在 `components/rm_uav_mujoco/packages/physics/`，import 名為 `rm27_contact_physics`。它不依賴 RL、Gymnasium 或 Torch；RL 消費這個套件。RL 專用 Docker 留在 RL；本地已有 `rm27-contact-rl:i4-arrival-cpu` 和拆分驗證使用的 `rm27-contact-rl:sim-separated-20261021` 紀錄，鏡像名稱本身不保證另一部機器可直接取得。

其他主倉庫直接 submodules 包括 ArduPilot、ardupilot_gazebo、舊 RMUC2026 場地，以及 legacy MuJoCo／凍結 ArduPilot baseline。`.gitmodules` 將後兩者指向 `rm27_drones` 的 `components/mujoco-uav`、`components/mujoco-uav-ardupilot` 分支；它們不是目前六條獨立 components 的替代品。保留凍結 G1，不把新場地／新控制器結果寫回其原有聲明。

`baselines/` 是研發線交接與證據索引，不是第二份源碼。部分 handoff 內容較舊，需與實作倉庫最新需求／計畫對照。

## 4. 主倉庫任務與硬體基線

### 4.1 機型與場地

目前可重用機型是 **Ver2，使用者報告含電池 150 g**；CAD 約 114 × 114 × 45 mm，選定 5-series frames，另加 30 × 65 × 5 mm 核心板包絡。舊研究常用 **180 g、120 × 120 mm proxy**；PX4 的舊 150 g proxy 也不等同 Ver2。

目前開發場地由 `configs/field.json` 釘選 `components/rm_mjcf/rmuc2027_moving_base_mjcf/model.xml`，資產 commit `d9439a97cee80baba9d13a1de9882f2d8095fca5`，預設滑台居中。新場景用 `configs/scene_v2.json`；`configs/scene.json` 留作歷史 180 g／RMUC2026 顯式重現。

機庫原始來源是 `data/references/cad_raw/dock.STEP`。使用者已退役舊整場 `RMUC 2025复活赛&全国赛_asm.stp`。`base2.STEP` 為獨立基地組件，其目前匯出僅視覺審核，不自動具有碰撞或正式目標語義。

### 4.2 已知與未知

硬體文件列出 1103 9600KV、2 吋槳、MicoAir 743v2 AIO 35A、MTF02P、M10G5883（僅羅盤用途）、M3C + OS04A10。這是記載的配置，不是本次拆機確認。

質心、慣量、推力曲線、40 ms 馬達時常、單馬達 1.103625 N 上限、0.018 m 偏航係數，以及剛性接觸材料，仍是工程估計。不能把它們當成實機校準值。

較後的仿真活動計畫已記載 `1018.param` 與 `2026_10_18_px4.ulg` 分析：BMI088 加速度 raw 約 1610 Hz／publication 約 805 Hz、gyro 約 1997／666 Hz，hover 約 0.51、2S 電壓約 7.1–7.9 V，並觀察到 clipping。**本次未重新解析原始檔**；檔案對應哪一架、完整載重與飛行環境仍須核對。早期硬體文件的「尚無飛行資料」不能再當成全專案最新狀態。

### 4.3 驗收層級

根 `docs/MILESTONES.md` 的 S0 已有歷史 ArduPilot 10/10 名義短程起降；S1 partial，S2–S5 尚未因局部研究而自動通過，S6 多機延後。

根級恢復門檻與局部 impact study 門檻不同，例如根門檻含角速率 ≤30°/s、速度模長 ≤0.3 m/s、保持 1 s；局部研究可採 <1 rad/s、垂直速度限制與保持 0.3 s。不能把同名 S 編號或 recovery PASS 混用。

「>10 m/s」的歷史動機來自 RMUC2026 42 mm 彈丸規則，**不是已確認的 RM2027 無人機撞擊速度要求**；等動量換算 3 m/s 也不是等能量或正式判傷等價。本報告未重新查核最新賽規。

## 5. Sim：目前最前沿的進度

### 5.1 新完整場地：已到移動交接，尚未撞擊

2026-10-28 活動計畫記錄：Ver2 從新場地負 XY 原生機庫位置出發，升至 2.3 m、外繞、下降，到達裝甲前的交接位置，持續前飛。這是 **headless、MuJoCo truth feedback、工程控制器**，不是 RL、PX4 固件或 VIO 飛行。

本次直接讀取 `full_field_approach_ground_v1/summary.json` 與 `full_field_approach_ground_repeat/summary.json`，兩者 `passed: true`，各三個 flight 均 true，內容 SHA-256 相同。MuJoCo 3.17.0、Python 3.12.3、NumPy 2.5.4；記錄的是 HEAD 加未提交工作區，不是純 commit 重現。

| 名義速度 | 全段時間 | 最後 0.5 s 最大位置誤差 | 最後 0.5 s 最大速度誤差 | 全程最大傾角 |
|---|---:|---:|---:|---:|
| 0.5 m/s | 33.60 s | 1.030 mm | 0.000245 m/s | 3.244° |
| 1.0 m/s | 19.05 s | 4.679 mm | 0.002443 m/s | 14.587° |
| 1.5 m/s | 14.20 s | 23.982 mm | 0.024979 m/s | 22.978° |

交接門檻是位置 ≤0.10 m、速度誤差 ≤0.10 m/s、傾角 ≤10°、角速率 ≤0.5 rad/s；只用於指定交接窗口，全程傾角有獨立 30° 限制。三次無禁止接觸或飽和，保存狀態後接續 0.1 s 的最大差為零。

修正了先前「起步已預載懸停推力」的問題：現在 act/ctrl 全零，先無動力沉降 0.5 s，確認支承、速度、角速率及傾角，再啟動馬達。移除支承的負對照在起飛前拒絕為 `GROUND_NOT_READY`（既有計畫紀錄）。飛行中沒有 reset。

但支承仍靠 **CAD 未包含的工程底球**：半徑 9 mm、中心 z = −44.5 mm，最低點比腳低 31 mm；不是四腳穩定停放驗證。交接世界位置約 `(9.9561, 0, 1.579)` m，距裝甲約 X 方向 1.5 m、上方 0.5 m；**尚未觸板**。「移動交接」描述飛機仍前飛，也不能單憑場地支援滑台就宣稱完成移動目標追擊。

這三個確定性案例及精確重複不是擾動成功率，也不是經認證的速度範圍。碰撞檢查每 1 ms 取樣，不是連續 swept-volume 證明。

本批記錄的回歸是舊環境 193 passed / 5 skipped、新環境場地與 approach 18 passed，另 smoke 12；本次沒有重跑。

### 5.2 局部撞擊、恢復與完整局部任務

獨立 impact study 曾發現區間取樣錯誤：先 `mj_forward` 再取接觸／加速度會讀到重算的端點解，出現撞前約 39 g 假象。修正為在 `mj_step` 後、forward 前讀取實際積分區間；因此舊數字不能直接和新數字混合。

既有修正研究重建 27,600 次接觸與 33,600 次動力敏感度實驗：

- 廣分布 soft 3910/4800、hard 3964/4800；I4 子集 soft 2367/2400、hard 2364/2400。分布不同，不能挑較高者代表所有工況。
- 名義 proxy 與「近似 PX4」內環差距很大；後者甚至無接觸自由飛行也失敗，且推力映射造成實際推重比不同。這不是 PX4 真固件性能比較，也未隔離單一原因。
- 新 seed 的 proxy soft 543/600、hard 549/600；近似 PX4 為 10/600、8/600。水平姿態 3 m/s 是較差單元。
- 理想輸入下 filtered 1 g 接觸檢測 soft 600/600、無誤報、最大延遲 3.5 ms；raw 1 g 有 15/600 誤報。仍不是實測 IMU 部署結果。

`mission_v2` 在局部有中央凹槽的試驗台上，連續起飛→接觸→恢復→返回→降落→停轉：soft/hard 各 20/20 名義重啟 + 50/50 物理擾動，共 140/140。每組 50/50 的 Wilson 95% 下界約 92.86%。軟硬使用相同 50 個 seed，不能算 100 個獨立物理樣本。初始停放姿態固定。

原 `mission_v1` 擾動僅各 22/50，失敗仍保留。改進沒有把舊失敗改成通過。局部凹槽平台是容納突出底球的工程假設；局部完整任務不等於新完整場地任務。

### 5.3 下一個整合缺口

先保存 1 m/s 交接的完整物理／馬達／控制器狀態，接入完整場地已登記的裝甲接觸面，完成末端對準與指定首次接觸；不能用局部 plant reset 偷換初態。然後接恢復、返航及原生機庫落位，再做擾動批次。真實估計與固件路徑需各自驗收。這是依據現有計畫的順序，**本報告不新增訓練或硬體操作授權**。

### 5.4 既有通用仿真與獨立工作區

舊 M0–M3 baseline 保有 Pygame 高層命令、UDP、起降與失聯懸停、非理想條件、簡單 IMU + flow/ToF 估計及任務 benchmark。M3 不是完整 EKF，沒有完整 bias／航向／真實光照材質模型；這些局部里程碑也不等於主專案 S0–S6。

仿真拆分紀錄：物理抽離前後 RL 26 項檢查通過、原物理來源 hash 保持；後續獨立工作區 root 128、sim 136 tests 通過，獨立 ArduPilot micro-flight instance 67 headless SITL 通過。這證明拆分時的兼容與獨立執行，不是新 PX4/VIO 任務認證。外部資產透過本地 `.local/assets.json` 類 manifest 接入，不會因 Git clone 自動獲得原始 CAD／錄影／二進位。

## 6. RL：自主末端控制研究

工作位置 `components/rm27_RL/`。當前方向是依狀態自主學習 collective thrust + body rates（CTBR），**不要求模仿 teacher action**。早期 baselines 或 imitation 研究不是現在 RL 的強制架構。

舊 I4 是「無接觸到達」，同一時刻滿足：板前 gap 0.20±0.04 m、法向速度 1.5–3 m/s、切向位置 ≤0.04 m、切向速度 ≤0.5 m/s、完整姿態誤差 ≤15°、body rate ≤3 rad/s。到達不等於真實接觸；更早嚴格 3 m/s 結果不能事後用新容差改判。

目前 SPEC §10 已改成原生 2027 基地 + Ver2：先連續穩定進場，再由 RL 接管末段，直到**底部中心首次有效物理接觸**。首階段不含恢復、返航或降落。新 task/reset schema 尚未完成實作與訓練，也沒有因這份整理而開啟新預算。

歷史實驗的重要結論（既有活動計畫）：

| 實驗 | 結果／啟示 |
|---|---|
| G2 informed reset + PPO + DR | 三 seed 名義 9/9、8/9、9/9，各自 heldout 50/50；較大噪聲仍有 seed 差異 |
| X1／X6／X7 | 部分無 curriculum 也可成功；去精度 reward 失敗，無 DR 較脆弱，不能把成效全歸單一技巧 |
| X5／X9 | history 或 SAC 在部分條件改善，但另一 seed 的質量／組合偏差仍失敗；沒有通用勝者 |
| X15 | 拆分後評估 5/9、0/9、3/9；原工程錯誤標記保留 |
| X16／X17 | 分別 9/9、9/9、1/9 和 0/9、9/9、0/9，跨 seed 不穩 |
| X18 sparse／tolerance curriculum | 0/9、7/9、9/9，整批 FAIL；未滿至少兩 seed ≥8/9 及 yaw 條件，未進 robustness |

X18 各次最多 3600 s 學習，最大 yaw 偏移約 211.71°、53.90°、17.41°。body yaw rate 指令限幅不等於 world yaw 限幅，roll/pitch 耦合會影響旋轉。缺乏容差切換前 checkpoint，不能斷言最終收緊容差造成遺忘。曾有 hover timeout reward exploit，相關失敗不應刪除。

2026-10-27 RL 計畫的完整場地 3/3 交接使用較早靜態場地／MuJoCo 3.12 Docker；不要與 10-28 Sim 的新場地零推力起步 3/3 混稱同一輸入。GPU／Warp／mjlab 加速尚未落地；mjviser 是顯示工具。

入口：本地 `.agents/docs/SPEC.md`、`DESIGN.md`、`.agents/plans/plan_agile_terminal_arrival_01.md`；環境、訓練器和獎勵屬 RL，物理屬 Sim。新增訓練需遵守活動計畫既有資源與授權邊界。

## 7. PX4：固件遷移完成到什麼程度

以組件的 `PX4_MIGRATION_CLOSEOUT.md`、`PX4_VER2_TRIAL.md` 為準；根目錄較早的 PX4 research 文件不能代表最新完成度。

使用真實 PX4 v1.17 固件（紀錄 commit 前綴 `d6f12ad1`）、HIL bridge、EKF2、Offboard、LAND、自動 disarm。研發遷移已在 2026-10-12 的使用者修訂範圍內完成；不是根 S0–S5 可靠性或硬體驗收完成。

| 路徑 | 既有結果 | 限制 |
|---|---|---|
| 舊 150 g proxy 完整任務 | 圖像進場、低速觸板、恢復、AprilTag 回庫、降落停槳，162.952 s 成功一次；40,730 motor frames replay 零差 | 首次 summary serialization 失敗保留；成功 1/1 不是可靠性批次；不是 Ver2 |
| Ver2 短程 SITL | 起飛、10 s hover、0.12 m 平移、返回 LAND／disarm；79.896 s，1/1；19,966 frames replay 零差 | synthetic ODOMETRY→真實 EKF2，不是真實 VIO；不是完整基地任務 |

Ver2 報告的 hover 最大 XY 4.083 mm、Z 45.825 mm、落位 4.632 mm 是仿真真值評分。舊完整任務的下視相機、80 mm tag／96 mm 背板是工程假設，2–3 g 相機質量尚未納入。歷史測試分別 459 passed / 5 skipped、462 passed / 5 skipped。

較激進 teacher／hybrid holdout 僅 1/14、9/14，失敗仍有效。不可拿低速圖像引導 nominal PASS 宣稱高速接觸策略已移植。入口為組件內 `python -m rm27 px4-sim`；與根目錄同名 package 不要混用工作目錄。

## 8. VIO 與 SLAM 必須分開

### 8.1 VIO

OpenVINS 是目前主線；舊單目工作已獨立／歸歷史路線。VIO-P 與 S0 仍 **PARTIAL**。

可靠輸入 PC 候選已有五次一致輸出 hash，但原始非同步採集失敗不能因此消失。candidate04 診斷記錄：logger 等待 10,443 μs、兩次 publication 相隔 1,498 μs、generation 跳號 815904→815906，ULog 缺少中間 gyro 積分；缺口 4,507 μs，而鏈路 76/76 無缺。這支持一次 overwrite 診斷，不是已修好所有丟樣。

整段 11,788 個積分平均 196.32949 Hz、有兩個 hole 與 startup rejection，原 ≥200 Hz、1 cm 與 timing 驗收仍未通過。148 passed / 1 ROS2 skipped 是歷史軟體檢查，不是定位精度認證。

未決：相機–IMU 外參與時偏、曝光／PTS 語義、跨時鐘映射、加速度完整性、穩定板端資料。handoff 記錄最後觀察到 candidate04 診斷版 ON；不能不查設備就當成現在的固件狀態。原 `.maixpy` 等本地原始資料也不會隨 Git clone 自動出現。

### 8.2 SLAM

獨立 MaixCAM2 單目研究起步包，既有 176 tests（含 clean clone）記錄。主機 Stella 公開資料集路線可用；stock ORB shutdown 曾 segfault，patched 路線成功，但精確 patch 依賴外部材料。

板端 KLT 前端測量：336×190／180 points／1 thread 約 29.33 FPS；672×380／400 points／2 threads 約 10.09 FPS。**這只是稀疏跟蹤前端，不能當完整 SLAM FPS**。OS04A10 標定有 27 fit + 7 validation，僅對同一光學設定有效。

完整後端的記憶體、建置、板端實時、單目尺度與飛行可用性仍未知。舊 README 中 `rm27_slam-vio` 名稱不應用來合併目前 SLAM 與 VIO 職責。

## 9. Tracking：高速採集與真正可用觀測的差別

主要來源是 upstream `IMPLEMENTATION_STATUS_V0.2.md` 與 `GREEN_DETECTION_PLAN_FULL180.md`（2026-09-09）；不是今天重新量測的結果。

已有 CPU 多尺度綠燈／裝甲檢測、LOS Kalman、track/state、`safe_for_control` 邊界；PnP／NPU 接口存在不等於標定／權重驗收完成。

- 480×360@60 的 185 min 紀錄約 60.001 FPS、P95 17.37 ms。
- 1344×760@180 的 30 min 紀錄：324,245 frames、180.135791 FPS，無序號異常；綠燈／裝甲調度約 88.672／59.792 Hz，估計輸出約 179.998 Hz，接收後 source age P95 22.992 ms。
- 自報 direct observation 約 10.433 Hz，沒有標註真值；不能解讀為召回率，也不能說 180 Hz 有效目標檢測。

全視野模式未標定，`fx=fy=1344` 是占位；曝光到控制的完整延遲未知。目前無效角度／control-safe=false 是正確拒絕。高速入口仍是隔離驗證程序，普通 APP >60 fps 會拒絕，NPU/Pose 未接入該高速入口。

待做的設計：多候選各自累積時序證據、全視野多尺度／背景正規化、原生資料與硬負樣本、motion transform 起訖時間、標定及帶真值驗收。單一確認器輪流處理五個候選，不能保證 100 ms 捕獲。

15–25 m 的 300 次捕獲／準確率驗收、UART/CAN 完整輸出鏈路與真實端到端延遲尚未完成。這不否定 PX4 分支另有離線 green AXMODEL replay；兩條路線的部署成熟度要分開描述。

## 10. MJCF 與可動基地

資產 repo 負責模型與 runtime assets，Sim 負責組合、任務與 viewer。目前修補新場地來源為 `rmuc2027_mjarena` 的 `ffd1eb7` 系列；不能把早期未修補場地的誤差套到後來修補版。

藍方綠燈、上裝甲與支架沿滑台移動，下方斜裝甲固定；約 ±0.3005 m 行程，預設居中。`RailMotion` 用 NumPy 產生 actuator 控制，支援 reset／seed／速度／停留，不直接 step physics 或 teleport state。

既有表面對齊審核：兩側各 441 裝甲 + 313 燈 rays，共 1508；裝甲誤差 <0.001 mm、燈 <0.058 mm，32 次 1／5 m/s 球撞擊有接觸。薄殼碰撞在有限步長仍可穿入，5 m/s 約可達 5 mm；不是零穿透或材料驗證。

新場地 manifest 145 個 runtime hashes；四 Ver2 靜置 2500 steps 最大 penetration 約 0.2314 mm。四架模型停著不代表四控制器或多機協同。

獨立場地比較在相同 MuJoCo 3.17 記錄 old 1.165 ms、new 0.613 ms、old+fusestatic 0.477 ms 的 median step；碰撞與 contacts 數不同，不能把性能差全歸幾何。mjviser／viser 瀏覽器顯示已有初始化與 FPV 檢查，不是 GPU 物理或學習加速。

## 11. 歷史研究：哪些成果仍有參考價值

### 11.1 S0 與走廊

歷史 ArduPilot S0 十次獨立 SITL 名義短程起降 10/10，導航有 MuJoCo truth 輔助，落位 XY 約 1.44–2.83 mm。此數字不是真實定位精度。

走廊兩次完整路線約 593／594 s，各經七個點、返航降落；最大路線偏差約 14.65／14.35 mm。最後到裝甲的 0.6 m 不在驗收內，完整基地碰撞也未因此通過。只支持舊場景的靜態真值走廊。

### 11.2 接觸、whole maneuver 與 teacher

aggressive-contact v1/v2 已研究首次指定接觸、恢復、motor lag、推力與板邊緣余量。關閉 recovery feedforward 降低角速率卻令恢復更慢；更大推力也不保證更好。失敗分類包含錯誤首次接觸、非指定部位接觸、miss、恢復逾時。

舊 180 g／2026 whole-maneuver 從機庫上方 **空中**開始，分段約 14.94 s、compressed 8.87 s、fast 5.77 s、early-pose 7.10 s；各自有限留出集結果不同，不能當新 Ver2 地面起飛結果。

Teacher balanced nominal 接觸恢復後連續返航至機庫附近，約 20.066 s，未精確降落；不是全流程停槳起飛。Dataset v1.1 有 320 demonstrations、762,836 samples、12 失敗保留。train／validation／holdout 按 scenario 隔離；不能用 holdout 訓練後仍稱其為獨立測試。

Recoverability kNN 有預測價值但未節省總計算；它是指定控制器下的經驗分數，不是可恢復域證明或校準成功概率。rate/thrust proxy 與真飛控不等價，physical N 不能直接當 normalized throttle。

### 11.3 蒸餾與估計器研究

純 action BC／history／DAgger 在所研究配置閉環 0/18；不能推論所有神經網路必然失敗。有效的是 **學習 B-spline reference + 保留模型 feedback／recovery**，不是純神經控制器或 neural residual。

初期 hybrid ideal 72/72、synthetic noisy 70/72、加 10 ms ego delay 59/72；後續 Phase3 的 20 ms noisy 組從 16/72 改善到 68/72，small-noise 66/72 保持。host runtime 可用不代表 M3C 實時已驗證。

Phase4 真 ArduPilot：truth ego nominal 接觸誤差 7.47 mm PASS，EKF telemetry 82.72 mm 撞 rim FAIL。Phase5 synthetic external-navigation→真 EKF、關 GPS，獨立 sensor-seed holdout 9/12 接觸／恢復；不是相機 VIO。commitment guard 的 safe abort 不能算任務成功。combined-error 16/16 僅對那組誤差／延遲設定成立。

這些結果支持把時序、估計偏差與內環模型納入研究，沒有完成新場地 Ver2 的完整機庫任務。

### 11.4 影片、概念動畫、攔截

原第一視角接觸返航影片來自 `artifacts/teacher-return-01/balanced` recorded trace；FPV 匯出 `artifacts/teacher-return-fpv-20260927-v2/first_person.mp4`，約 20.1 s。鏡頭隨機身姿態，光學參數是 CAD 對齊／理想針孔假設，不是視覺閉環。

獨立 concept mission 是為展示平順動作製作的概念動畫；忽略控制與物理的展示不能作為撞擊穩定性證據，且返回機庫附近不等於降落。

飛鏢攔截研究使用舊 180 g proxy、truth 與預定飛鏢路徑；記錄碰撞約 2.294 s、impulse 0.393 Ns，未介入時約 2.870 s 命中 proxy。沒有真實相機追鏢與撞後穩定恢復驗收。不要與基地接觸任務合併評分。

## 12. 優先研究問題與目前缺口

以下是根據現有證據整理的研究順序，不是已完成的工作或新增執行授權。

1. **新場地連續任務接通**：從既有 moving handoff 接到首次有效接觸，保留完整狀態；再接恢復、回程與原生機庫落位。先使場地／幾何／評分一致。
2. **支承與接觸件合理性**：實物底部件尺寸、突出量及停放支承需確認，避免靠仿真凹槽與底球假設取得不可轉移的結果。
3. **動力辨識**：以原始 log／參數、推力測試等建立質量、慣量、推力、lag、yaw 模型；區分不可辨識項，不把一次仿真吻合當唯一真值。
4. **RL 新任務契約**：落實原生 Ver2 首次接觸 task/reset/evaluator，再決定有限訓練；保留舊任務結果與失敗，不能用新門檻改舊結論。
5. **導航和目標觀測**：解決 VIO 採集／時序、Tracking 標定與真值驗收，再評估接觸前後的延遲、偏差及失鎖，不能用 source FPS 代替可用觀測率。
6. **固件整合**：在相同機型與場地用真 PX4，分清 truth、synthetic odometry、實際 VIO 的貢獻；舊 proxy PASS 不替代這一步。
7. **重複性與擾動**：保留所有嘗試、固定門檻、獨立 seed，分開 nominal repeat、paired perturbation 和 recorded replay；之後才討論真機遷移與多機。

## 13. Git、本地資料與 GitHub 的差距

以下為本次讀取的本地 snapshot；dirty 包含修改或未跟蹤項，不能由 HEAD 單獨重現。乾淨只指這次 `git status --porcelain` 輸出為空，不代表分支已合併 main。

| 倉庫 | 分支 | HEAD | 狀態 |
|---|---|---|---|
| root | `astra/research` | `1fc395c433cf4574816047a362e464d6279fc0c9` | dirty |
| RL | `i4/informed-reset-arrival` | `ddf159395d5b1391011a0839732b6b7f5a3807b6` | dirty |
| Sim | `feat/moving-base-target` | `f414b4c7517db7eae79ffee112d60bc45f2ef302` | dirty |
| MJCF | `feat/moving-base-target` | `d9439a97cee80baba9d13a1de9882f2d8095fca5` | dirty |
| PX4 | `docs/progress-handoff-rule` | `539ce7412b98ae67c53bf93c46d2595683d2c38a` | clean |
| SLAM | `docs/progress-handoff-rule` | `3e94df8a85815ed6042de6d87b6dba9cda14e625` | clean |
| VIO | `docs/progress-handoff-rule` | `a8392c6f1e32e76532bd3ea8aa34ef98e0f06da6` | dirty |
| Tracking | detached HEAD | `ac83e8c224214bc99483f9617450e110c5c16022` | clean |

`.gitmodules` 的 simulator URL 本地已是 `shiuhou/rm27_sim`，但主倉庫該檔仍有未提交修改。父 repo 的 gitlink 是某個固定 commit，不會因 component main 更新而自動移動；`branch = main` 也不等於本地 checkout 就是 main。

根目錄許多歷史研究文件、config、程式及最新場地 config 尚未跟蹤／提交；RL／Sim 也有正在進行的本地變更。`.agents/plans/`、`.agents/runs/` 通常忽略，`artifacts/` 是本地產物，CAD、logs、checkpoints、Docker images 不自動隨 GitHub 源碼可用。這正是單一 `report.md` 必須包含結論而非只放連結的原因。

初次整理沒有 commit／push／merge。其後使用者授權文件補充與 PR，本報告和導覽修訂以獨立文件分支發布；未發布表中其他 dirty 程式、原始資料或變更 component gitlink。PR 清單是建立文件 PR 前的查詢快照，不替代逐 PR 的合併審查。

## 14. 以後如何維護這份報告

**固定總覽位置就是根目錄 `report.md`，持續更新同一檔案，不新增 report-v2／每日總結副本。** 使用者可直接上傳這一檔，附上當次研究問題。

- 各任務仍在所屬 repo 的唯一活動計畫 `§0 進度與交接` 維護詳細進度／todo；正式 SPEC、DESIGN、契約各自保有權威。
- 跨組件里程碑、驗收結論、工作位置、硬體／場地基線、阻塞或發布狀態改變時，同步更新本報告相關段落、日期和版本。小修補不必全文改寫。
- 獨立 component 工作時先更新其本地計畫與必要正式文件；可存取主工作區時再更新這裡。無法存取時明確留下「總覽待同步」，不能假稱已同步。
- 每條成果至少交代機型／場地、控制與導航來源、配置、結果分母、失敗與限制、證據路徑；新增實測與假設分開。
- 本報告是派生快照，不新增一份平行驗收標準或任務權威。遇到衝突回到最新使用者要求、原始資料與所屬契約，修正本報告。
- 不把密鑰、完整原始 logs、大型 CAD、影片或虛擬環境塞進 Markdown／Git。必要時另外提供明確選定的附件。

給只收到本檔的研究者：可分析架構、瓶頸與實驗設計；無法僅憑文字核實原始軌跡、接觸波形、訓練 checkpoint 或影片內容。需要這些時，請明確指名資料、版本和要驗證的假設，不將缺失證據補成事實。

## 15. 來源索引與可重現邊界

以下為本次總結所依據的文件群與本地計畫。路徑以工作區根為準；外部讀者不需逐一打開才能讀懂正文。本報告沒有內嵌全部原文、原始資料或第三方程式碼。

| 主題 | 主要來源 |
|---|---|
| 全局範圍／啟動／職責 | `README.md`、`START_HERE.md`、`AGENTS.md`、`docs/INTEGRATION.md`、`docs/REPOSITORY_MAP.md`、`docs/LEGACY_COMPONENTS.md` |
| 工程方法／里程碑 | `.agents/docs/engineering.md`、`docs/MILESTONES.md`、`baselines/README.md` 與各研發線 handoff |
| 機型／硬體 | `docs/REAL_HARDWARE.md`、`docs/DRONE_V2_SIMULATION.md`、`configs/real_hardware.json`、`configs/field.json` |
| 舊整合驗證 | `docs/S0_REPORT.md`、`docs/MICRO_FLIGHT.md`、`docs/CORRIDOR_REPORT.md` |
| 接觸與 teacher | `docs/AGGRESSIVE_CONTACT_REPORT.md`、`docs/AGGRESSIVE_CONTACT_V2_REPORT.md`、`docs/WHOLE_MANEUVER_REPORT.md`、`docs/WHOLE_MANEUVER_TEACHER_REPORT.md` |
| 蒸餾／估計器 | `docs/WHOLE_MANEUVER_DISTILLATION_REPORT.md`、`docs/HYBRID_STUDENT_DEPLOYMENT_REPORT.md`、`docs/HYBRID_POLICY_SITL_REPORT.md`、`docs/ESTIMATOR_AWARE_HYBRID_MANEUVER_REPORT.md`、`docs/MARKER_VISIBILITY_CONTROL_REPORT.md` |
| 動畫／攔截／歷史產物 | `docs/CONCEPT_MISSION.md`、`docs/DART_INTERCEPTION.md`、`docs/ISOLATED_RED_DART.md`、`docs/ARTIFACT_INDEX.md` |
| Sim 目前任務 | `components/rm_uav_mujoco/README.md`、其 `.agents/plans/plan_impact_recovery_01.md`、`plan_field_migration_1.md`、`plan_base_rail_1.md`、`plan_arena_comparison_1.md` |
| 拆分／發布歷史 | 根 `.agents/plans/plan_simulation_separation_01.md`、`plan_github_submodule_sync_01.md` |
| RL | `components/rm27_RL/README.md`、其 `.agents/docs/SPEC.md`、`DESIGN.md`、`.agents/plans/plan_agile_terminal_arrival_01.md` |
| PX4 | `components/rm27_drone_px4/README.md`、其 `docs/PX4_MIGRATION_SCOPE.md`、`PX4_MIGRATION_CLOSEOUT.md`、`PX4_VER2_TRIAL.md`；根 `docs/PX4_MIGRATION_RESEARCH.md` 為較早背景 |
| VIO | `components/rm27_vio/README.md`、`HANDOFF.md`、`AGENTS.md` |
| SLAM | `components/rm27_slam/README.md`、`HANDOFF.md`、`docs/STATUS.md`、`AGENTS.md` |
| Tracking | `rm27/perception/vision/maixcam2_dart_vision/docs/IMPLEMENTATION_STATUS_V0.2.md`、`GREEN_DETECTION_PLAN_FULL180.md` |
| MJCF | `components/rm_mjcf/README.md`、本地碰撞修補計畫及 Sim 的 field／rail migration 計畫 |

`ARTIFACT_INDEX.md` 的 356 directories／48.91 GiB／305 summaries 是 2026-10-16 的歷史盤點，不是現在的總數，更不是整個任務成功率。

### 本次直接核對的最新結果

兩份 summary 相對 Sim 根目錄：

- `artifacts/impact_recovery/full_field_approach_ground_v1/summary.json`
- `artifacts/impact_recovery/full_field_approach_ground_repeat/summary.json`

共同 SHA-256：`b1d2778fcc35d7e84eccfbcb4159ae844445d77e850862a29d0867f576aafb91`。

兩份皆 `passed=true`，三個 flight、launch、handoff window 全通過；`inputs_unchanged=true`；三個 `restore_max_abs_error=0`。這是讀取紀錄與檔案 hash 的結果，本次未重算每幀物理或重新驗證所有輸入檔 hash。歷史研究其他大量數字屬前述「既有紀錄」。

### GitHub open PR 快照（2026-10-29，建立本輪文件 PR 前）

| Repository | 查得的 open PR（base 均為 main） |
|---|---|
| `shiuhou/rm27_drones` | 無 |
| `shiuhou/rm27_RL` | 無 |
| `shiuhou/rm27_drone_px4` | 無 |
| `shiuhou/rm27_vio` | 無 |
| `shiuhou/rm27_slam` | 無 |
| `shiuhou/rm27_sim` | #2 可動基地／mjviser／場地整合；#3 撞擊恢復與 Ver2 局部連續任務 |
| `CosmosMount/rm_mjcf` | #2 基地燈／裝甲碰撞；#3 修補場地與可動基地 |

此處查詢的是 open PR 清單；「無」不判定歷史 PR 是 merged 或 closed，也不證明本地工作全部已發布。本次沒有完整比較每個遠端 main 與 dirty checkout。Tracking 與其他 vendor 的遠端 PR 未盤點。

### 關鍵來源檔案 SHA-256

用於辨識這次讀取的版本；本地活動計畫即使未上 GitHub，也可透過本表與正文定位。維護本報告時，實際重讀並更新受影響來源的 hash，不將舊 hash 當成新版驗證。

| 工作區相對路徑 | SHA-256 |
|---|---|
| `configs/field.json` | `64b849a196c4ebb0aa47eb61b1498df7c9c397ee20369735cfbc13d49269182b` |
| `docs/MILESTONES.md` | `1604d880eb1a7acf9aa798661aa9176834dc839c65e4b9f8f48d7d1845b6baf0` |
| `components/rm_uav_mujoco/.agents/plans/plan_impact_recovery_01.md` | `41486a8e16994341d7e96486ce88a97e195ed1f1675d8dd2578f3ef8e51f1c1e` |
| `components/rm27_RL/.agents/docs/SPEC.md` | `4d793a5e27a0388fdaa2616b5c5889324c09a083e569413e31e051133abd8e4b` |
| `components/rm27_RL/.agents/plans/plan_agile_terminal_arrival_01.md` | `f49da8617892881c0b9d0fe6847a6fbd697abdde4b66ff694a006d6b2bd6fee8` |
| `components/rm27_drone_px4/docs/PX4_MIGRATION_CLOSEOUT.md` | `08c32fcca9fb7de15c09b3e5a2c740c4c5fafb0bd97b7c94af19e0d6fd459eaa` |
| `components/rm27_drone_px4/docs/PX4_VER2_TRIAL.md` | `f2ff4409a7b7b6979f0d1e8d7fa8facbc566df351a003be6f7cbcf6135f91cbf` |
| `components/rm27_vio/HANDOFF.md` | `3a08f18caae77088f370e7a81a286d1812615945a1ef0357be3d3b43a373d698` |
| `components/rm27_slam/docs/STATUS.md` | `b98055e3d512de09f0307414ceb462aed869f4dd8de6c90d6de8610b4c839e71` |
| `rm27/perception/vision/maixcam2_dart_vision/docs/GREEN_DETECTION_PLAN_FULL180.md` | `a0ed08c35b46a0b92c02f67762bc7d3a4ebffc00344ad7ec015f0e4ca65ffcfd` |

### 文件 PR 準備時的發布版本核對

本輪從遠端 main 建立獨立文件工作區，沒有把研發工作區整批提交。已重新 fetch 的 main：

| Repository | main SHA |
|---|---|
| root | `8abf971cc26d325090ad5eaa19a18a711ebc7835` |
| RL | `aff12170e0c0c84ce11554fcf49dbeb6a231eb85` |
| Sim | `57e73658186ccee886f8560011413ec3fcf605f3` |
| SLAM | `a6bb60de08a6cea33df9f1bc1db4db4dec22193f` |

關鍵差異：RL main 仍包含 `packages/physics/`，其安裝命令應使用同 checkout 內的套件；本地研發分支才改用外部 Sim Physics。Sim main 已有獨立 `rm27_sim/`，可動基地與 impact 工作另見 PR。主倉庫 main 的 `.gitmodules` 仍指向舊 `shiuhou/rm_uav_mujoco` URL，父 gitlink 也未因本文件 PR 升級；正文新架構描述的是研發工作區和現行所有權，不是聲稱每個 main 已完成同步。不能只把 README 命令換新，就假裝程式與版本釘選已遷移。
