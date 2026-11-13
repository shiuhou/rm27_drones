# RM27 全倉庫研發進度與研究背景

RM27 Non-RL 最新追加診斷：**0.95 m/s 是配置的機體總速度資格上限，非物理極限；超過它不等同不可恢復。** 使用者授權 1 趟，已完成 `straight_contact_09_diagnostic`：保持 08 控制器、估測、起點及 seed，只允許超速後有界觀察；原速度 FAIL 永久保留，其他原停止條件不變，另設診斷 2.0 m/s 上限。共同前 412 個物理狀態逐位相同，所有保護來源與原始證據雜湊不變。首觸仍在 1.8426 s 合格；2.0564 s 首次超 0.95，峰值 1.032029，約 2.215 s 降回限額內；2.100 s 達 8 cm 分離，2.650 s 開始符合原穩定條件。沒有承載禁碰、再次接觸或馬達飽和。**正式任務仍 FAIL，完整一秒恢復未證實：** 2.9056 s 越過原 x 下界 10.8861 m 而停止，速度已降至 0.256264 m/s、傾角 0.73465°，穩定段只觀察到 0.2556 s。不是計算超時；wall 53.91 s。這證明先前的速度截斷遮住後續分離／減速，並未證明框外完整恢復。保持原評分，不把短暫穩定改成成功。

[診斷回放](components/rm27_non_rl/artifacts/asm2_handover/replays_fixed_camera/straight_contact_09_diagnostic/maneuver.gif)／[速度與事件圖](components/rm27_non_rl/artifacts/asm2_handover/speed_diagnostic_01/comparison/speed_envelope.png)／[配對完整性](components/rm27_non_rl/artifacts/asm2_handover/speed_diagnostic_01/integrity.json)。活動計劃 §0 記錄唯一進度與授權。未做新控制器調參、獨立留出、硬體、RL、commit 或 push。

RM27 Non-RL 最新派生結果（2026-11-14）：**進場與合格首觸 PASS；連續恢復 FAIL。** 使用者追加有界授權後完成協同七階減速／高度／姿態修正及 6 趟原生配對（`straight_contact_03`–`08`），每趟先通過實際 sensor-runtime 採用預檢。相同開發起點／seed，0.820 s 帶速接入，交接同拍 CTBR／馬達差零；首觸前前向速度峰值由 0.74954 降至 0.47242 m/s，最大速度回升由 0.57727 降至 0.01744 m/s，最大側移 8.73 mm。六趟都在 1.8426 s 由後底片合格首觸，法向約 0.07160、切向 0.04306 m/s；首觸點板邊餘量最小 14.67 mm。不代表六個獨立成功案例。

六趟在各自終止前都未記錄原 scorer 的承載機身禁碰，但均在 2.0454–2.0702 s 觸發原 0.95 m/s 速度包絡，沒有達到正式分離及 1 秒保持。固定 COM 回正間隙等待、機體沿斜面加速與角速度／馬達反應形成時序矛盾；逐步幾何回正、既有馬達 lead、參考導數與較強推離改善部分反應，仍未達整趟成功。原始評分、physics、感測／估測均未改，沒有 GT runtime 補值或馬達 rail。07 離線保存姿態的最小 forbidden-pair 距離為 **−0.0410 mm**；微小幾何重疊和未產生承載禁碰分開報告，不宣稱正間隙或連續安全。有限板面檢查否決了另一個提早分離候選，沒有用板外頂點或真實首觸時間補資格。 最後 08 在停止前已達原分離度量 6.300 cm（要求 8 cm）、傾角 13.27°，速度 0.950141 m/s；因此是已推離但未完成正式分離／穩定保持，不能把這個中間狀態算成功。

驗證：32 focused、159 Non-RL、486 Sim PASS，Sim 另 5 skipped／4 subtests PASS（含 M3）；3126 份受保護歷史 artifact／physics 雜湊不變，所有新趟 scene hash 同舊趟，03–08 分離前 integration 逐位相同。單趟原生計算 39.29–43.35 s，沒有 timeout；本輪 6/6 原生、2/2 離線開發輪已用完。沒有新獨立留出、真實感測、硬體、RL、commit 或 push。唯一進度／未決項在 [活動計劃 §0](components/rm27_non_rl/.agents/plans/plan_asm2_autonomous_handover_01.md)。[逐案數據](components/rm27_non_rl/artifacts/asm2_handover/coordinated_results_01/results.json)／[比較圖](components/rm27_non_rl/artifacts/asm2_handover/coordinated_results_01/comparison.png)／[07 全程回放](components/rm27_non_rl/artifacts/asm2_handover/replays_fixed_camera/straight_contact_07/maneuver.gif)／[07 接觸慢放](components/rm27_non_rl/artifacts/asm2_handover/replays_fixed_camera/straight_contact_07/contact_slow.gif)／[08 回放](components/rm27_non_rl/artifacts/asm2_handover/replays_fixed_camera/straight_contact_08/maneuver.gif)。

RM27 Non-RL 前一輪配對試驗歷史（2026-11-13）：當時唯一追加趟 `straight_contact_02` 在 0.760 s 被原接觸幾何 gate 拒絕，未採用末段，整體 FAIL。前 153 個狀態與舊趟相同；當時 29 focused／156 Non-RL PASS，沒有物理改善證據。此結論保留，上方 2026-11-14 批次為後續實際驗證。

RM27 Non-RL 最新派生診斷（2026-11-13）：`straight_contact_01` 的兩段前進由採用參考本身的先減速／再加速產生（0.149→0.782 m/s）。分離排程依預測 5 mm 接近窗口啟動，比實際首觸早 20.1 ms；碰後向後／下移及回正期間，前左下側非接觸區在 2.0722 s 碰板下緣。錄製分析與時間軸已保存，零新增原生步；未證明單一控制改動可解除失效，控制器與評分未改。唯一進度與下一項仍見 Non-RL 活動計劃 §0；原整趟 FAIL，修正後物理驗證 NOT_RUN。

六份共用契約 Phase 2（2026-11-13）｜**`0.1.0-draft` 結構與合成語義驗證通過。**
[機讀草案](contracts/v0.1.0/README.md)包含六份 Draft 2020-12 schema、一份共用 schema、
39 份合成樣本及離線測試；Phase 1.1 七份凍結文件的 SHA-256 全數不變。14 項測試通過：
18 份有效樣本通過，10 份結構反例及 11 份語義反例按預期拒絕。複製必要檔案至隔離目錄，
在測試程序封鎖網路連線及 DNS 後重跑，結果相同。既有三個介面回歸檔前後均為
16 通過、1 既有失敗：歷史 source lock 拒絕 RL `src/rm27_contact_rl/__main__.py`，
完整跨程序鏈未執行。本輪未變更現有 runtime、模型、評分器或子模組；消費端相容性未驗證，
Runtime Adapters 與整合須另行批准。

六份共用契約 Phase 1.1（2026-11-13）｜**設計審閱完成；0.1.0 草案凍結。**
[基線與決策](.agents/docs/contracts/README.md)已明定 Sim／PX4 分配與消費端適配分工、
接觸點速度推導、獨立禁碰事件及各階段可行性證據來源。原四項成功條件保留。
僅核對文件；RL 消費端、模型／評分相容性與感測自主能力仍待驗證。Phase 2 待另行批准。

六份共用契約 Phase 1 歷史快照（2026-11-13）｜**初版文件草案。**
[契約索引](.agents/docs/contracts/README.md) 收錄六份規格與 25 份本地來源 SHA-256。
總倉擁有契約；Sim 擁有物理、幾何與獨立評分；Non-RL／RL 消費接口並保留各自算法。
本機 XML 靜態核對為 3,522 個碰撞 geom／15 個接觸片；附件所述 11,874 hull 的
版本與雜湊尚未確認。完整原生慣量與對角控制代理、ASM2 原生 scorer 與舊 RL
evaluator、取樣淨空與連續掃掠等差異已明列；RL 消費端標為待確認。
本輪文件檢查通過：7 份草案的 49 個本地連結／錨點有效，接觸片名稱與 metadata
一致，主慣量重建完整矩陣誤差低於 1e-12 kg m²。
本輪只執行文件、靜態資料及來源身份核對；沒有 runtime／physics／訓練／部署變更、
SSH／遠端稽核或 Git 發布，未提升任何歷史結果的驗收資格。
來源清單保留初次檢視雜湊；23／25 份在交付核對時一致，另外兩份 Non-RL 來源
有並行修改，已重新檢視相關定義並另列觀察雜湊；S12 隨後持續變動，後續版本
不在本次檢視快照內。本清單不是凍結版本鎖。

RM27 Non-RL 分工快照（2026-11-13）｜研究線正式名稱為 **RM27 Non-RL**。
傳統 ASM2 規劃、引導、估測與恢復歸 [rm27_non_rl](components/rm27_non_rl/README.md)，
目前為總倉擁有的本地 component，未建立獨立遠端、版本庫或分支；共用原生物理、
場景、合成感測、scorer 與回放歸 [rm27_sim](components/rm27_sim/README.md)，
沿用原 `shiuhou/rm27_sim` 版本庫；`components/rm27_RL/` 保持原有訓練／評估邊界。
舊 `components/rm_uav_mujoco` 為相容 symlink，供凍結證據、RL 及 source lock 使用；
下文歷史連結保留原路徑。此遷移未新增物理軌跡，也不構成新的接口或任務資格。
本次工程核對：Non-RL全套151 PASS；Sim全套486 PASS、5 skipped、4 subtests PASS
（含既有M3檢查）。六個新舊CLI入口通過，共用Sim在封鎖Non-RL／RL匯入後可獨立使用。
支援原環境的editable來源安裝；可攜wheel及部署尚未驗證。
另行只讀核對總倉來源鎖：8份input一致，62份來源中RL三份及PX4 runtime一份不符，
preflight FAIL、完整跨程序驗收NOT_RUN；本次未修改上述來源或來源鎖，也未判定變更原因。
詳見[來源鎖檢查](components/rm27_sim/artifacts/asm2_handover/migration_cross_repo_preflight.json)。

R3.1–R3.2 執行快照（2026-11-13）｜**自主接管已有實際行為，完整任務驗收 FAIL；擴大起點 NO-GO**。
兩條凍結 witness 重現完整成功；dev01–03 的三條真正無接觸安全退出保留。
幾何查詢加速後 dev04 在 2.06 s 自主接管、3.1037 s 合格首觸，3.1047 s 已禁碰，
早於 3.105 s 分離命令；不是恢復控制器介入後的失敗證據。提前換軌 dev05–10 無完整收益，
該 gate 放寬已撤除，所有失敗保留。凍結後六新起點 × 舊固定交接／新自主交接共 12 案：
兩方法均 0/6 合格首觸、0/6 完整恢復。新方法四案 1.86 s 實際調整後接管並後續禁碰；
另兩案幾何拒絕後停止，不能算安全退出。舊方法 6/6 禁碰、4/6 分離，新方法 0/6 分離。
新案例在末端規劃前已有約 2–3.4° 姿態估測誤差；0.5° 壓力假設不是校準界，
尚未證明單一因果。146 筆局部候選查詢 108 通過只是模型結果，不能當作物理成功率。
已用 24/24 任務、4/8 短期預測；不追加預算、不依留出再調參。Non-RL 最終 154 PASS，
Sim/M3 486 PASS、5 skipped、4 subtests PASS；3130 歷史 hash 與凍結 47 runtime/config hash 全同。
原生 scorer/physics/BNR/1g detector 保留；合成感測＋登記靜態目標，無真實 VIO／校準信心資格。
[結果與重現](components/rm27_sim/artifacts/asm2_handover/final_analysis/results.md) ·
[逐案量化](components/rm27_sim/artifacts/asm2_handover/final_analysis/comparison.json) ·
[成功 witness 回放](components/rm27_sim/artifacts/asm2_handover/replays_fixed_camera/baseline_0/maneuver.gif) ·
[自主接管次觸失敗](components/rm27_sim/artifacts/asm2_handover/replays_fixed_camera/dev04_enclosure/maneuver.gif)。
2026-11-13 另依使用者指示做一次**無接觸直線進場**：同原感測設定與 seed，從登記綠燈前
0.55 m 起步，2 s 前進 22.1 cm，側移最大 2.80 cm，無接觸。1.20 s 對準器接受時，離線航向
誤差 6.30°（原側飛在同時刻 0.44°），因此「用直飛直接取代前置對準」驗收 FAIL；實際飛行未接入末端，
未調原對準器，不消耗或擴充 24 次接觸任務預算。隨後用同一錄製離線重放，253 筆估測一致，1.20 s
候選通過 30 ms 規劃限時、1.26 s 在軟體影子流程採用。原直飛在 0.595 s 達前向峰值 0.382 m/s 時
尚未對準；1.20 s 前向速度約 −0.010 m/s。該軟體採用只證明近停狀態通過現行檢查。
[單案結果](components/rm27_sim/artifacts/asm2_handover/straight_alignment_01/summary.json) ·
[直飛回放](components/rm27_sim/artifacts/asm2_handover/straight_alignment_01/maneuver.gif) ·
[末端軟體重放](components/rm27_sim/artifacts/asm2_handover/straight_terminal_replay_01.json)。

同日另跑一次**無接觸帶速交接**：0.820 s 以 0.472 m/s 接入安全距離內的減速參考，沒有先停；
最近水平前向距離 32.7 cm，側移最大 5.1 mm，全程無碰撞或馬達限幅。120 ms 混合使交接當拍同狀態
命令差為零，實際相鄰 5 ms 最大單軸 rate／單馬達命令差為 0.00303 rad/s／0.00110 N。離線交接
航向誤差 +3.47°，高於本次 0.5° 比較值。帶速交接及命令連續 PASS，航向精度 FAIL；原接觸任務未重跑。
[帶速交接結果](components/rm27_sim/artifacts/asm2_handover/moving_handover_01/summary.json) ·
[固定相機回放](components/rm27_sim/artifacts/asm2_handover/moving_handover_01/maneuver.gif)。

只讀兩條直飛錄製的航向診斷顯示：帶速案同受外部速度與 IMU 積分誤差影響，往返案主要受 IMU 積分端影響。
聯合位置／速度與積分殘差加權，把帶速交接誤差從 3.47° 降至 0.91°，但往返案仍 4.64°，且換初始化時窗未一致改善。
離線候選未通過 0.5° 比較值，未改正式估測或飛行，0 新物理步；Non-RL 154 PASS。
[配對重放結果](components/rm27_sim/artifacts/asm2_handover/heading_replay_01.json)。
上游接續審核（2026-11-13）：**來源及既有命令核對 PASS；可信航向接續 BLOCKED。** 兩條直飛只有合成 IMU 與位置／速度，三個決策時刻同趟上游航向均 0 筆。綠燈 NAV 航向設定值不是機身姿態；已錄 PX4 四元數來自另一 SITL 場景且融合 truth-generated ODOMETRY；OpenVINS 原生姿態也不屬於該趟，現有 adapter 的 frame 映射尚未驗證。原 0.820 s 帶速交接的指令差重算一致，航向 +3.470° 仍未通過原 0.5° 比較值。54 項介面測試 PASS；未補真值、未發新命令或執行新物理。[離線審核證據](components/rm27_sim/artifacts/asm2_handover/upstream_heading_audit_01.json)。

無接觸直飛航向敏感度（2026-11-13）：**固定錄製狀態下，原樣與 ±3.47° 均通過離線帶速接入。** 固定 0.760 s 提案／0.820 s 生效；原樣 400 拍指令逐值重現。三組交接當拍指令差均零，接入後均 0/236 拍馬達限幅，最壞馬達命令 0.4110 N（上限 1.103625 N）。偏航指令最多相差 0.426 rad/s；採樣參考機身間隙維持 22.66 cm，錄製估測姿態的靜態間隙最多減少 0.60 mm。原 0.5° 仍是未通過的估測精度比較，並非此無接觸交接的安全閘門。這次不優先修改航向；不推論受擾動閉環間隙或接觸結果。4 項相關測試 PASS；0 新物理步、0 發送命令、不補真值。[三組離線結果](components/rm27_sim/artifacts/asm2_handover/straight_yaw_sensitivity_01.json)。

原生閉環直飛與保持（2026-11-13）：**PASS（同 seed 一趟，無接觸）。** 0.820 s、實際前速 0.4724 m/s 接入，1.480–2.600 s 穩定保持 1.120 s，末速 0.00389 m/s。全程最大側移 9.72 mm，原生機身幾何 5 ms 採樣最小間隙 22.05 cm（1.420 s），最小前向標記距離 32.69 cm。馬達 0/520 拍限幅；峰值命令 0.3883 N（上限 1.103625 N），實際推力峰值 0.3734 N。航向誤差 +3.470° 未達原比較值；沒有補真值或加橫向激勵。26,000 原生步；前 401 個保存狀態與原帶速試驗逐位相同，延長保持不算新起點。未測航向擾動閉環或接觸。[回放](components/rm27_sim/artifacts/asm2_handover/moving_handover_02/maneuver.gif) · [量測](components/rm27_sim/artifacts/asm2_handover/moving_handover_02/measured_hold.json)。
原場景目標接觸板的追加閉環（2026-11-13）：**帶速接入 PASS，正式接觸與完整恢復 FAIL。** 一趟 seed 231101、21,467 原生步、2.1467 s 模擬／46.37 s driver wall；既有直飛估測、原控制器與評分，不先停、不加側飛、不補真值航向。0.760 s 求解 21.41 ms、0.820 s 以 0.47242 m/s 採用一條固定末段參考，當拍命令差零。1.8851 s 後側底部接觸片先接觸目標板，法向／切向速度約 0.0449／0.0764 m/s，表面法線誤差 1.960°，物理 contact records 無拒絕；但預定分離已於 1.865 s 開始，原正式 phase=APPROACH 條件不符，qualified_contact=false。2.0722 s 機身禁碰，未達分離或一秒保持；最後速度 0.950209 m/s 超原 0.95 上限停止。IMU 事件 2.080 s 才送達。430 拍馬達無到達上下限、峰值 0.58592 N；5 ms 保存狀態最大側移 13.62 mm。未追加試跑、未改正式 runtime。初始位置換算與舊錄製有最多 6.26 μm 差異，不宣稱逐位重現。48 聚焦／154 Non-RL 測試 PASS；47 凍結來源全同，歷史清單亦經既有 M3 遷移對應核對。Sim/M3 全套回歸 NOT_RUN。[回放](components/rm27_sim/artifacts/asm2_handover/replays_fixed_camera/straight_contact_01/maneuver.gif) · [首觸與時序](components/rm27_sim/artifacts/asm2_handover/straight_contact_01/contact_analysis.json)。現行名稱見[術語表](docs/INTEGRATION.md#current-research-terminology--2026-11-13)；唯一進度／待辦／交接在 Non-RL 活動計劃 §0，歷史結果保留如下。

ASM2 R3 快照（2026-11-13）｜**名義連續任務PASS，穩健性NO-GO**。在原native scorer／physics／BNR／1g detector下，加入有限接觸片餘量、沿板上移8mm候選及參考窗口排程分離；兩個開發起點均合格首觸、無禁止接觸、分離並保持穩定1.005s。名義首觸3.7428s、首控3.745s、4.435s分離；與未提前分離對照的前750個完整物理狀態逐位相同。兩成功案IMU event=None，排程不是及時接觸偵測，亦未宣稱整面同時貼合。凍結後三個新起點×四方法，A固定／B固定／B線上／改進線上均0/3乾淨成功；分離依序1/3、2/3、2/3、2/3，未回頭調參。名義0.5°stress不是校準信心；7°stress使15/15採樣交接拒絕，本輪留出姿態RMS仍1.1–1.75°。8條估測初值native模型預測僅作離線診斷，完整native post-contact gate尚未接入30ms線上採用。24條完整試驗共111.8056s模擬／1481.12s driver wall；全倉630 PASS、5 skipped、4 subtests，2594份歷史／凍結來源未變。早期120s超時兩案保留，最終每案180s且總主機預算未擴；無RL、硬體、commit或push。[分析與限制](components/rm_uav_mujoco/artifacts/asm2_r3/analysis.md) · [四方法比較](components/rm_uav_mujoco/artifacts/asm2_r3/comparison.json) · [成功回放](components/rm_uav_mujoco/artifacts/asm2_r3/dev_final_nominal/native_replay.gif) · [匹配時序圖](components/rm_uav_mujoco/artifacts/asm2_r3/matched_contact_timeline.png) · [研究配置](components/rm_uav_mujoco/configs/asm2_recovery_planning.json)。詳細進度唯一保存在component原活動計劃§0；此段為派生結果快照。

ASM2 連續感測非 RL 整合快照（2026-11-13）｜**整合可執行，完整任務仍FAIL**。新增由連續IMU＋external p/v飛行歷史求姿態／世界框架對準，接入同一native飛機的handover、原online planner、1g detector、supervisor與BNR；runtime無GT，目標為登記基地板面地圖，非視覺辨識。最終名義fixed／online均合格首觸，online實際採用一次換軌，首觸至禁碰26.5→81.9ms、至命令133.4→90.3ms；換軌前593個完整native snapshots逐位相同，兩案均無乾淨恢復。名義首觸前p/v/姿態RMS=2.164mm／0.02162m/s／0.394°。同一sensor流的凍結R2.2-B姿態／平移valid為0/781，新估測541／536，但不等同校準信心。12開發＋4獨立留出完成；留出0/4合格首觸、1/4分離、0/4乾淨恢復。其中速度擾動案在不合格首觸及禁碰後，仍不重設地分離並保持運動學穩定1.465s、進入POSITION_HOLD，安全得分保持FAIL。其他留出暴露對準姿態誤差4.18／6.39°及模型擾動下對準前碰撞；未在heldout調參。較早排程分離與motor lead沒有避免有限板邊機身碰撞，反向切向參考不可行，未升為預設。covariance=null、trusted=false，僅明示synthetic simulation_execution；scorer/物理/原控制器與1g baseline保留。全倉627 PASS、5 skipped、4 subtests，2311份歷史／凍結來源hash未變；未RL、硬體、commit或push。[分析](components/rm_uav_mujoco/artifacts/asm2_integrated/analysis.md) · [量化對照](components/rm_uav_mujoco/artifacts/asm2_integrated/comparison.json) · [名義接觸回放](components/rm_uav_mujoco/artifacts/asm2_integrated/development_12_online/native_replay.gif) · [獨立穩定回放](components/rm_uav_mujoco/artifacts/asm2_integrated/heldout_velocity/native_recovery_replay.gif) · [使用入口](components/rm_uav_mujoco/README.md#asm2-連續感測接觸恢復與估測重放)

**研究定位澄清（2026-11-12）**。使用者將 RM27 定位為機器人自主系統研究，涵蓋無人機視覺感知、自主飛行仿真、VO／VIO、辨識追蹤、視覺伺服、運動規劃與強化學習。「裝甲板」是目標物名稱，「撞擊」描述指定相對姿態下的接觸任務。研究意圖與競賽來源分別記錄，術語定義見[研究定位](docs/INTEGRATION.md#research-positioning)。本次更正描述，既有 PARTIAL／NOT_RUN 依實際交付與測試證據保留。

研發工作入口已按 Onboard 的分工同步到總倉與七個現行元件：
`AGENTS.md` 導航，[SPEC](.agents/docs/SPEC.md) 管需求與驗收、
[DESIGN](.agents/docs/DESIGN.md) 管分工與接口、[ROADMAP](.agents/docs/ROADMAP.md)
管里程碑；當次進度仍在本機活動計畫 §0。這是文件結構更新，未新增能力驗收。

**全倉平台決定（2026-11-11）：RK3576 是唯一現行機載平台，M3C 停止使用。**
原 VIO／Tracking 雙平台並行決定已由此取代；VIO、SLAM、Tracking、Onboard 與
部署驗收以 RK3576 為目標。M3C 的分支、資料、標定、失敗及回放結果保留為歷史證據。
既有 RK3576 P1 的 detector 仍取自歷史 M3C Tracking 的 Linux 來源，未使用 M3C 板，
本輪未更換其建置依賴或重新驗收。正式決定見
[整合契約](docs/INTEGRATION.md#platform-decision--2026-11-11)。

ASM2 局部快照（2026-11-11）｜**ASM2 v1.0.0（142 g）** 已登記為最新共用飛機，
新 MuJoCo 工作選用 **v0.1.0 碰撞簡化版**。總倉及同一批九個本機元件／工作樹
由 `data/shared_aircraft/` 讀取單一份資產。已驗證發布包 15,475 個 payload 雜湊、
簡化版載入及其 3,522 個碰撞幾何、15 個撞觸片和四個旋翼。
共用資產只提供使用者上傳的簡化版 XML 及其引用網格。
本機同用 0.1 ms 物理步長，簡化版的懸停和平地接觸步進分別約快 3.27×、2.14×。
使用者交付的首觸回歸為 10/10 一致，其中倒置禁止接觸提早 0.5 ms。
碰撞後反彈、恢復及完整場地效能待驗證。Isaac 資產隨原發布包持有，
本地共用資產安裝 MuJoCo 模型。詳見 [ASM2 共用模型](data/shared_aircraft/README.md)。

ASM2 132.6 g 局部快照（2026-11-14）｜[獨立飛行配置 v0.1.0](data/aircraft_models/quad-132p6g/models/v0.1.0/README.md)
引用 ASM2 CAD 與 3,522 個碰撞幾何，未複製或縮放網格。CAD 馬達中心和日誌力臂的水平差最大 1.5 mm，
動力學保留日誌力臂。推力曲線與 26.7 ms 為日誌擬合，偏航比例為粗估，重心及慣量為占位。
本機 MuJoCo 3.12.0 無視窗驗證載入、地面接觸偵測、單馬達力矩、一階響應及 1 s 配平懸停；未校準接觸動態或驗證 SITL。
三份實飛日誌的五個留段（17.8 s）已完成無視窗開迴路重播，未重現實機多秒響應。兩個 Z 段機身比力 RMSE 約 0.27 m/s²，
但全五段的世界垂直加速度 RMSE 為 9.65–14.94 m/s²；只用實機姿態做診斷投影時降為 0.26–1.61 m/s²。
優先重查重心／分馬達推力和橫滾、俯仰配平，其次偏航配平；未重估模型。[留段重播報告](artifacts/quad-132p6g/2026-11-13_v0.1.0_holdout/report.md)。
逐顆 ESC 診斷（同五段）：只換 CAD 力臂消除兩個 Z 段約四分之一的俯仰力矩偏差；實測 RPM 改善合力，但分馬達推力與重心無法可靠分離，候選未採用。兩個正式模型均未修改。[ESC 診斷](artifacts/quad-132p6g/2026-11-13_v0.1.0_esc_pitch/report.md)。
固定原 ASM2 重心、逐件按 132.6/142 重算完整慣量是配重假設；在同五段用相同指令與日誌力臂，俯仰 RMSE 從 7.40 升為 27.30 rad/s；CAD 力臂及實測 RPM 配對同樣變差。未採用。[慣量假設診斷](artifacts/quad-132p6g/2026-11-14_v0.1.0_cad_inertia/report.md)。
142 g ASM2 的索引、預設 XML 與 plant metadata 雜湊不變。

ASM2 非 RL 局部工程對照（2026-11-11）｜完成固定參考與線上重規劃共 8 條配對試驗。
改變交接狀態後，線上方法採用新參考，路徑改變 54.6 mm。兩條合成失視案例均開始撤離並達到分離距離，
穩定保持未驗收。合格接觸及接觸後恢復均為 0/8。控制輸入為真值或合成估測，接觸事件由 MuJoCo 求解器提供。
完整場地及視覺閉環待驗證。[結果](components/rm_uav_mujoco/artifacts/asm2_contact_recovery/base_comparison_02/summary.json)

ASM2 跟蹤診斷（2026-11-11）｜原名義固定案例已逐點重現。速率回路未補償 40 ms 馬達響應，角速度要求也超過原命令上限。加入角加速度前饋與有界馬達補償後，無接觸配對的速度 RMS 誤差從 0.322 降至 0.021 m/s。原場景及評分的 8 條對照仍為 0/8 合格接觸及 0/8 恢復。名義案例實際接觸切向速度 0.284 m/s，參考材料點約 0.282 m/s，兩者均超過 0.25 m/s 門檻。線上 10 個候選因換軌角速或斜率超限而拒絕。兩份實際接觸後的獨立恢復試驗分別因角速度越界及計算時限結束，兩份種子的接觸均未合格。[跟蹤對照](components/rm_uav_mujoco/artifacts/asm2_contact_recovery/tracking_comparison_02/summary.json) · [獨立恢復](components/rm_uav_mujoco/artifacts/asm2_contact_recovery/tracking_detached_recovery_01/summary.json)

ASM2 接觸約束規劃（2026-11-11）｜原場景與評分下，固定及線上各 2/4 合格首觸，四例均用真值。名義首觸法向／切向速度 0.086／0.131 m/s，姿態誤差 2.73°。擾動交接線上實際採用新參考，0.25 s 後路徑差 35.9 mm。合成動力學配對均未通過，完整恢復仍為 0/8。逐步回放證實名義首觸後 0.6 ms 已碰到機身，早於首個恢復控制更新。獨立恢復可達 80 mm 分離及瞬時穩定，但前接觸片再接觸已違反門檻。全部使用局部場地與 MuJoCo 接觸事件。[配對結果](components/rm_uav_mujoco/artifacts/asm2_contact_recovery/contact_constraints_comparison_01/summary.json) · [板面側視回放](components/rm_uav_mujoco/artifacts/asm2_contact_recovery/contact_constraints_comparison_01/nominal_fixed/plate_side.gif)

ASM2 恢復感知規劃（2026-11-12）｜名義真值案例首次連續通過合格接觸、分離及 1 s 穩定保持，全程無重置或二次接觸。1.0782 s 由前片首觸，0.572 s 後分離 80 mm，2.750 s 完成，120.44 s 主機耗時。原恢復的匹配軌跡在首觸後 155.1 ms 因轉平再次觸板。兩次試驗的首觸與其後第一個控制取樣狀態逐位一致。擾動交接線上仍可採用新參考並合格首觸，但原恢復因早期禁碰失敗。成功組合的線上、擾動及感測器驗證待做。[名義成功回放](components/rm_uav_mujoco/artifacts/asm2_contact_recovery/recovery_aware_front_brief_release_01/nominal_fixed/plate_side.gif) · [線上配對](components/rm_uav_mujoco/artifacts/asm2_contact_recovery/recovery_aware_pair_01/summary.json)

ASM2 面貼合目標（2026-11-12）｜機體幾何中心軸與板中心法線同軸，前後合法接觸面盡量完整貼合。編譯模型的兩組底面共面，居中投影可落在已登記的有效區內。既有成功軌跡的首觸前軸線偏差為 3.632 mm、夾角 0.978°，首觸後已稽核的 21.8 ms 內前後組先後承載。面貼合尚未驗收。[模擬器規格](components/rm_uav_mujoco/.agents/docs/SPEC.md) · [原生接觸區間](components/rm_uav_mujoco/artifacts/asm2_contact_recovery/recovery_aware_front_brief_interval_01/summary.json)

ASM2 同軸參考比較（2026-11-12）｜原生靜態同軸姿態可雙組承載，最近禁碰淨空約 29 µm。B 居中參考的名義與小擾動固定／採用線上換軌三條均通過接觸、分離與恢復。B 名義實際首觸軸線偏差 6.27 mm、夾角 1.61°。C 整面參考及兩條修正都合格首觸並分離，二次接觸使恢復失敗。最佳角動量變化由 A 的 101.64 降到 75.39 µN·m·s，仍有二次接觸。動態雙組同時承載尚未驗收。[靜態幾何](components/rm_uav_mujoco/artifacts/asm2_contact_recovery/surface_alignment_native_geometry_01/summary.json) · [小擾動線上回放](components/rm_uav_mujoco/artifacts/asm2_contact_recovery/surface_alignment_physical_small_online_01/small_handover_online/plate_side.gif)

ASM2 R1 接觸穩健性（2026-11-12）｜156 個短接觸物理試驗顯示 47 µm 的模型雙組承載窗口對所測步長、摩擦及求解器時間尺度維持不變，改變接觸片相對高度後會變化。動態禁碰對摩擦和求解器時間尺度敏感。150 ms 的凍結配對篩選共 48 對，開發組 A／B 13/32、6/32，獨立驗證各 2/16，驗證差值 95% 配對區間 [−40.9, +40.9] 個百分點。證據不足以判定哪一種策略較穩健。完整恢復、合成延遲與真實感測器均未在 R1 驗收。[物理結果](components/rm_uav_mujoco/artifacts/asm2_contact_recovery/r1a_physics_01/summary.json) · [配對分析](components/rm_uav_mujoco/artifacts/asm2_r1/screening_01/paired_diagnostics.json)

ASM2 獨立恢復（2026-11-12）｜由 8 個原生接觸後快照各分支 3 種既有恢復，24 條皆完成。速度優先、短法向釋放、長法向釋放各有 0/8、3/8、0/8 通過安全分離與 1 s 穩定保持；兩條名義短釋放的完整恢復狀態與歷史逐位一致。兩個種子已帶入禁碰，之後達到運動學穩定也未取得安全驗收。未見電機指令飽和或主機逾時，步進佔耗時約 95–99%。使用 MuJoCo 自機及板面真值，證據只涵蓋選取的獨立恢復案例。[恢復圖](components/rm_uav_mujoco/artifacts/asm2_r1/independent_recovery_01/recoverability.png) · [分析](components/rm_uav_mujoco/artifacts/asm2_r1/independent_recovery_01/analysis.json)

ASM2 R2 獨立低能量試驗台（2026-11-12）｜普通 18° 斜面與凍結 ASM2，12 個留出狀態共 84 條配對分支。0／5／10／20 ms 求解器事件配真值各 1/12 安全恢復，5 ms 求解器事件配合成輔助估測亦 1/12，通過的是同一狀態。合成 IMU 檢出 10/12，所有已檢出分支在首次恢復指令前已禁碰，唯一可恢復狀態未檢出。IMU 加合成輔助估測和 IMU-only 均為 0/12。合成平移及初始姿態對準均為建模假設，真實感測器和恢復感知規劃未執行。[留出分析](components/rm_uav_mujoco/artifacts/asm2_r2/validation_01/analysis.json)

ASM2 R2 時序與偵測審核（2026-11-12）｜五個既有中性試驗台案例經0.1 ms原生重播，完整狀態與接觸歷史一致。零延遲真值事件的12案有6案在首筆恢復命令前禁碰，5案於命令後違規，1案通過。預選近成功失敗在命令後226.3 ms再接觸，與首控前0.7 ms內禁碰的案例分開。IMU-only全部未發恢復命令，10案缺平移、2案漏檢。重建合成IMU的獨立新留出12案中，原1g檢出8、研究候選0.5g檢出12，但候選10次在首次禁碰後觸發，不能宣稱已辨識首次輕觸或及時恢復。無接觸留出僅8s未見誤報，0.25g已在正常推力變化誤報。正式門檻、控制器與物理未變，外部p/v及初始姿態仍為合成假設。聚焦20項測試PASS，可靠無特權恢復仍NO-GO。[時間軸](components/rm_uav_mujoco/artifacts/asm2_r2/timeline_02/summary.json) · [偵測結果](components/rm_uav_mujoco/artifacts/asm2_r2/detection_01/results.json)

ASM2 R2 再接觸因果診斷（2026-11-12）｜固定流程下，10個有禁碰期限的新留出案中7案即使首觸立即偵測也來不及經2.5ms傳輸與5ms控制時鐘發命令。development01的COM持續離板，但固定機身材料點於123.6ms起因旋轉接近平面。重建指出接觸前延遲速度被凍結為恢復參考初值，降低早期推力需求。單一配對反事實中，原分支逐位重現251.3ms禁碰，只替換一次真值速度初值、保留後續合成估測的分支無再接觸並完成1s保持。這建立本案因果貢獻，沒有形成可部署修正或無特權恢復資格。控制器、門檻、物理、評分器及接近規劃保持不變。21項聚焦測試PASS。[禁碰前可觀測性](components/rm_uav_mujoco/artifacts/asm2_r2/observability_01/results.json) · [因果拆解](components/rm_uav_mujoco/artifacts/asm2_r2/return_diagnosis_01/causal_analysis.json) · [單因素配對](components/rm_uav_mujoco/artifacts/asm2_r2/anchor_counterfactual_01/results.json)

ASM2 R2 事件適用初始化有限修正（2026-11-12）｜特定缺陷修正PASS，一般可靠恢復仍NO-GO。新參考分列量測年齡與事件前／歧義／事件後適用性；等待期間只用明示且有限的原法向分離模型，收到適用合成外部速度後於當下採用，初值影響有限且不倒填時間。development01兩種事件分支由再接觸改為乾淨分離及1s保持，validation06求解器事件能力保留、IMU仍漏檢。採用瞬間同狀態CTBR跳變0。凍結後12個新物理狀態48分支，兩版×兩種事件均0/12；求解器各9案首控前／3案首控後禁碰，IMU各12案首控前禁碰（4漏檢）。三案首控後禁碰均早於適用初值可影響下一控制更新，不能宣稱留出恢復率提升。56條總預算完成，留出26.23s無超時；全倉528 PASS、5 skipped、4 subtests。控制器、1g門檻、物理與scorer不變，未用真值速度修正，外部p/v及初始姿態仍為合成假設；未推進規劃、訓練、硬體或Git發布。[開發配對](components/rm_uav_mujoco/artifacts/asm2_r2/initialization_01/development_gate_metrics.json) · [凍結留出](components/rm_uav_mujoco/artifacts/asm2_r2/initialization_01/heldout_v1/summary.json)

ASM2 R2.1 接觸可觀測性與狀態估測（2026-11-13）｜離線診斷PASS，可靠感測恢復仍NO-GO。command-aware residual使用issued collective、原40ms電機模型及同階段濾波，不讀即時真值activation。固定1g檢出未改善（診斷開發12/14，已看過歷史detection留出8/12、10個有限禁碰期限前有效命令0/10）；0.25g描述性無接觸誤報開發2/4→0/4，但歷史留出仍2/4，raw1g在8/8無接觸片段誤報，未改正式門檻。102條合成分支審計確認age-valid≠post-impact，原及新留出求解器組首控各12/12尚無post-contact平移樣本；candidate明示模型過渡而非採用舊速度。56條統一時序分列原生首觸、IMU、送達、外部量測、首控、motor response、禁碰與獨立再接觸；新留出IMU各12/12在首控前已禁碰。無新增物理／感測重建，58項聚焦測試通過、876個歷史證據檔未變。R2.2僅建議同步完整記錄與獨立估測完整性驗證，未自動開跑；現有外部p/v和初始姿態仍合成、沒有實際sensor可靠性證據，試驗台並非原基地裝甲板。[命令殘差診斷](components/rm_uav_mujoco/artifacts/asm2_r21/command_residual_01/results.json) · [狀態輸入審計](components/rm_uav_mujoco/artifacts/asm2_r21/state_contract_01/results.json) · [統一時序](components/rm_uav_mujoco/artifacts/asm2_r21/timing_02/results.json)

ASM2 R2.2 狀態估測完整性（2026-11-13）｜時間／能力契約與離線審計PASS，無特權撞後估測資格仍NO-GO。56條歷史合成分支18785次query中18213次可核對p/v，36426項誤差重算一致、clock mismatch0；位置已傳播到query但速度仍停在acquisition，未保存完整估測R/omega、generation、covariance／innovation。development01首控25ms使用0ms取得、模型15ms送達的速度，首觸15.1ms；query速度誤差0.16195m/s，取得時刻誤差0.02243m/s，兩者不得混作感測精度。新增純optional supervisor驗證七個估測／六個恢復能力模式、事件區間epoch、同控制時刻對齊、GT拒絕與失效撤銷，不發命令、不接入原恢復。最終相關133項測試通過；0新增研究物理軌跡／感測重建（既有回歸可執行native步）。Faessler2015與Liu/Karydis2021原文已審查；Li2020書目核對但原文方法BLOCKED，未宣稱reachability或移植新控制器。新結果只寫R2.2；12份凍結來源及912份歷史證據hash複核不變，初次新增3個pyc快取已單獨清除並保留失敗核查，最終舊根無新檔。歷史接觸／恢復率不因介面審核改變。沒有真實sensor／VIO資格，不自動進入R2.3、規劃、訓練或硬體，未commit／push。[分析](components/rm_uav_mujoco/artifacts/asm2_r22/analysis_01.md) · [同步時間圖](components/rm_uav_mujoco/artifacts/asm2_r22/stream_timeline_01.png) · [模式契約](components/rm_uav_mujoco/.agents/docs/ASM2_ESTIMATION_MODES.md)


ASM2 R2.2-B 感測驅動撞後估測（2026-11-13）｜runtime與時間完整性PASS，完整撞後狀態資格FAIL，R2.3 NO-GO。重用未改M3 predict，以合成IMU傳播及external p/v取得時刻互補修正，不讀runtime真值；六條封存物理軌跡各350ms、獨立sensor種子，21000 native重放步及420端點逐位一致，新增研究物理軌跡0。這是retrospective sensor-heldout，非新物理留出。12最終配對852 query皆未取得非GT初始姿態／世界框架對準，姿態與p/v valid均0；輔助p/v數值改善只屬明示框架假設下診斷，不能當合格世界精度。body-rate health-valid仍有撞後誤差，三留出安全窗RMS約1.12–1.34rad/s。四留出皆先禁碰後detector acquisition；開發validation06於350ms內漏檢。covariance=null，NEES/NIS不可用，未創造confidence。被動supervisor不發命令，planner/scorer/physics/1g門檻及Recovery Controller凍結；620 PASS、5 skipped、4 subtests，16來源及961歷史hash未變。未做感測閉環恢復、真實sensor/VIO、R2.3、RL、硬體或Git發布。[分析](components/rm_uav_mujoco/artifacts/asm2_r22b/analysis_01.md) · [估測指標](components/rm_uav_mujoco/artifacts/asm2_r22b/evaluation_02/metrics.json) · [同步時間圖](components/rm_uav_mujoco/artifacts/asm2_r22b/synchronized_timeline_01.png) · [保存核查](components/rm_uav_mujoco/artifacts/asm2_r22b/preservation_audit_01.json)

ONB-NAV-01 非接觸視覺航向研究（2026-11-12）｜PARTIAL。Onboard 新增 PC live／replay 組裝入口，重用既有 Sim mounted camera、C++ detector、pixel-heading、PX4 SITL 和 MuJoCo 馬達物理。定位沿用 GT-generated ODOMETRY 經 PX4，與影像航向修正分開宣告。名義場景誤差收斂至0.0971°且保持5.004 s；側移／1 s blackout場景重獲0.256 s，黑畫面接受0、失效yaw變更0。298／424幀錄製各有282／407筆真detector回放一致。RK3576正式P1僅作10 s被動720p短測，取像29.80、有效處理15.69 FPS、峰溫40.692°C，取樣均LOST，沒有本輪真實目標資格。PC約0.63× real-time，純感測定位、板端servo、跨機閉環與完整控制歷史回放仍待完成。正式P1 payload與current身份核對未變。[使用入口](onboard/README.md#onb-nav-01-非接觸航向對準) · [狀態](onboard/.agents/docs/ROADMAP.md) · [本輪證據](onboard/.agents/runs/onb-nav-01/)

Onboard 局部快照（2026-11-12）｜ONB-P0／P1 ACCEPTED。
`onboard-b5f24de264ad6e8980579e2e` 已安裝於 YR-rk3576 的 `/opt/rm27`。
P1 通過 1080p UVC、綠燈像素追蹤與遮擋重獲、跨機 PX4 SITL 監看、錄製回放和
故障隔離。10 分鐘平均處理 6.40 fps，板端 MAVLink 送出 0。
命令與驗收狀態見 [Onboard](onboard/README.md) 和
[ROADMAP](onboard/.agents/docs/ROADMAP.md)。
ONB-PERF-02 短窗已量測 A～E。暫時風扇下，以固定 EuRoC 配對資料的 0.5×
OpenVINS 回放加入並行後，首次出現 VIO 排程超期，實拍處理從 A 的 15.82 FPS
降至 C 的 12.06 FPS。E 的 PPO 200 Hz 回放無超期，D/E 觀察到 SITL 遙測中斷。
無風扇 A 在 116.06 秒達 70.23°C 並由守門停止。1× VIO 與持續機載散熱待驗證。
ONB-PERF-03 已完成局部效率診斷，未修改 production 或替換 release。
RK3576 同一錄製前 24 幀的 P1→PERF-01 processing 為 4.80→6.12 FPS；
主要成本仍為候選外觀驗證、積分影像與粗候選排名。四組回放像素／觀測一致，
固定時間戳完整非耗時輸出 72/72 一致，PC CTest 13/13 PASS。沒有已驗證的新優化，
本輪短測不代表熱穩態或最終感測器性能；詳見 Onboard ROADMAP。

ONB-PERF-06 量到原生 720p 開發吞吐，40 cm、5100 K 可取得有效像素觀測。
同一錄製從 1080p 縮到 720p 後未確認目標，識別仍 PARTIAL。
EuRoC 原生與縮放 1× 均有 50 ms 超期。風扇下 32 秒 A～E 已量測，
全載感知 14.55 FPS，PPO 200 Hz 無 5 ms 超期，峰溫 48.08°C。
正式 P1 release 的檔案已重新校驗。識別真值、1× deadline 與最終散熱仍待驗收。

ONB-VIO-02（2026-11-12）以 OpenVINS `6948812` 完成 28 個 RK3576 EuRoC 1× 短窗。
保留 200 目標特徵和 4 OpenCV threads。隔離候選 `6f58c0bc…` 顯式略過 headless
視覺化重三角化，Easy 兩輪的 600 幀超期從 110／119 降為 39／45，Difficult 從 99 降為 43。
姿態與逐幀校正完全一致。真實相機／IMU、1× 期限及機載並行資格仍待驗收。
[Onboard ROADMAP](onboard/.agents/docs/ROADMAP.md)

ONB-PERCEPTION-02（2026-11-14）為PARTIAL。原生720p共298幀在RK3576精確回放，
292筆live觀測一致。無錄製USB短窗capture29.818／有效處理16.691 FPS，峰39.769°C。
隔離診斷ARM與原P1 detector82/82非耗時結果一致，68次全畫面與14次ROI的候選收集
P50為59.587／5.545ms。22個AI暫定可見樣本中17筆零候選、1筆未確認、4筆確認，
所有標註bbox均在multiscale搜尋矩形內；內部filter拒絕原因仍待定位。
PC及ARM派生影像狀態回歸通過。22 幀的離線互動覆核頁、原始 JPEG 與覆核圖已備，使用者回報覆核完成，
正式 PR／定位品質待匯出 CSV。RK3576 原生 720p Auto WB 錄製移動綠燈 60.084 s、1,281 幀，
19 幀人工覆核已計分。16 幀等時抽樣均可見且全畫面搜尋零候選，Precision 未定義，Recall／F1 為 0。
3 幀確認檢出的補充樣本中心誤差 0.98–1.72 px，IoU 0.933–0.990。
PC 使用原 P1 封存源碼重跑，與既有 RK3576 輸出 1,281/1,281 幀非計時一致。16 幀框內精算通過的
1,560 個峰值全部被外觀驗證拒絕，15 幀最常見拒因為局部綠色支撐不足。
10／15／20／25 m 和真實遮擋資格為 NOT_RUN。
[Onboard ROADMAP](onboard/.agents/docs/ROADMAP.md)

ONB-ARCH-02 Completion（2026-11-12）軟體 Gates A～J 為 ACCEPTED。
既有監看加入 typed Vehicle／Perception／VIO／Telemetry、有界 State／Observation、
Navigation／Policy 接口、被動仲裁及容量受限事件重建。host 195 PASS加真實producer
組合1 PASS，Tracking48 PASS／60 subtests，P0 CTest1/1；RK3576先前完整176 PASS，
最終受影響60 PASS加真實producer組合1 PASS。候選 `onboard-270b7715312332f3d852a3ed`
在隔離目錄安裝23 payload，正式 P1 的17 payload／manifest及current仍相符。
原生720p USB短測capture29.795／effective15.001 FPS、processing P95 63.258ms、
峰溫38.846°C，取樣LOST；25來源／53衍生事件確定性重建PASS。
新增序列化與健康資料有CPU／RSS／紀錄量成本，沒有性能改善結論。
OpenVINS native JPL／frame轉換、最終校準、可信live VIO、新SITL接線及飛行資格
仍未完成。實際RM27 PPO未訓練，本輪沒有模型依賴或控制出口。
來源為根HEAD `651f8a2` 加final manifest及source-inputs封存的dirty程式。
[Onboard ROADMAP](onboard/.agents/docs/ROADMAP.md)

Tracking 元件現名為 `rm27_tracking_rk3576`；舊名與舊路徑在下文歷史紀錄中保留。
本輪更名、驗證及發布狀態見末節。

最新 Sim 局部快照：2026-11-08，America/New_York。已補回使用者要求的完整視覺流程：
機庫承托面起飛、上升設定 1.3 m、原始 RGB 辨識／航向閉環、綠燈前約 1 m 懸停。
前輪固定航點替代不能算這項任務完成；新飛行與回放 PASS，末端停距 1.012175 m、
連續合格 5.004 s，詳見 §9。原辨識器、速度、PX4 參數及原場地保持不變；其餘章節保留各自驗證日期。

最新局部快照：2026-11-05（本次更新RL §6；其他章节保留原验证日期），RL 新增 working PPO 已通過原獨立flight scorer Hover19/19、Shift4/4，兩候選589824controls/1695.478s後停止；見§6。完成 3D＋左上角 FPV／舊機庫頂面起飛驗證後，新增左右運動與一秒黑畫面的 tracking／PX4 閉環及被動對照；四段共用真實影片的原生辨識結果也已完成 45,462 筆離線航向輸出，方向／限速／失效保持及完整重跑一致性 PASS，明確使用假設相機與固定參考姿態，未傳輸到飛控。現已補上四片完整追蹤／航向曲線影片，媒體解碼與雜湊核對 PASS，見 §9。實際飛機指令正確性仍未驗證，識別準確率仍缺人工真值。新機庫待使用者提供；仿真結果仍用合成定位與未標定模型，見 §9。RL 原β1與新β=.1 Stage 1-A seed0比較均完成，正式final皆0/19；新run131072controls／1523.502s，角速失敗轉為越界但未學會受控接觸，見 §6。新增独立非接触flight probe A/B也已完成：各65536controls，均0/23，传统控制5/5；见§6。後續C僅加12/s早停剩餘成本，外層143中斷後用最後完整125更新完成23例評估，仍0/23；速度與裁切改善、位置控制未學會，未補訓，見§6。新增C final只讀PD對照：零誤差仍有推力/角速偏置，對稱水平誤差方向不正確；未發現座標或normalizer載入錯誤，0新物理/訓練，見§6。其他章節保留各自原驗證日期。

同日完成 M3C 視覺品質第一階段的工具與資料完整性 PASS：128 張均勻原生樣本＋4 張診斷案例，
逐像素核對原片，AI 初標與兩次評估結果一致。暫定均勻樣本為 100 可見／24 不在畫面／4 不確定；
可見樣本 82 個確認位置相符、12 無候選、6 有候選未確認，不在畫面樣本有 5 次確認輸出。
使用者隨後選擇略過人工覆核（NOT_RUN），不再等待標註；正式品質評估仍因缺獨立真值而 BLOCKED，詳見 §9。

## M3C 載板共用資料（2026-11-03，歷史參考）

使用者提供的 `/home/shiuhou/Projects/Maixcam2_m3cboard/` 已集中到
`data/hardware/maixcam2_m3cboard/raw/`，原入口保留為相對符號連結。
共 11 個原始檔、207,345,690 bytes，包含 M3C 378C／MaixCAM2 379C 的原理圖與
3D／機構檔案、另版 `MaixCAM2_3.0.step`、`BL-M8800DS2.pdf` 及追加的 ICM-42688-P 規格書。
來源分組按檔名記錄，未以這次整理推定手上載板修訂或實板接線。

總倉、七個 components、M3C Tracking 及獨立 M3C VIO 工作樹共十個 checkout，
都提供 `data/shared_m3c_board/`。各自 README／AGENTS 已補上取用入口；
[共用索引](data/shared_m3c_board/README.md) 與 manifest 記錄檔名、大小、完整 SHA-256
及所有入口。原始資料僅存一份，raw 已由 owning repo 忽略；獨立 clone 仍需另取資料。

初次整理已驗證逐檔搬移前後 SHA-256，並經十個入口實際讀取原十個檔案；原入口亦核對。
這是資料完整性與存取結果，不代表載板、IMU 接線、相機同步或 RK3576 相容已驗收。
沒有修改來源內容、共用工程規範、研究算法或硬體設定，沒有 commit／push。

2026-11-03 追加的 ICM-42688-P 規格書為 DS-000347 Rev 1.2（2020-04-19，109 頁；
首頁標示 pre-production），保存於 `raw/ICM-42688-P/ICM-42688-P_DS-000347_Rev1.2.pdf`。
附件／副本 SHA-256 為 `4d7c04e3e86f37defe27c20a82b9fcd9d4ddbd3b9d68a525c2d49595d52cf97a`；
更新共用索引及 manifest，原 10 檔保持不變；追加檔案已經十個共用入口讀取並核對 SHA-256。
規格書不建立目前載板接線、DRDY／FSYNC
或相機同步的實測結論；本輪沒有操作硬體。
共用索引也已補上既有自研板 SPI1 接線與 `imu_icm42688` 採集程式入口；
目前實板／部署對應、DRDY／FSYNC 與採樣時間驗證仍待完成。

## 新 PDF 對齊與跨倉接口批次（2026-10-30）

**本輪判定：`INTERFACE_INTEGRATION_PASS`。** 研究方向依使用者指定的
《RM27 无人机组开发文档 (2).pdf》，SHA-256
`d4526f85248623ada0a7c64e120184c8b6838dae215593ea034ee6e8825c437e`。
正式目標、責任、接口及命令在 `docs/INTEGRATION.md`，共同驗收在
`docs/MILESTONES.md`；各 component README 保留自己的接口及能力邊界。

六個獨立 JSONL worker 使用各自工作目錄與 Python 環境，實際呼叫 native 型別：
VIO／SLAM → PX4 ODOMETRY 編解碼、本方方位目標 → UAVGuidance、
Sim 狀態 → 未訓練固定種子 RL 推理 → Sim 馬達映射／PX4 CTBR 封包，
以及合成恢復事件 → 控制權撤銷與定位重新准入。未改 reward、未執行 learn，
未啟動定位後端、飛控固件或硬體。本方目標適配在總倉，Tracking 上游未修改。

| 本輪實際執行 | 結果／範圍 |
|---|---|
| 原工作區跨進程驗收 | 35/35 檢查通過；含非零姿態 NWU/FLU→NED/FRD、時效／尺度／缺項拒絕、epoch、控制交接、實際 worker 退出 |
| 接口 pytest | 7 passed；另含 SIGSTOP 斷流逾時、缺映射、舊 epoch 指令及定位斷流後新指令拒絕 |
| 相關既有回歸 | PX4 定位／時鐘 46、VIO schema/hardening 44、SLAM schema/hardening 44、RL Contact env 8、總倉 guidance/schema 10 passed；各 repo 分開計數 |
| 獨立目錄重建 | `/tmp/rm27-interface-rebuild-w1_7w960`；空 UV 快取、六個全新環境、`UV_OFFLINE=1` 與 `--no-index` 安裝 |
| 重建後驗收 | 同樣 35/35、pytest 7 passed；samples、checks、配置／來源身份完全一致；載入路徑與 editable 安裝均不依賴原工作區 |

相容環境限定本次 Linux x86_64、CPython 3.12.3、uv 0.13.1。Sim 使用
MuJoCo 3.17.0；RL 與 PX4 適配路徑使用 3.12.0，RL 為 Torch 2.14.1+cpu／
SB3 2.9.0，PX4 為 pymavlink 2.4.49。其餘依賴逐 repo 鎖定並附 wheel SHA-256。
完整來源快照採白名單：106 個來源／配置／文件、35 個外部 wheels；沒有原始錄影、
憑證、大型策略權重或完整場地 meshes。Ver2／場地 XML 僅作身份核對，
RL 介面使用歷史 Contact proxy；不宣稱兩個 MuJoCo 版本或不同機型動力學等價。

本地交付位置（相對總倉）：
- 配置／來源鎖：`configs/interface_exchange.json`、`configs/interface_sources.json`。
- 原始驗收：`.agents/runs/pdf-interface-alignment/validated/summary.json`。
- 重建結果／比較：同目錄 `rebuilt-summary.json`、`reconstruction.json`。
- 可重建包：同目錄 `local-rebuild/`；`manifest.json` 記錄 HEAD、父倉 gitlink、dirty 狀態、檔案 hash、依賴及工具，`README.txt` 提供離線重跑命令。

本輪 config＋source-lock 身份：`0838cdd7d0c490c9fdec7ce5bf7c0787be901c33168f91c7e90ea8ff4be29b27`。
只以 HEAD 無法重現本地未提交實作，重建使用打包來源；不改 gitlink，不以遠端 main
替換本地程式。保留使用者既有及同期修改，沒有 Git commit／push。
下載依賴時曾遇到主機 ROS 路徑污染及 PyPI 超時；依賴查詢已隔離，改為只收必要
接口依賴並逐 wheel 保存，最終離線重建成功，沒有用放寬驗收來掩蓋失敗。

**仍為 NOT_RUN：** 真實 VIO／SLAM 運行與精度、完整飛行／反覆接觸、物理恢復、
裁判判傷、新 PX4 SITL 飛行及 RK3576 性能。這是該接口批次的範圍；後續另行授權的有限 PX4 定點起降實測見 §9，不改寫該批歷史。各 repo 算法與硬體實驗仍由使用者另行安排。
下文保留各研究線歷史結果，不能將那些成功合計為新整機能力。

研發快照日期：2026-11-02（America/New_York）；本次更新第6節 RL E1-A，保留E0證據及開頭接口批次10月30日範圍。本報告是可單獨分享的跨倉背景與結果，
不是第二份任務計畫；唯一活動計畫在總倉 `.agents/plans/plan_pdf_interface_alignment_01.md`。

## 1. 先讀這一頁

現行研究目標（2026-11-11 使用者澄清）是**無人機透過視覺辨識定位目標裝甲板，以相對板面的指定姿態用機腹接觸片受控接觸，接觸後分離並恢復穩定飛行與狀態估測，再以視覺重新辨識目標**。綠燈提供定位線索，實體接觸面是登記的裝甲板。單次接觸與恢復先分開驗收，同架次重複接觸是後續整合里程碑。完整 RK3576 視覺閉環尚待驗證。下列歷史結果依原機型、觀測來源及驗收範圍解讀。

目前不是一套已完成真機驗證的系統，而是多條研發線各自取得有限證據：

| 研發線 | 已有能力／結果 | 還不能宣稱 |
|---|---|---|
| 跨倉接口與本地重建 | 六個 native worker、35 項整合檢查及隔離重跑通過 | 真實估計器、閉環飛行、判傷或 RK3576 部署 |
| 新場地 + Ver2 仿真 | 零推力起步、原生機庫起飛、完整場地繞行，到裝甲前保持前飛的交接狀態；三個名義速度通過 | 已在新場地完成指定姿態接觸、恢復、返航與落庫 |
| 歷史局部接觸與恢復 | 真值輔助的局部試驗台上連續起飛、接觸、恢復、返回、降落與停轉，軟／硬兩組共 140/140 | ASM2 視覺閉環、競賽原生機庫與完整場地通過，或真機可靠性 |
| RL | 有自主 CTBR 學習、到達環境、PPO/SAC 與多輪對照研究；部分舊任務配置成功 | 新 Ver2 真實接觸任務已訓練完成；所有 seed 都穩健 |
| PX4 | 真實 PX4 v1.17 固件 SITL；舊代理機型完整圖像引導任務；Ver2 短程起降任務各有成功例 | Ver2 在新場地完整任務、真實 VIO 或實機部署已完成 |
| VIO | OpenVINS 路線、資料鏈路與時間診斷、可重現 PC 候選 | 相機–IMU 時序、標定和完整定位驗收已過 |
| SLAM | 獨立單目起步包、主機資料集路線與板端前端測量 | 板端完整 SLAM 實時、穩定尺度與飛行定位 |
| rm27_tracking_rk3576 | 本方通用採集／預處理／回放與 S2 主機驗證紀錄 | RK3576 實板、目標定位／控制、已納入六-worker 整合驗收 |
| Sim 相機資料鏈 | 完整 2027 原始 RGB／有界回放；PX4 定點起降與原生綠燈 RGB 航向／地圖輔助一公尺懸停；四段真實影片回歸 | 真實相機時序、單目公尺測距、VIO、RK3576 或實機驗收 |
| Tracking 上游 | 綠燈／裝甲管線、180 fps 全視野採集與估計輸出測量 | 180 Hz 有效目標觀測、帶真值準確率、可安全控制 |
| MJCF | 飛機／機庫／場地資產、碰撞重建、可動基地 | 可見幾何全部具有經驗證碰撞，或材料參數已實測 |

**最重要的閱讀原則：不同飛機、場地、控制器、導航輸入、驗收門檻的成功不能相加成完整能力。** 仿真真值不是相機定位；錄製輸入精確回放不是另一次獨立閉環飛行；動畫不是物理驗證。

## 2. 證據標記與本次整理範圍

- **本次核對**：讀取現有檔案、Git 狀態或結果 JSON；除開頭明列的本批接口／回歸外，沒有重跑該歷史實驗。
- **既有紀錄**：原報告或活動計畫記載的結果，本次整理引用；不是本次重新驗證。
- **工程假設**：為建模或研究指定的值，不是實測。
- **未完成／未知**：沒有相應證據，或原驗收仍未通過。

此處「本批」指10月30日跨進程接口、相關回歸及隔離重建；該批未訓練、未執行飛行、未刷固件、未寫真機參數。第6節11月2日E1-A為另行授權的最小策略更新。下文各研究線原有測試數量仍屬其歷史版本；接口批次重新執行結果單列於開頭，不與歷史結果合計。

## 3. 倉庫架構與在哪裡工作

工作區根目錄為 `/home/shiuhou/Projects/rm27_drones`。以下相對路徑都以此為基準。

| 本地位置 | GitHub repository | 職責 |
|---|---|---|
| 根目錄 | `shiuhou/rm27_drones` | 跨組件契約、版本釘選、需求、整合與歷史研究 |
| `components/rm27_RL/` | `shiuhou/rm27_RL` | 訓練環境封裝、reward、policy、訓練／評估、RL Docker |
| `components/rm27_non_rl/` | 總倉擁有的本地 component；未指定獨立遠端 | RM27 Non-RL 傳統規劃、引導、估測與恢復 |
| `components/rm27_sim/` | `shiuhou/rm27_sim` | 共用原生物理、場景、合成感測、scorer、viewer 與回放 |
| `components/rm27_drone_px4/` | `shiuhou/rm27_drone_px4` | PX4 固件介面、SITL、估計器與飛行任務整合 |
| `components/rm27_vio/` | `shiuhou/rm27_vio` | OpenVINS、相機／IMU、時序、定位驗證 |
| `components/rm27_slam/` | `shiuhou/rm27_slam` | 局部地圖、失鎖重定位及地圖輔助重新搜尋；保留單目研究基線 |
| `components/rm_mjcf/` | `CosmosMount/rm_mjcf` | 可重用 MJCF 與 runtime meshes／場地資產 |
| `components/rm27_tracking_rk3576/` | `shiuhou/rm27_tracking_rk3576` | 本方通用影像採集、預處理、時鐘來源與回放；獨立於舊上游 |
| `rm27/perception/vision/rm27_tracking_m3c/` | `blademaster679/maixcam2_dart_vision` | 歷史 Tracking 來源、回放及既有 P1 Linux detector 的建置來源 |

`components/` 包含獨立版本庫及目前由總倉擁有的 `rm27_non_rl`。`rm27/` 是主倉庫自己的整合命名空間，並保留歷史 vendor、baseline 與兼容入口。兩者不是兩套相同的主倉庫。

2026-11-13 起，共用場景、原生模擬與 viewer 的現行實作位置為 `components/rm27_sim/rm27_sim/`，傳統接觸規劃與恢復研究歸 `components/rm27_non_rl/`。根目錄部分 `rm27` 模組是兼容轉接、部分 config 是 symlink；不要再在根目錄複製仿真實作。舊路徑 symlink 及歷史接觸研究保有原契約與重現用途。

共用物理套件在 `components/rm27_sim/packages/physics/`，import 名為 `rm27_contact_physics`。它不依賴 Non-RL、RL、Gymnasium 或 Torch；兩條研究線消費共用物理接口。RL 專用 Docker 留在 RL；本地已有 `rm27-contact-rl:i4-arrival-cpu` 和拆分驗證使用的 `rm27-contact-rl:sim-separated-20261021` 紀錄，鏡像名稱本身不保證另一部機器可直接取得。

其他主倉庫直接 submodules 包括 ArduPilot、ardupilot_gazebo、舊 RMUC2026 場地，以及 legacy MuJoCo／凍結 ArduPilot baseline。`.gitmodules` 將後兩者指向 `rm27_drones` 的 `components/mujoco-uav`、`components/mujoco-uav-ardupilot` 分支；它們不是目前六條獨立 components 的替代品。保留凍結 G1，不把新場地／新控制器結果寫回其原有聲明。

`baselines/` 是研發線交接與證據索引，不是第二份源碼。部分 handoff 內容較舊，需與實作倉庫最新需求／計畫對照。

## 4. 主倉庫任務與硬體基線

### 4.1 機型與場地

既有 Ver2 場景使用 **Ver2，使用者報告含電池 150 g**；CAD 約 114 × 114 × 45 mm，選定 5-series frames，另加 30 × 65 × 5 mm 核心板包絡。舊研究常用 **180 g、120 × 120 mm proxy**；PX4 的舊 150 g proxy 也不等同 Ver2。

目前開發場地由 `configs/field.json` 釘選 `components/rm_mjcf/rmuc2027_moving_base_mjcf/model.xml`，資產 commit `d9439a97cee80baba9d13a1de9882f2d8095fca5`，預設滑台居中。新場景用 `configs/scene_v2.json`；`configs/scene.json` 留作歷史 180 g／RMUC2026 顯式重現。

機庫原始來源是 `data/references/cad_raw/dock.STEP`。使用者已退役舊整場 `RMUC 2025复活赛&全国赛_asm.stp`。`base2.STEP` 為獨立基地組件，其目前匯出僅視覺審核，不自動具有碰撞或正式目標語義。

### 4.2 已知與未知

歷史硬體文件列出 1103 9600KV、2 吋槳、MicoAir 743v2 AIO 35A、MTF02P、M10G5883（歷史配置；使用者於 2026-10-31 明確排除 GNSS 與羅盤導航）、M3C + OS04A10。這是舊配置，不是 RK3576 新整機或本次拆機確認；新載板、雙相機、獨立 IMU、飛控 MCU 及整機參數仍待驗證。

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

### 5.2 歷史局部接觸與恢復

獨立 impact study 曾發現區間取樣錯誤：先 `mj_forward` 再取接觸／加速度會讀到重算的端點解，出現撞前約 39 g 假象。修正為在 `mj_step` 後、forward 前讀取實際積分區間；因此舊數字不能直接和新數字混合。

既有修正研究重建 27,600 次接觸與 33,600 次動力敏感度實驗：

- 廣分布 soft 3910/4800、hard 3964/4800；I4 子集 soft 2367/2400、hard 2364/2400。分布不同，不能挑較高者代表所有工況。
- 名義 proxy 與「近似 PX4」內環差距很大；後者甚至無接觸自由飛行也失敗，且推力映射造成實際推重比不同。這不是 PX4 真固件性能比較，也未隔離單一原因。
- 新 seed 的 proxy soft 543/600、hard 549/600；近似 PX4 為 10/600、8/600。水平姿態 3 m/s 是較差單元。
- 理想輸入下 filtered 1 g 接觸檢測 soft 600/600、無誤報、最大延遲 3.5 ms；raw 1 g 有 15/600 誤報。仍不是實測 IMU 部署結果。

`mission_v2` 在局部有中央凹槽的試驗台上，連續起飛→接觸→恢復→返回→降落→停轉：soft/hard 各 20/20 名義重啟 + 50/50 物理擾動，共 140/140。每組 50/50 的 Wilson 95% 下界約 92.86%。軟硬使用相同 50 個 seed，不能算 100 個獨立物理樣本。初始停放姿態固定。

原 `mission_v1` 擾動僅各 22/50，失敗仍保留。改進沒有把舊失敗改成通過。局部凹槽平台是容納突出底球的工程假設；局部完整任務不等於新完整場地任務。

### 5.3 下一個整合缺口

先保存 1 m/s 交接的完整物理／馬達／控制器狀態，接入完整場地已登記的裝甲接觸面，完成末端對準與指定首次接觸；不能用局部 plant reset 偷換初態。然後接脫離、三層恢復、重新取得目標與同架次再次接觸，再做獨立可靠性驗證。返庫落位保留為歷史研究，不是新主線條件。真實估計與固件路徑需各自驗收。這是依據新 PDF 的整體方向，細部任務由使用者另行安排；**本報告不新增訓練或硬體操作授權**。

### 5.4 既有通用仿真與獨立工作區

舊 M0–M3 baseline 保有 Pygame 高層命令、UDP、起降與失聯懸停、非理想條件、簡單 IMU + flow/ToF 估計及任務 benchmark。M3 不是完整 EKF，沒有完整 bias／航向／真實光照材質模型；這些局部里程碑也不等於主專案 S0–S6。

仿真拆分紀錄：物理抽離前後 RL 26 項檢查通過、原物理來源 hash 保持；後續獨立工作區 root 128、sim 136 tests 通過，獨立 ArduPilot micro-flight instance 67 headless SITL 通過。這證明拆分時的兼容與獨立執行，不是新 PX4/VIO 任務認證。外部資產透過本地 `.local/assets.json` 類 manifest 接入，不會因 Git clone 自動獲得原始 CAD／錄影／二進位。

## 6. RL：自主末端控制研究

### 最新：第一個通過基本飛行門檻的 PPO baseline（2026-11-05）

獨立`state_flight_working_v1`已實際完成兩候選，final同一seed0 policy在原flight probe正式評估 **Hover19/19、±X/±Y 0.20m Shift4/4**，最後0.5秒連續穩定門檻與Physics/scorer未變。PPO輸出推力與小角度姿態，由固定姿態迴路轉成原CTBR；位置/速度回饋由PPO學習，無位置PD/teacher。Actor採body相對12維、固定尺度、無bias Tanh與時間上限bootstrap；不把這解讀為直接body-rate actor學會全部內環。

第一候選196608controls仍0/23；第二候選262144先過hover19/19才開shift，393216最終23/23。兩候選共589824controls、1695.478s learning、771完整episodes；評估378.757s，另候選2工程檢查10.687s、回歸1.93s，皆在共享上限內停止。8env實際約343–359controls/s，短測單env約81。PPO shift平均末端誤差1.10cm、速度RMS.125m/s、穩定2.412s、最大超調5.37cm；PD對應1.224s/.0125cm，仍有差距。Formal actor/mixer裁切及零推力均0。中途±Y穩定.453/.429s保留FAIL，不改標準。

證據：`components/rm27_RL/artifacts/agile_flight/flight_working_seed0_01/index.html`；final checkpoint/載入契約、23例CSV、全段曲線與成功/失敗MuJoCo回放已保存。127保護來源hash與3798歷史檔案metadata無改動，原A/B/C及contact0/19維持。這是名義物理單seed非接觸baseline，未證明多seed、真機或敏捷接觸能力；未commit/push/操作硬體，已停止，不自動進入下一階段。


工作位置 `components/rm27_RL/`。當前方向是依狀態自主學習 collective thrust + body rates（CTBR），**不要求模仿 teacher action**。早期 baselines 或 imitation 研究不是現在 RL 的強制架構。

舊 I4 是「無接觸到達」，同一時刻滿足：板前 gap 0.20±0.04 m、法向速度 1.5–3 m/s、切向位置 ≤0.04 m、切向速度 ≤0.5 m/s、完整姿態誤差 ≤15°、body rate ≤3 rad/s。到達不等於真實接觸；更早嚴格 3 m/s 結果不能事後用新容差改判。

目前主線已改為 **PerchRL-style Two-Stage PPO**：首先從固定平台前穩定懸停接管，自主學會連續加減速與姿態協調，直到指定底部件首次受控實體接觸；後續才做處理後視覺／定位估計資訊微調。傳統系統負責前段起飛、導航及搜尋；沒有Teacher action imitation或預定轉姿時程。舊移動進場／只接管末段是歷史工程基線，不再限定新Stage 1。恢復另設控制／估計器重新准入支線；BPTT／SuperMimic／Flightning仍是方法比較，原E1-B不等於新Stage 1。

**2026-11-05 C final只读控制诊断完成：主要证据指向策略未学会有效反馈，未发现坐标/观测接线错误。** 直接使用final update125和冻结normalizer，比较同状态原成功PD；0新物理步进/0训练。原C23例9682控制的raw obs/normalized input/actor mean逐值复现，physical CTBR误差≤5.6e-17；PD5条成功轨迹4000控制亦复现。零误差/速度/水平/剩余4s时，PPO给[2.07862N,−.48892,−.25655,−.20428rad/s]，PD为[1.4715,0,0,0]。±.20m X误差PPO均负pitch，±Y均负roll，不能两侧同时纠正；291个标准化探针无normalizer裁切。高度误差→推力局部增益PPO.0339、PD.9N/m；vz阻尼虽方向正确但抵不过偏置。保存的实际电机力投影约1.59660N>mg1.4715，力积解释名义上升末vz1.75479m/s，近似误差9.1e-5m/s；无重新积分。Reward距离方向正确、上飘扣分；dense项饱和、rate成本尺度弱、无直接yaw角成本仍是学习假设，不证明reward exploit或唯一根因。另有264/9682状态Rxx/Ryy normalizer裁切，名义首次2.045s，不能解释初始错误。14037离线查询/40.148s，289输入hash及权重/RMS不变；只新增诊断产物，原Physics/Reward/Action/scorer不变。C0/23、原接触0/19及143来源UNKNOWN保持，无追加训练/硬件/commit/push。详见[只读控制行为诊断](components/rm27_RL/artifacts/agile_flight/state_flight_probe_c_control_diagnosis_01/index.html)。

**2026-11-05 Flight Probe C诊断已执行并停止：最后完整策略0/23，存在局部行为改善，训练中断限制保留。** C仅在B安全失败terminal reward加入固定12/s剩余4s折扣成本；γ=.999按5ms，失效slot剩余1ms区间正确计入、物理立即停止。使用原B initial actor/critic/optimizer/RMS，首512物理与策略字段逐值一致，仅终止成本改变；原Physics/Action/PPO/scorer不变。离线182训练+23评估显示早失败组旧G−11.140优于晚失败−11.883，补偿后−31.962劣于−19.272；公式显式800slot误差1.42e-14，不据此证明策略刻意求失败。两项focused tests通过，C125完整有限更新/64000训练transitions。外层143、来源UNKNOWN，未重启/补训；保存158完整episode及129checkpoint，已落盘64083controls，未完成rollout内存尾段缺失，实际总controls界64083..64512。最后持久化学习時計779.122s，文件时戳/退出观察给出含初始化上界约819s，精确停止时刻未知；import+init约3.764s，0编译。恢复最后完整checkpoint后仅评估原23例，107.342s/9682controls/48371子步，全部越界，完整4秒0/23。23例均值B→C：飞行时间.934→2.103s，速度RMS1.578→.776m/s，峰tilt21.39→9.25°，zeroT21.31%→0、Actor15.27%→0、Mixer45.35%→0；但平均位置误差.3747→.3854m、yaw37.38→48.19°。名义策略推力/角速振荡并上飘越界，四移位三例末端方向错误，未学会闭环位置保持。共同C目标G−33.586→−23.400、共同B目标−10.759→−10.932；支持早停成本影响学习行为，不证明唯一根因。C125与历史B128更新数不完全匹配；单seed且缺失中断尾段限制保留。训练有11接触失败及2满4s但不稳定，不能称全面更安全。4244受保护输入hash不变，原接触0/19及A/B历史保持，无硬件/commit/push、无追加训练。完整23例、曲线、checkpoint与MuJoCo回放：[C独立结果](components/rm27_RL/artifacts/agile_flight/state_flight_probe_c_seed0_01/index.html)。

**2026-11-04 独立 State Flight Probe A/B seed0已实施并停止：工程PASS，PPO基础飞行仍未学会。** 固定AirGym bdaa61f6与IsaacLab207f319d（所给quadcopter旧路径移除前版本），实际阅读Hover/CTBR/obs/reward/reset/PPO源代码；未安装或迁移框架。新独立`state_flight_control_probe_v1`复用原MuJoCo/40ms motor/5ms CTBR/1ms积分，β=.1及原collective映射。A/B唯一差异是预登记位置/速度/tilt密集成本λ0/1，同initial网络、optimizer、normalizer和RNG；首512控制的sample/mean/物理路径/共同reward逐值相同。传统PD悬停＋四方向.20m移位5/5，23reset通过。A65536controls/749.708s/165完整episode，B65536/733.819s/182完整episode，各另1未结束片段；各128有限更新。共享评估310.399s，initial及A/B final均0/23，全部越界，未达末.5s连续稳定。A所有formal case无actor/mixer裁切或零推力仍失败；B hover平均存活2.183→.934s、速度RMS1.001→1.569m/s、峰tilt11.75→21.60°，zeroT21.59%、mixer45.91%。更小积分误差伴随更早失效，不能算改善；B亦非稳定拒绝移位。密集成本干预在此单seed预算不足，不能全面否定Dense Reward。下一步仅建议优先审查Reward/提前终止的剩余时域成本，Action先冻结；未证明唯一根因或reward exploit，不追加训练。264份checkpoint套件、全部失败、69正式轨迹及7段原MuJoCo记录状态回放保留，0额外物理重跑。回放仅修新task与原plant身份分离，单列运行后源码hash；原Physics、接触Reward/Scorer、contact0/19和历史数据不变。来源／预算核对1755输入通过，exit0，无硬件/commit/push。详细：[结果与23例完整表](components/rm27_RL/artifacts/agile_flight/state_flight_probe_ab_seed0_01/index.html)。

**2026-11-04 State-Based学习机制离线审查完成；无新训练。** 覆盖β=.1全部256更新与131072controls；逐步RMS和行为均值重建对原记录误差0，GAE explained variance差≤4.77e-7。现推力映射a≤−.5均裁为零：首control为探索尾部，第138个rollout起均值也进入该区，末32更新零推力23.79%，Gaussian std仍约.22。final的71.77% actor裁切来自roll/pitch，推力轴0；连续零CTBR的.230–.480s仍获得净reward+.17997，而末段失败惩罚正常生效。零CTBR仍有混控分配及滞后motor force；不能称电机停转。最高优先级学习假设是接近塑形与全程协调/制动信号、信用分配及采样覆盖不匹配，尚未证明唯一根因或reward exploit。只提出独立非接触hover/短移位配对消融：同β=.1动作链，唯一变量为密集状态协调成本开关；未实施、待批准。原接触0/19、成功条件、源代码与历史文件保持；证据在RL `.agents/runs/stage1_learning_review/`，详细设计与预算见唯一活动计划§0。

**2026-11-04 β=.1 seed0匹配PPO完成并停止：工程PASS，原接触任务仍0/19。** 精确恢复原initial actor/critic/optimizer/normalizer、首物理状态/观测与随机normalized动作；仅rate映射20→2rad/s，原19初态、Physics/reward/scorer/bounds/安全终止和PPO设置不变。131072controls/654464子步、1523.502s学习、256有限完整更新；正式前后19例加early/mid行为共192.438s评估，初始化3.473s/0编译，进程1720.380s/exit0。前后均0/19、全部越界；旧final为0/19、全部角速超限。最终19例平均mixer裁切96.04%→44.54%、峰角速20.006→3.432rad/s、G(γ=.999)−4.464409→−3.547759；新initial G−3.454768，因此不认定任务改善。新final名义1.045s越界、yaw33.04°、68/209控制T=0，最终下降1.162m；接近距离改善但位置/姿态协调、受控接触及局部稳健性未学会。256逐update checkpoint/normalizer、459完整episode（455越界、4非法真实接触）及1未结束片段全部保留；5个记录状态回放无新物理积分，无成功回放。不同rollout导致normalizer分化，不将beta称为唯一失败原因。记录I/O计入学习预算；另列测试7.59s、离线张量.035s/分析6.404s/渲染17.808s。证据与完整19表：[独立结果入口](components/rm27_RL/artifacts/agile_flight/stage1a_beta01_seed0_01/analysis/index.html)，来源保护见同目录validation.json。建议进入下一阶段State-Based任务设计审查，不直接追加训练或改成功条件。原实验、checkpoint、dirty保留，无硬件/Git操作；原外层143仍UNKNOWN。

**2026-11-04 最新 Action Scale 單因素控制驗證完成：尺度假設獲支持，任務仍失敗。** 使用者批准的診斷僅在RL `NormalizedCTBR`增加顯式`rate_scale_beta`（預設1），候選.1使每軸rate映射20→2rad/s。總推力映射、環境bounds±20、reward正規化、Physics/PX4、scorer及成功界線不變；原dirty、checkpoint與結果保留。配置`configs/stage1/action_scale_diagnostic.json`，入口`tools/stage1_action_scale.py`，未接入或執行新PPO訓練。

16/16窗口完成，單CPU affinity0／OS threads1／單環境：3739controls／18692個1ms子步／45.740s，共同硬上限4096／20480／120s；包含導入啟動.876s、初始化1.877s，0編譯、0訓練／optimizer，進程exit0。原訓練外層143仍UNKNOWN。7組相同physical CTBR配對（4s hover＋三軸±1rad/s、.2s pulse+.6s release）逐控制及每1ms物理狀態、motor ctrl/activation/實際force、觀測/reward全數零差；hover無漂移，pulse無裁切、峰值1.11010rad/s、歸零末端norm≤.001103rad/s。rate歸零不代表姿態或位置回正：roll/pitch脈衝仍橫移.4064m、橫速1.255m/s。

| 同名義初態凍結final策略 | Mixer裁切 | 峰值角速rad/s | world yaw擺幅 | 原安全終止 |
|---|---:|---:|---:|---|
| β=1 | 96.72% | 20.00173 | 161.76° | .304s EXCESSIVE_BODY_RATE |
| β=.1 | 0% | 1.34859 | 4.19° | .788s FLIGHT_VOLUME_EXIT |

舊映射完整重現原final所有保存列（零差）；兩者Actor／physical CTBR clipping均0。候選目標座標第一軸.802176m越過原.8m界線，速度3.01956m/s、終端角速1.29922rad/s，沒有接觸；未執行到1秒或改判成功。分配collective最大誤差.684708N→2.22e-16N，候選torque分配也近機器精度，實際wrench仍有原40ms電機延遲。實際motor force逐1ms從solver記錄，不能用mixer旗標代替。

**控制可行性假設SUPPORTED；完整飛行／接觸NOT_DEMONSTRATED；原「1秒無安全失敗且裁切減半」聯合工程門檻FAIL。** 不放寬原門檻。值得另行申請匹配條件seed0從頭重訓，研究小動作域能否學會外環協調，但不是聯合工程門檻已通過；須保持原initial模型/normalizer、reward/scorer/19初態/131072或1800s及600s評測預算，僅改β，不能續訓final或自動開始。沒有跨初態/seed/硬體泛化結論。

證據`components/rm27_RL/artifacts/agile_flight/stage1a_action_scale_01/index.html`、analysis.json、run/16窗口的trajectory/control_chain/physical_substeps及完整失敗摘要；共同前304ms亦獨立比較。開始前735份文件固定hash、前輪712份核對一致，final validation排除明確授權的映射/tests/docs改動後檢查其餘全部保護來源；運行24份來源hash記run/summary.json。純映射2 tests PASS/.98s，離線圖表分析1.345s，均無額外物理步；沒有安裝依賴、重做整套工程或commit/push。

**2026-11-04 前輪只讀失敗診斷：執行鏈重建完成；當時沒有新增訓練或MuJoCo軌跡。** 直接核對原38條正式評估、initial/early/middle/final保存觀測與checkpoint，actor均值離線重算誤差≤1.2e−7；normalized→physical CTBR、共享rate proxy、編譯旋翼位置／gear獨立構成的mixer、40ms filterexact遞推均與原記錄一致。未發現符號／尺度接錯。`proxy_saturated`在此實作就是逆混控motor需求越界並裁切的旗標，沒有另一層proxy torque limiter。final Actor與physical CTBR裁剪均0；mixed-motor clip59/61=96.72%（294/304ms）。實際模擬data.act僅0.009025..1.073970N，為上限1.103625N的0.82..97.31%，不是96.7%的實際推力觸頂；act/force也沒有另設硬裁切。

時序：t=0 CTBR=[1.829916N,4.580603,11.243249,0.430252rad/s]已產生−0.034701N motor需求而裁成0；t=5ms另一motor需求1.159495N首次超上限；50ms首次保存角速norm>4（只關乎接觸條件），85ms命令norm首次>20，各軸仍在±20內。145ms首次保存pitch高於該區間保持命令，瞬態超越不能單獨證明失穩。280ms要求pitch torque−0.008848Nm，motor裁切後變+0.002207Nm；collective1.442301→2.127008N，延遲後區間平均2.210969N。期望pitch減速−27.28rad/s²，保存280→285ms角速17.9427→18.0166卻是+14.77rad/s²。304ms角速norm20.00173終止，沒有接觸。每列action作用於前一個timestamp起的區間；非終止角速僅有5ms端點，沒有偽造完整1ms狀態。

候選優先序：①policy輸出／探索尺度與致動可行域不相容；②獨立motor裁切與40ms滯後造成所需推力／力矩失真；③reward／信用分配及單seed探索問題。①②有直接控制鏈證據，但唯一因果與修正後閉環仍未驗證。Hover靜止單軸rate error餘量roll/pitch12.678、yaw4.796rad/s，多軸必須分享推力空間；collective在0或4.4145N時沒有額外差動力矩餘量。原每軸±20的box最大norm34.641，安全norm20。初始Gaussian rate std4.463rad/s，首rollout混控裁切56.45%，而initial確定性policy為0%；middle/final確定性mean才出現大幅旋轉，四快照不證明逐更新單調惡化。

| 同名義初態checkpoint | 命令norm峰值rad/s | 混控裁切 | 時間／失敗 |
|---|---:|---:|---|
| initial | 0.430 | 0% | 2.138s／越界 |
| early 512步 | 0.779 | 0% | 1.162s／越界 |
| middle 65536步 | 21.342 | 82.22% | 0.223s／角速超限 |
| final 131072步 | 20.797 | 96.72% | 0.304s／角速超限 |

Reward解讀新增限制：final名義guide+0.301382、alignment7.13e−9、action−0.002769、constraint−0.012958（含飽和−0.005880）、time−0.0304、event−5。原19例raw回報−5.290627→−4.735710，但用實際γ=.999計算確定性完整episode折扣回報G0，為−3.401278→−4.464409，反而變差。名義final−4.471597、既有4s穩定hover−2.523419。提早失敗雖少付時間代價，終止懲罰也較少折扣，故「提早失敗reward exploit」未獲證明。G0不是PPO surrogate或隨機policy的期望值；不將此結果誤寫為訓練因果消融。

原Foundation4s hover/零角速證據：800controls、速度／角速／位置漂移及飽和全0。initial/early可變小rate無裁切，但不是專門階躍追蹤測試；其他PX4 inner-loop的rate測試不能代替Stage1 proxy。缺同場景±軸小rate pulse-and-release基線，本輪沒有補跑。

唯一優先提案（未實施）：RL policy輸出rate增益加β=.1，有效每軸20→2rad/s；保留環境action bounds±20、reward原正規化、推力映射、Physics/Scorer與門檻。直接把env body_rate_limit改2會連帶改reward尺度，不是單因素。靜態同final首次動作縮放後motor需求0.408261..0.500096N全可行，hover靜止±2三軸8角點範圍0.098379..0.637371N也可行，探索std降至0.446rad/s；任意collective／omega／motor狀態下的閉環效果仍未知。

下一步只提案：原／候選映射各跑相同physical hover4s和三軸±1rad/s的0.2s pulse＋0.6s release，再凍結final actor/normalizer各同名義初態閉環≤1s。工程上限4096controls／20480區間／120s、單CPU，無訓練；保留全部失敗，原物理命令基線須一致，小rate無飽和、歸零誤差≤.1rad/s，final不觸及原安全失敗且混控裁切至少減半為提案判準。這不修改接觸成功條件。通過後才值得提請批准同初始模型／seed0／原預算的單因素PPO；不建議原配置直接續訓。

外層143來源仍 **UNKNOWN**：現有console.log、summary.json、process_identity.json與前輪findings沒有signal sender／時間／supervisor timeout或生命週期紀錄，不歸因OOM或超時，不重跑完整訓練。證據`components/rm27_RL/artifacts/agile_flight/stage1a_seed0_diagnosis_01/index.html`、diagnosis.json與逐控制trace，原實驗全部保留。712份輸入保護；離線載入/重建及圖表約3.80s（兩版分析1.512+1.295、圖表.993；文件審查及hash另計），0新physics step、0optimizer、無新增依賴／pytest全套／硬體／commit或push。

**2026-11-04 前輪 Stage 1-A seed0：有界學習與評估完成並停止；PPO 工程更新 PASS，受控接觸學習 FAIL。** 使用者已明確批准這一次 simulation-only baseline，底球9mm／body z−44.5mm、現摩擦接觸模型、90×100mm、法向20°／yaw自由、normal0..1m/s／tangent≤1m/s／bodyrate≤4rad/s及原容差、1m懸停擾動／4s均沿用。新`task_stage1a_seed0.json`的CONFIRMED僅指本輪仿真假設獲批，不是實機標定；先前PROPOSED配置與checkpoint沒有覆寫。

單CPU線程／affinity0／單環境，rollout512、batch128、4epochs。恰好131072 training transitions、653519個1ms物理區間、1501.227s學習，先達transition預算停止，無丟棄更新。256次完整rollout更新／4096次minibatch優化，53248個有限gradient hook；所有更新的參數均變化，最終actor/critic相對初始L2變化2.27517／4.10900。初始0、early512、mid65536、final131072步checkpoint均保存且可載入；原MuJoCo與全部觀測／CTBR／reward／scorer數值未改。

| 判斷項目 | 本輪證據與結論 |
|---|---|
| PPO工程正常更新 | PASS；有限梯度、參數與optimizer狀態變化、完整checkpoint及固定接口評估 |
| 受控接觸已學會 | FAIL；正式前0/19、後0/19，無成功回放可交付 |
| 敏捷與連貫動作 | NOT_DEMONSTRATED；最終名義0.304s即因角速度超限終止，尚非受控機動 |
| 局部初態任務穩健性 | FAIL；原19例全部保留，final均EXCESSIVE_BODY_RATE，未擴大到多seed或更廣分布 |

前評估19例均越界，後評估全部在0.233–0.369s角速超限、沒有物理接觸。final名義底部點最近距目標0.81641m、world yaw擺幅161.76°、完整姿態轉動240.62°，proxy飽和96.72%，normalized Actor裁剪0。19例yaw擺幅160.58–348.30°；部分接近垂直，需同看3D姿態轉動164.34–305.33°，不把Euler表達跳變當真實瞬跳。每軸動作±20rad/s與飛行角速范數20rad/s屬不同條件，final命令范數最高20.797rad/s；角速度接觸驗收仍4rad/s。沒有改限幅來使結果通過。

922個完整隨機訓練episodes皆無成功，272次首次接觸全部不合格、均沒有designated合法pair；所有terminal/contact前後狀態保留，12類失敗另存完整位置／速度／姿態／CTBR軌跡。失敗計數可重疊：越界537、超速93、角速超限110、非指定接觸272。最早接觸episode205在1.061s由post_6碰37220，法向速度4.752、切向2.697m/s、底部法向103.16°；不是合法底球低速接觸。訓練整體proxy飽和77.40%、Actor normalized裁剪4.21%、實際CTBR裁剪3.38%。

平均正式reward−5.29063→−4.73571；guide平均增益約+0.372、較早失敗少付時間代價約+0.193。未見final長懸停或完整繞圈，主要是快速旋轉／翻轉。這是回報與任務不一致的實測跡象，尚未做因果消融；不能斷言reward為唯一原因，亦未改reward、放寬scorer或啟動第二輪。最後critic explained variance0.97358只是對該失敗分布的估值指標。

評估硬預算獨立於學習：前後38例＋early/mid名義初態共9933controls／49589區間／124.999s；另final與錯誤接觸的完整記錄命令重模擬274controls／1365區間／6.083s計入相同額度，累計10207／50954／131.082s，低於30400／152000／600s。兩條qpos/qvel/motor act/ctrl/obs/reward逐值0差、終止相同。初始化2.696s；已核hash的MJB直接載入，編譯0s；原進程總牆鐘約1630.10s（約1s精度）。12項針對性測試14.30s、無額外optimizer；五段完整時間範圍20Hz MuJoCo回放含預覽28.791s，圖表與補充分析約4.01s，tensor只讀分析0.015s，成本另列。文件審查整理未逐條累計計時。

應用程式完整寫出`TRANSITION_BUDGET`及`error=null`摘要後，外層工具session回報143，原因未查明，不能稱外層正常exit0。已獨立核對256更新、38正式例、兩行為例、checkpoint與兩次精確重模擬；沒有為此重啟訓練。439份原Physics／配置／歷史結果／checkpoint等受保護檔與23份運行來源hash不變；41份實際dirty代碼／配置另固定於sources.tar.gz。無Physics、原場地、舊I4/E0/E1-A或歷史實驗修改，無真機、commit/push、Stage2/Recovery。

完整證據入口：`components/rm27_RL/artifacts/agile_flight/stage1a_seed0_01/analysis/index.html`，含所有訓練曲線、19初態前後表、五段回放與checkpoint連結；`run/summary.json`、`updates.jsonl`、`training_representatives/episodes.jsonl`為原始紀錄。下一步僅建議先評審CTBR向量界限／混控飽和及guide/時間/控制代價尺度，再提出單變量有界對照由使用者決定。此次訓練授權已用完；以下Foundation／Geometry的「尚未批准」均是歷史時點，原結果與原判保留。

**2026-11-04 Stage 1 Task Foundation 歷史結果：接口工程完成；當時實體接觸夾具FAIL；正式訓練未就緒。** 已閱讀PerchRL原文arXiv:2606.03441v1，保留原始HTML、版本、SHA-256及採用差異。新task/schema為`fixed_surface_contact_v1`／`fixed_surface_state_v1`，使用指定的`rm_uav_mujoco/artifacts/2027-base-dock-ready`抽取場景、Ver2、MuJoCo3.12.0、既有Physics rate proxy／混控／40ms電機延遲與5ms控制。私有XML固定平台並加入已有S1c底部球假設，不改原資產或Physics；全場景以命名關節／致動器地址讀寫，避免舊單機qpos索引假設。

Actor和critic同用26維：表面中心相對位置3、外法向3、世界速度3、旋轉矩陣9、body rate3、上一實際CTBR4、剩餘時間1；不含motor act、contact labels、scorer phase或未來狀態。Reset是12維小範圍位置／速度／旋轉向量／角速度擾動，電機初始化懸停值。Reward為有界接近potential差分、近目標法向對齊、CTBR幅值／變化、速度／角速、限幅／飽和、時間及一次終端事件；獨立scorer不讀reward權重，每1ms消費實際MuJoCo contact/force，核對接觸對、區域、法向、接觸點速度、body rate及穿入；同區間安全失敗優先。球接觸只約束底部法向、yaw自由，未冒稱完整姿態驗收。

新測試26項6.08s、受影響舊接口回歸67項11.78s通過。只編譯一次私人場景88.01s。唯一PPO工程rollout為seed0／64 transitions／1 epoch／batch32共2個minibatch optimizer steps／0.792s，26次有限gradient hook；actor/critic參數L2變化0.026635/0.033446。Critic explained variance -12.16，不代表估值充分。Checkpoint的policy、optimizer、歸一化、counter及確定性動作逐內容往返一致；19個固定初態全數保留、歸一化凍結，**0/19且均FLIGHT_VOLUME_EXIT**。hover逾時、實碰夾具、裁切飽和及policy中心四類完整episode的MuJoCo命令重模擬逐值零差，含motor act/ctrl、時間與終止。可保存完整軌跡與位置／速度／姿態／CTBR圖，另以原MuJoCo文字覆蓋渲染實際畫面；互動viewer入口已實作，本批未宣稱人工觀看全段GUI。

主要阻擋是6ms首次接觸同時有註冊`35364`與未註冊`36293`，所以scorer按既有登記拒絕成功。須區分未註冊和實際非目標：`base_target.json`既有確認裝甲是visual117、visual115為周邊暗框；36293幾何最吻合visual117的5mm置中碰撞殼。389/441個逐mesh射線命中，與visual117深度差約-2.5mm、與visual115近共面；需要Sim核對corrected面及原板殼的表面位置、來源歸屬與登記完整性，不能只因共面擴大合法集合，也未刪除殼。另一區域外夾具實際開始即INITIAL_OVERLAP，沒有搜尋有利位置替換。這兩次前置失敗及全部軌跡保留；接口完成不等於接觸任務PASS。

有界smoke含兩次策略更新前失敗，累計8512 controls／42505個實際1ms區間／188.77s，低於20000／100000／900s；編譯、開發測試、渲染另列。38份實際輸入／程式在正式run開始與結束hash相同，245份既有Physics／配置／歷史實驗／資產hash不變。完成後修新renderer的無Pillow文字顯示，另補1項frame0／零步篡改回歸，與既有短回放共2項4.04s通過；原零步夾具回放通過且無額外物理區間。各自版本和驗證另記，沒有再次更新policy。無長訓練、真機、視覺微調／恢復整合、commit或push。

上述為 Foundation 時點的阻擋，本輪幾何結果見下；底部件尺寸材質、有效區、接觸速度／法向／角速／穿入門檻、yaw自由度、懸停偏差和4s時限仍待確認，配置仍`PROPOSED_UNCONFIRMED`／`training_allowed=false`。首輪seed0提案為單CPU線程／單環境、512 rollout／128 batch／4 epochs，最多131072 transitions或1800s學習；前後各完整19例，評估上限30400 controls／152000區間／600s，合計牆鐘上限2400s，先到即停。保留最後完整更新；來源／契約變化、非有限及MuJoCo warning立即停止。未授權執行此提案，不以reward/loss或單次成功宣稱學會。

證據入口：`components/rm27_RL/artifacts/agile_flight/stage1_foundation_01/`，其中`inputs.json`保護既有內容，`paper.json`原論文身份，`scene/stage1_scene.json`私有模型來源；`smoke/`與`smoke_continued/`保留失敗，`smoke_completed/summary.json`／`sources.json`／`sources.tar.gz`／`checkpoint/`／`evaluation/`保存實際工程結果和來源。`contact_surface_overlap_audit.json`保存441點獨立幾何審核，`render_contact_telemetry/`保存帶狀態文字的MuJoCo圖像。正式契約SPEC§10、DESIGN§14；唯一活動計劃§0含交接與未完成項。

**前輪 Stage 1 接觸幾何／註冊工程 PASS；當時可進入 seed0 批准階段，尚未批准或啟動訓練。** 精確源重建證明35364是visual115第8個既有skin/rim代理、帶原2.5mm inward backing，36293是已確認裝甲visual117第7個碰撞殼分片，頂點誤差分別9.39e-10／5.53e-11m；不是未知支架誤觸。來源可追到runtime mesh、三角分組及生成器，但沒有STEP裝配料號。原17面登記漏掉visual117面向測試區的8片（36286..36293），新任務自有registry只補齊這組，另14片不同方向裝甲不納入；不將整個暗框／裝甲實例一概合法化。

新預設`configs/stage1/task_contact_geometry.json`只新增registry path/SHA，全部數值沿用舊task；`contact_surface_registration.json` SHA256為`13142f27dc9548f46e0aa3d5012e9cda64310bb493e3b31577eaeba5e8fea8f7`。原場地、Physics、私有XML/MJB/manifest、90×100mm區域、材質、reward和scorer門檻未動。原配置、歷史checkpoint及dirty保留；無registry配置仍原17分類，舊Foundation checkpoint只讀載入與既有內置probe PASS。

實測：中心非重疊初態6ms，35364/36293壓縮力7.311/7.077N，兩pair均接受；同狀態／CTBR下，修正前後物理軌跡逐值0差，舊配置仍原判拒絕。原across60mm／gap1mm的重疊來自球相對當地約2.22mm凸起殼的1.20/1.22mm穿入，不能再寫作已證實其他機體碰支架。保持across60mm、改gap30mm／vn0.5m/s後，初始零接觸，61ms首次接觸35369/35370，唯一失敗CONTACT_OUTSIDE_REGION。倒置core_board_29從非重疊起點79ms被拒絕，原重疊夾具仍零步INITIAL_OVERLAP；真MuJoCo小模型另覆蓋同geom側面／背面及同區間合法+非法pair，全部拒絕。

針對性17 tests PASS、22 deselected、3.20s；未重跑整套PPO工程。5例完整記錄及命令重模擬PASS，主檢查66controls／304個1ms區間／10.675s，前置夾具定位另146區間／2.766s。初次完整記錄後只補載入拒絕分支及mesh-name身份檢查，最終Plant hash與正例重模擬另記（2controls／6區間／1.479s，PASS），主工程累計456區間，不覆寫前次版本或事件；零PPO更新、零全場重新編譯。400份受保護Physics／配置／歷史結果／checkpoint／資產hash全數不變。證據`components/rm27_RL/artifacts/agile_flight/stage1_contact_geometry_01/`含來源重建、完整pair／force／前後狀態、軌跡、反例XML及來源hash。幾何工程通過不會改判舊0/19，也不證明策略已學會；保留的雙殼解算及模擬壓縮力不是實機接觸力校準。

建議首pilot先批准沿用既有工程假設：底部球半徑9mm／中心z=-44.5mm、摩擦0.5／接觸模型12ms阻尼比1、central90×100mm、yaw自由、法向20°／速度normal0..1與tangent≤1m/s／bodyrate≤4rad/s／plane及penetration各5mm，以及1m懸停接管與原擾動／4s時限。這些不是硬體實測；此次中心實測僅vn≈0.1945m/s、vt≈0.02265m/s、法向0°及微米級穿入，不能證明全範圍。原seed0預算可提請批准；現有正式評測入口尚未傳共同budget，下一授權包需先最小接入600s／控制及區間硬限再執行。

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

**2026-11-02 前輪E0分層實測：名義無接觸一致性通過；當時E1短窗口有條件就緒，尚未實施／訓練。** 純Torch解析代理保留既有CTBR、電機filterexact、5×1ms執行順序，對齊MuJoCo3.12獨立自由體隱式陀螺項。依使用者調整，640ms門檻作研究目標；是否可用於BPTT由實際偏差、梯度與任務覆蓋判斷。原E0摘要、配置、原判及舊PPO/SAC實驗保持原樣。

128個固定窗口的float64前向P95如下，逐時域及逐任務組的最大值保存在補充原始分析中：

| 預測ms | 位置m | 速度m/s | 姿態deg | body rate rad/s |
|---|---:|---:|---:|---:|
| 5 | 0 | 2.78e-17 | 3.82e-14 | 1.8e-15 |
| 40 | 2.71e-20 | 4.97e-16 | 9.76e-14 | 9.37e-15 |
| 160 | 4.44e-16 | 1.46e-15 | 1.15e-13 | 1.64e-14 |
| 640 | 3.59e-15 | 1.61e-14 | 3.37e-13 | 1.72e-14 |

指定名義離散模型的誤差仍在float64舍入量級。float32在640ms的位置／速度P95約2.91μm／7.15e-6m/s，尚未驗證float32梯度與訓練吞吐。這不代表實機精度、模型校準或學習效果。

**梯度及失效區域**：5/40/160/640ms的內域樣本51/51/34/32、嚴格飽和32/32/33/32、邊界45/45/61/64。原640ms光滑覆蓋全來自懸停與短飽和脈衝；機動／末端窗口沒有全程光滑樣本，舊總數不可解讀成末端全域可微。補充215個邊界“窗口×時域”左右方向核對，最大塊相對誤差2.37e-5；固定8個PPO/SAC機動／末端窗口完成獨立動作、初態與原歸一化動作鏈核對。單側導數的絕對＋相對判準無失敗，但不宣稱唯一Jacobian。

ε=1e-7時，5ms位置導數曾有28.43%相對誤差；導數範數僅1.78e-8、絕對差5.06e-9，與小量差分消減一致，不能只靠百分比判模型失效。完全限幅方向三檔ε均為零導數／零絕對差，cosine未定義；這也表示真實飽和可能使優化停滯。原384條擾動窗口均無事件；另驗證第2／5子步到達、第5子步安全越界及初始接觸，全部停止並排除事件梯度。

G2 PPO seed0與X9 SAC seed0各九條舊CTBR輸入重放，保存端點及終止子步完全一致，非重新推理策略。插樁累計63562次reference控制、317768物理子步、95.39s；含未插樁舊檢查的預留後記帳64074／321864／125.40s，低於原上限。預留不是全部歷史回歸成本已證實的上界。新測試13 passed；既有Arrival／Contact27 passed保留先前身份。本次無新訓練、依賴或Git發佈。

**來源與證據**：RL `ddf159395d5b1391011a0839732b6b7f5a3807b6`、Physics `f414b4c7517db7eae79ffee112d60bc45f2ef302`，實際使用dirty內容。95份所需源碼／配置／依賴鎖的內容包、兩倉HEAD／dirty、載入路徑、版本、生成XML及編譯參數均固定；原129份來源／配置／舊軌跡hash不變。原 `components/rm27_RL/artifacts/agile_flight/diff_e0_01/summary.json` SHA256 `b8bd6e6a07ed2ec3773c8f23ef532cbb2b38638e5480b5dcbcade39c3bdb07ac`；補充 `diff_e0_01_stratified/analysis.json` SHA256 `7e77e3a0a617776a3148386b673fcd5501f64c34601d9f625f4eb3b5adacc876`，同目錄sources.json／sources.tar.gz可重建實際輸入。正式方法見SPEC §11／DESIGN §12，原始分析入口diff_analysis.py。

**E0當時的E1適用邊界**：名義模型40–160ms短窗口BPTT有條件就緒；原觀測／動作、事件mask與九初態接口已由下列E1-A接入，完整學習另需預算。既有 `set_dynamics_scales(mass=1.15)` 令body_mass=0.1725kg而MuJoCo快取質量矩陣仍0.15kg；代理拒絕此不一致，本輪未修復或重判舊DR。接觸、reset、任意長時域梯度、JAX、學習世界模型／殘差與實機仍未驗證。

**2026-11-02 最新E1-A：工程PASS；I4任務0/9；E1-B未啟動。** 使用者另行明確授權80ms短窗口BPTT、critic bootstrap與有界最小更新。新增RL側 `diff_policy.py`／`diff_smoke.py`、唯一配置 `configs/agile_flight/diff_e1a.json` 與測試；正式契約／設計為SPEC §12、DESIGN §13。沿用E0名義MuJoCo3.12.0／CPUfloat64及原26維觀測、hover-centered normalized CTBR、正式ArrivalEvaluator；actor與critic都無電機act或評分器phase特權。影子MuJoCo原env.step逐1ms決定終止，解析模型保留已執行區間的梯度；電機、上一CTBR、時間與完整評分歷史跨窗口保留。

單seed、單CPU執行緒，實際只做3次actor SGD與6次critic momentum SGD更新，沒有完整E1-B訓練：

| 更新 | actor梯度norm | 參數變化norm | 同批固定critic的loss前→後 | critic固定target MSE降幅 |
|---|---:|---:|---|---:|
| 1 | 0.168327 | 1.68327e-5 | 0.0320264643→0.0320236311 | 7.45% |
| 2 | 0.168476 | 1.68476e-5 | 0.0333523876→0.0333495494 | 15.41% |
| 3 | 0.168801 | 1.68801e-5 | 0.0361493323→0.0361464826 | 21.57% |

梯度全部有限，gradient·delta皆負。Actor參數方向autodiff0.1683268561、ε=1e-4中心差分0.1683315022，相對差0.002760%；bootstrap端點觀測梯度norm0.129432。Actor更新凍結critic權重但保留其輸入梯度，critic使用detached target且不改actor。跨批critic已變，不能直接比較跨批loss。已記錄窗口／探針最大state／觀測／reward差為1.14e-14／2.73e-15／2.23e-16。

第2／5子步到達、第5子步安全退出、初始接觸及原3s任務逾時全部取消bootstrap；逾時探針跨38個窗口、3000個1ms子步，沒有重新初始化評分歷史。單元另驗證累計飽和到閾值後的第1子步終止及嚴格裁切零梯度。3個更新批的24個窗口均為完整80ms且無飽和，不能解讀成全域可微覆蓋。接觸模型、事件時間導數及質量快取路徑仍不支持。

原固定九初態評估接口正常執行，**0/9成功，FLIGHT_VOLUME_EXIT×9**；這是工程接口PASS、任務FAIL，沒有因最小更新改判成功。新格式checkpoint的actor／critic／歸一化buffer及優化器狀態逐內容往返一致；載入策略重跑中心初態，13個原始軌跡陣列、175行逐值相等。來源、配置、版本或動作契約不符會拒絕，舊SB3 checkpoint保持原樣。

正式smoke單次成本為**3371次控制、16822個實際子步、15.1763s**；執行前保守預扣5子步後記帳16855，低於8192控制／40960子步／300s上限。相關60項測試通過（4.28s），無額外優化器更新；測試成本另列，完整回歸未全量逐步插樁。開始固定159份既有檔案，結束全數hash不變；來源包固定112份實際含dirty內容，Physics／評分器／PPO／SAC／E0／歷史結果未改，無新依賴或Git發佈。

原始證據：`components/rm27_RL/artifacts/agile_flight/diff_e1a_01/summary.json`，SHA256 `f02efa93fa670a789de70397e771d273a8df8de29d104b90a64d51f7d38da541`，`passed:true`且兩組來源不變檢查為true；同目錄sources.json／sources.tar.gz、預登記配置、XML、checkpoint與評估軌跡。來源包SHA256 `6d802be2135f0e5c74c35639780815e434829c5fc329569c543e1f91ca880b4d`。

**E1-B建議（PROPOSED，未授權執行）**：核心鏈具備工程條件，尚需有界的連續episode窗口採樣排程；3步fresh-reset驗證不證明critic長程估值或到達學習充分。建議seed0、原任務／原評分、80ms／8窗口、單CPUfloat64，訓練≤512次actor／1024次critic更新、65536次控制／327680子步／1800s；評估初始及每64更新的原九初態，另含整episode critic診斷與checkpoint重放，評估獨立≤60000控制／300000子步／600s。單seed合計≤2400s、125536控制、627680子步，先達即停。實測每批前向＋backward平均1.53s，512批線性外推約784s，只是部分成本估計。非有限／warning／前向或來源不一致立即失敗；連續3批actor梯度norm≤1e-12停止診斷。記錄原任務成功數／失敗類型、到達時間、reward、飽和率及critic回報誤差，不追加seed或預設學會。不引入Teacher／模仿、質量DR、GPU或JAX。

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

**2026-11-11 現行分工：** RK3576 是唯一機載研究與驗收平台。下表取代 2026-11-03 的雙平台並行安排。

| 模組／平台 | 本機入口 | 狀態 |
|---|---|---|
| VIO／RK3576 | `components/rm27_vio/` | 現行 Camera–IMU 估計研究入口 |
| Tracking／RK3576 | `components/rm27_tracking_rk3576/` | 現行通用影像採集、預處理與回放入口 |
| VIO／M3C | `/home/shiuhou/Projects/rm27_vio-m3c/` | 歷史 `research/m3c-vio` 工作樹；停止 M3C 驗證 |
| Tracking／M3C | `rm27/perception/vision/rm27_tracking_m3c/` | 歷史 `m3c_tracking` checkout；保留第三方上游、回放與 P1 detector 的來源 |

M3C 曾用於單目 Camera–IMU 準備及 OpenVINS 研究；舊部署計畫不再執行。
資料、時鐘、標定與硬體驗證仍按來源區分，不能用 M3C 證據驗收 RK3576。
本輪只同步文件，以下結果保留各自的原驗證範圍。

OpenVINS 是目前主線；舊單目工作已獨立／歸歷史路線。VIO-P 與 S0 仍 **PARTIAL**。

可靠輸入 PC 候選已有五次一致輸出 hash，但原始非同步採集失敗不能因此消失。candidate04 診斷記錄：logger 等待 10,443 μs、兩次 publication 相隔 1,498 μs、generation 跳號 815904→815906，ULog 缺少中間 gyro 積分；缺口 4,507 μs，而鏈路 76/76 無缺。這支持一次 overwrite 診斷，不是已修好所有丟樣。

整段 11,788 個積分平均 196.32949 Hz、有兩個 hole 與 startup rejection，原 ≥200 Hz、1 cm 與 timing 驗收仍未通過。148 passed / 1 ROS2 skipped 是歷史軟體檢查，不是定位精度認證。

未決：相機–IMU 外參與時偏、曝光／PTS 語義、跨時鐘映射、加速度完整性、穩定板端資料。handoff 記錄最後觀察到 candidate04 診斷版 ON；不能不查設備就當成現在的固件狀態。原 `.maixpy` 等本地原始資料也不會隨 Git clone 自動出現。

### 8.2 SLAM

獨立 MaixCAM2 單目研究起步包，既有 176 tests（含 clean clone）記錄。主機 Stella 公開資料集路線可用；stock ORB shutdown 曾 segfault，patched 路線成功，但精確 patch 依賴外部材料。

板端 KLT 前端測量：336×190／180 points／1 thread 約 29.33 FPS；672×380／400 points／2 threads 約 10.09 FPS。**這只是稀疏跟蹤前端，不能當完整 SLAM FPS**。OS04A10 標定有 27 fit + 7 validation，僅對同一光學設定有效。

完整後端的記憶體、建置、板端實時、單目尺度與飛行可用性仍未知。舊 README 中 `rm27_slam-vio` 名稱不應用來合併目前 SLAM 與 VIO 職責。

## 9. Tracking：高速採集與真正可用觀測的差別

現行 Tracking 平台為 RK3576，入口見 §8.1。下列 M3C 真機與使用原辨識器的
PC／SITL 結果按原配置保留，不能當作 RK3576 真機驗收。更名及版本見末節。

### 上升 1.3 m 後的完整視覺閉環與停距（2026-11-08）

使用者指出前輪遺漏後半段，故重跑既有 `visual-heading` 入口，僅把 native 設定的
起飛升高量改為 1.3 m；原綠燈、原辨識器 binary／設定、0.2 m/s 接近速度、PX4 參數
及所有時效／碰撞／停距驗收保持不變。私人場景新建，未改寫原 MJCF。
相機 RGB 實際送入辨識器，採納新鮮確認像素後修正 yaw；公尺停距仍由靜態地標 map
與 PX4 估計器輔助，定位是合成 ODOMETRY，沒有改稱單目測距或 VIO。

產物：`components/rm_uav_mujoco/artifacts/visual-standoff-hangar-top-1p3m-001/`，
summary／validation 及獨立 50 Hz 物理／ULog 審核 PASS。101.712 s 仿真，
最終法向距離 1.012175 m，連續合格 5.004 s；最後五秒 250 個樣本的停距
0.997823–1.049331 m，水平誤差最大 0.048879 m，垂直誤差最大 0.010221 m。
初段五秒穩定視窗實際升高 1.22448–1.29106 m，沿用原 0.1 m 位置門檻；
1.3 m 為起飛設定，後半段另向原生燈高度接近，不是全程保持該高度。
605 個採納觀測、1,018 幀原始 RGB／938,188,800 bytes，1,001 次完成辨識的逐筆重放 PASS。
兩次共享回放均 1,018 幀，有序像素摘要一致；兩個不同主機時鐘 epoch 的原始時間及完整像素核對 PASS。
原 28,050 個保留碰撞體通過 0.15 m 淨空准入，原場地 hash 不變；75 項參數讀回／ULog 相符。
完整回歸 439 PASS／5 環境 SKIP（149.81 s）。版本仍為 Sim `f414b4c` 加既有未提交實作、
MuJoCo 3.12.0／PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`；無硬體操作、commit／push。
8081 的 mjviser 已切換此完整錄製並循環播放，保留 3D／左上角原始 FPV、當幀綠燈標記與
已送出指令。實際 Chromium 原始像素摘要、40／60／40 秒跳轉、標記／儀表切換、
播放與兩種視窗大小核對 PASS；結果在同目錄 `browser-check.json`／`viewer.png`。

同次錄製的開頭失視審核：來源 23.3–30.0 s 的 68 幀均完成辨識，無順序／傳輸錯誤，
但沒有當幀確認像素；23.504–30.060 s 保持原位置與 yaw 參考。29.9／30.0／30.1 s
連續候選在第三幀恢復確認。九張原始 RGB/hash 抽查顯示綠點仍在畫面，距離約
12.66 m、辨識框約 1–2 px；具體被拒門檻仍未知。原像素放大在同目錄
`early-loss-crops.png`。因此停距 PASS 不能解讀成連續追蹤品質 PASS；本次只讀診斷，
沒有修改辨識／控制參數或重跑飛行。

### 前輪 1.3 m 固定航點回放基線（2026-11-08）

前輪誤把使用者「其他照舊」套用到當時 active viewer 的固定航點版本，遺漏了使用者所指的視覺後半段。
該基線保留「機庫頂面仿真承托塊 → 上升 → NED [−0.8, 0]、[−0.8, +0.6] m
→ 五秒懸停」，速度仍為 0.25 m/s。`configs/fixed_waypoints_half_height.json`
升高量由 1.26825 改為 1.3 m，對應 slot_1 原點 D=−1.3635 m；其餘配置保持一致。
沒有啟用視覺導引、辨識器、降落或硬體操作。

`components/rm_uav_mujoco/artifacts/fixed-waypoints-1p3m-001/` 的 summary／validation 均 PASS：
35.708 s 仿真、358 幀原始 RGB 與共享回放、842 次本機指令送出完成；最後五秒的
250 個物理樣本升高 1.299764–1.300136 m，水平誤差最大 0.036611 m，
垂直誤差最大 0.000236 m，速度最大 0.137415 m/s，零接觸並持續 ARMED/OFFBOARD。
有序影格／時間戳／完整原始像素核對 PASS；原 MJB、韌體 binary／commit、PX4 參數不變。
程式基線 Sim `feat/moving-base-target`／`f414b4c` 加既有未提交研究；MuJoCo 3.12.0、
PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。新錄製與舊證據分開保存，未 commit／push。
mjviser 0.0.14／Viser 1.1.1 的 8081 服務當時切換此錄製並循環播放；Chromium 三張原始
影像摘要、播放時間推進及零頁面錯誤核對 PASS。相關測試 122 PASS，完整回歸 439 PASS／5 環境 SKIP。
這是固定參考航線與合成定位的 SITL 結果，不是新的綠燈閉環或實機驗收。

### 原生綠燈影像到 PX4 SITL／一公尺懸停（2026-11-03）

本輪使用 `components/rm_uav_mujoco/` 的原生 2027 抽取場景、Ver2 與實際 MuJoCo RGB，
連接 `rm27_tracking_m3c` 原 `GreenLightDetector::process()`，再由真實 PX4 v1.17.0
`d6f12ad1…` 執行姿態／混控／馬達控制。新增 `visual-heading` 入口；相機、錄製及
共享 replay 沿用現有實作。formal 視覺段始於機庫上空，正常初始化／短升起後移除
標記支撐台，再先自由懸停五秒；沒有逐幀重設狀態或把支撐當成懸停。

±15° 與 0° 定點航向三案通過 ≤2°／連續五秒與位置／速度門檻；停用修正的對照保持
約 15.31°。軟體測試燈前 1.011 m 及原生 `visual_116` 燈前 **1.012 m** 均完成持續懸停。
原生正式產物：`components/rm_uav_mujoco/artifacts/visual-standoff-native-002/summary.json`，
PASS；90.512 秒仿真／163.029 秒主機，視覺段 69.296 秒、連續合格 5.004 秒，
末端速度 0.00348 m/s、水平視線誤差 0.0526°。原生案例在私人初始化中隱藏額外軟體燈，
沒有修改場地本體；11.963 m 路線先檢查 28,050 個原生碰撞 AABB／0.15 m 餘量。

906 幀精確 RGB3／834,969,600 bytes；兩次完整共享回放各 906 幀，端到端
1633.11／1635.04 FPS，零 queue drops、有序像素 SHA-256 一致。889 個原辨識請求
含暖機的完整確定性輸出逐筆精確回放；估計回放亦一致。624 個採納量測的像素／
實際採樣 PX4 姿態與有界 yaw 指令可重建；原始場地 145 個 runtime hash 不變。
render P50/P95/P99 12.446/20.572/29.332 ms；原辨識處理 0.282/1.876/2.838 ms；
來源首次 render-complete→本地 send-complete 41.821/73.333/90.191 ms。
視覺段父程序 RSS 增約0.219 MiB，CPU約一核心；不是板端或隔離效能承諾。

已定位並修正三個實際失敗：仿真機型舊 yaw 迴路振盪（逐次執行限定角度增益／torque cutoff）；
浮點時間與奈秒序列化誤拒絕同刻資料（統一精度）；條件式測距融合在地形變化時切換高度參考
（原生案例明確 EV-only height，仍保留測距訊息）。完整失敗／ULog 留存，未放寬驗收。
最終 Sim 345 PASS／5 SKIP，獨立 M0–M3／協定39 PASS，PX438 PASS、共用 tracking62 PASS，
M3C 原 CTest13／影片 CLI5／新 RGB7 PASS。重現指令及程式／設定／wheel hash 見 Sim/PX4 README。

這是**影像偏航＋地圖／估計器輔助停距**，定位仍是 truth 產生的 synthetic ODOMETRY→EKF2，
不是 VIO 或單目公尺測距。燈照明／物理鏡頭、M3C/RK3576 通信與實機未驗證；結束為仿真清理，
本輪未宣稱正常 LAND。無硬體操作、Git commit／push 或無關倉庫修改。


顯示與起飛更新（2026-11-04）：`replay-sitl-browser` 同時顯示 3D 主場景與左上角原始 FPV，
辨識標記及已送出指令保留來源核對；PX4 儀表改為預設隱藏的獨立面板。Chromium
兩種視窗尺寸、像素 hash、播放／跳轉與面板切換通過。服務當時播放新的頂面起飛記錄，
不是即時硬體連線。證據沿用 Sim `artifacts/sitl-viewer/browser-check.json`。

使用者確認先用舊機庫停放格最高上緣：原底面 world z1.76m，上緣 z1.80m，
私人場景加入單格142.5×142.5×40mm仿真墊高塊；保留原機庫，等待新資產。
從此面實際 PX4 SITL 起飛，升至既有已檢查高度後接續非接觸視覺段。
`artifacts/visual-standoff-hangar-top-001/summary.json` PASS：96.896s仿真、
最終機體中心法向距離1.0111m、連續5.004s合格。969幀原始RGB／兩次共享回放像素摘要一致，
952次原辨識器精確回放通過；75項參數與ULog相符。獨立物理／ULog審核PASS，
無自由飛行碰撞；全Sim360PASS／5SKIP含M0–M3矩陣。來源場地hash不變。
原格內起飛候選依使用者更改範圍中止，未完成資料留存，不計為任務成功。
新機庫幾何、物理鏡頭、真實定位與實測動力／時延仍未驗證；無commit／push或硬體操作。

Tracking 動態軟體驗證（2026-11-04）：使用者優先驗證識別程式與視覺飛控閉環，
不把真機模型校準當前置條件。Sim 在現有 `visual-heading` 增加獨立的三公尺軟體燈測試：
相同左右平滑位置參考／一秒錄製前全黑，主動視覺航向修正與被動關閉修正對照。
原辨識器、PX4 firmware／參數、原始錄製／共享回放共用，沒有飛行中重設位姿。
`artifacts/visual-tracking-stress-active` 與 `-passive` 均完成試驗及原有最後五秒穩定驗收；
被動 PASS 只代表對照有效。10 Hz 動態評分主動 RMS1.241°／P95 2.585°／最大2.748°，
被動 RMS1.652°／P95 3.446°／最大3.568°；兩者實際左右範圍均0.356m，
航向範圍分別5.397°／0.252°。各10張黑畫面均無確認量測被採納；各23個過期量測指令
均保持航向。主動遮擋後0.304s開始新鮮且≤2°的連續0.5s恢復視窗，稍後才確認。
獨立50Hz物理／相機幾何重算各550個可見動態樣本：主動RMS1.233°、被動1.649°，
改善25.22%，通過事先≥20%門檻；raw/BGR/hash、來源／過期指令／publication、
75項參數／ULog／飛行邊界審核均PASS。證據為Sim既有診斷目錄的
`tracking-stress-audit.json`／`formal-flight-audit.json`。
主動425幀／408原辨識器請求、被動424幀／407請求；各自兩次共享回放及精確辨識回放PASS。
完整Sim **371 PASS／5 SKIP**，含直接執行的M0–M3矩陣。
來源hash與未完成的實機校準界線保留，當時8081顯示原生頂面起飛案例。
三公尺軟體燈案例不是新的原生場地任務驗收；沒有把動態最大誤差超過2°寫成全程精準居中。
重現入口在Sim README；證據沿用 `.agents/runs/sitl-camera/`，無Git發布。

### 2027 基地、機庫與飛機抽取（2026-11-03）

Sim 新增 `extract-base-dock --config configs/field_subset.json --output NEW_DIRECTORY --preview`。
本機可用產物為 `components/rm_uav_mujoco/artifacts/2027-base-dock-ready/`，含自足的
`scene.xml`／資產、MuJoCo 3.12.0 的 `scene.mjb`、五張實際預覽及驗證摘要。
保留目前 2027 +X 藍方基地、中央機庫、兩個原生托盤、所選原始碰撞與滑台；
跨邊界碰撞整件省略並計數，另加明示的平面測試地板，沒有混用舊基地或修改來源。

重用現有四架150g Ver2；`parked` 為四機停放，預設 `mission_start` 讓1號機直接
從負XY槽位地板上方2m、世界z3.76m開始，其餘三架停放。這只是空中初始條件，
沒有執行懸停控制。來源明確標示 +X 基地為藍方，但兩個H邊框均為藍色、托盤灰色，
無法從來源確認機庫隊伍；負XY指定紅方只屬這份仿真配置，原資產未改色。

**PASS：** 實際編譯95.54s、含EGL預覽總計102.65s；29,101個編譯幾何、23,575個mesh。
28,084個保留來源幾何與10個原生site的世界姿態、尺寸和碰撞參數逐項完全一致；
42個複製資產、11個飛機來源和145個場地來源hash通過。
兩個初始狀態均零碰撞／速度／致動器輸出；MicroPlant可載入但未步進物理。
相關回歸92 PASS、2環境SKIP、2目標／接近案例排除。精確default/keyframe檢查曾揭露
姿態序列化誤差，已改由MuJoCo繼承編譯後初值，沒有放寬相等檢查。
影像→航向、新場景飛行控制為 **NOT_RUN**；沒有新增真機或RK3576驗收，未commit／push。

### PX4 SITL 固定懸停與被動機載相機（2026-11-02，America/New_York）

本輪另行授權範圍只有完整 2027 場地中的固定位置起飛、懸停、正常降落與相機資料鏈。
Sim 新增 `python -m rm27_sim px4-camera-hover --config configs/px4_camera_hover.json
--firmware-source /path/to/PX4-Autopilot --output NEW_DIRECTORY --instance 75`。
場地仍為 `d9439a97`，全部 145 項 runtime hash 前後不變；沒有改資產或連接硬體。
PX4 元件把原有通用 Runtime／HIL bridge／profiles 打包為 `rm27-px4-sitl` 0.1.0，
Sim 直接安裝此 wheel 與已驗證的 `rm27-tracking` 0.3.0，沒有複製影像 runner 或匯入父倉實作。

實際使用 PX4 v1.17.0 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`，
binary SHA-256 `6dd3602f0f57355688f255c9fc0458fc35504f562e0e9bbf2cd4f8db2b76fdf8`；
Python 3.12.3、MuJoCo 3.12.0、EGL / RTX 5060 Ti。固件 `px4` target 實際建置通過，
獨立 CMake fresh configure、wheel checkout 外載入及 17 項通用 backend 測試通過。
EKF2 融合仿真真值產生的合成 ODOMETRY；**不是 VIO**，相機完全被動。

Sim `artifacts/sitl-camera-003/summary.json` 為 **PASS**：73 項期望參數解鎖前讀回相符，
0.6 m 機庫 slot 原點高度連續懸停 5.004 秒，物理真值最大 XY 誤差 6.13 mm、
高度誤差 80.41 mm、速度 0.03460 m/s、傾角 0.2078°。
21.376 秒請求正常 LAND，45.676 秒應用層確認 PX4 自動停轉，46.676 秒完成接地／零輸出檢查；
ULog 無損壞、確認外部定位融合及 landing disarm，沒有強制 disarm 指令。程序與本地 sockets 釋放。
另一個經 hash 核對的 MJB 快取獨立起降試驗亦通過；不宣稱獨立非同步閉環飛行逐位元一致。

錄得 467 幀／430,387,200 bytes，640×480 原始 RGB3、10 Hz 仿真排程、0 漏槽，
保存首／懸停／末張 PNG、姿態、來源與分離的仿真／主機／回放時間。
逐幀原始 RGB→BGR 完全相等，兩次完整回放各 467 幀、queue drops 0，
有序處理像素 hash 均為 `8efece2ddca3f3b256bbc2bf222b0bcd18c9488550cea71cd3c22180c08d97f1`。
回放 1,712.41／1,749.65 FPS，preprocess＋process P50／P95／P99 分別
0.097／0.186／0.204 ms 與 0.094／0.106／0.134 ms。
場景編譯 369.51 秒、飛行執行 89.41 秒、整個命令 460.81 秒。
回放不足一秒，只有一個 RSS 樣本，不能推論持續穩定性；長時 bounded replay 見下節。
機載視角被機庫支柱遮擋，也顯示部分碰撞代理，只是軟體資料鏈輸入，不是實測光學設定。

前兩次嘗試分別因應用程式起飛計時邊界與降落預算不足而 **FAIL**；
摘要／ULog 保留。依實際 spool、離地與低空 crawl／land-detector 時序，
改成分別限制解鎖→離地、離地→穩定，並把總時間由 45 增為 55 秒、LAND 由 20 增為 30 秒。
懸停門檻與 PX4 參數未放寬；不是降落加速或舊 45 秒設定已通過。

受影響安全範圍 64 tests PASS、2 個需場景環境變數的檢查在該批跳過、2 個目標／接近案例排除；
其中 rail 隔離另以真實 MJB 執行為 1 PASS，完整場地初始姿態比較仍 NOT_RUN。
通用 M0–M3／控制器／protocol／UDP 104 PASS、
checkout 外已安裝通用影像 wheel 62 PASS。可重現命令及完整限制見兩個元件 README。
Sim／PX4 本輪改動未 commit／push；保留原有未提交研究工作。
視覺導航與真機 NOT_RUN，物理相機、匹配標定及 RK3576 runtime 仍 BLOCKED。

### Sim 場地相機錄製與共用回放（2026-11-02，America/New_York）

本輪實作在 `components/rm_uav_mujoco/`，新增
`python -m rm27_sim record-camera --config configs/camera_recording.json --output NEW_DIRECTORY`。
直接使用 `configs/field.json` 選定的完整 2027 場地，來源 commit `d9439a97`、
manifest SHA-256 `5664e1391c1ba7436769b8bc252ea491b5af5ae765fd9dd89744938f4431aef6`；
145 項 runtime hash 前後核對通過，不抽取或混入舊基地、不修改資產。
只建立私人渲染狀態與相機路徑，不步進物理或接入控制。

共用 `rm27-tracking` 0.3.0 已驗證 wheel 的 `RecordingWriter`、解碼、預處理與 runner，
沒有複製錄製 schema／新增第二套處理 pipeline。640×480、30 FPS 排程、300 幀、
512 MiB 上限，錄製為原始 RGB3；仿真時間 `[0,10)`、主機渲染邊界與回放接收時間分開保存，
沒有以主機減仿真時間推斷相機延遲。容量／儲存／渲染失敗保留不完整證據，禁止覆寫輸出。

本輪主機實測：Python 3.12.3、MuJoCo **3.12.0**、OpenCV 5.0.0、NumPy 2.5.1，
EGL / RTX 5060 Ti / NVIDIA 580.196.07；不冒稱 README 其他 profile 的 3.17.0 同等通過。
`artifacts/camera-recording-002/` 保存 300 幀、276,480,000 bytes、三張預覽及摘要，
全數 RGB→BGR 逐值相等、序號／時間／metadata 保留。模型載入 382.21 秒，
逐幀錄製 3.31 秒／90.59 FPS，整個命令 388.45 秒，RSS 高水位約 4.07 GiB；
啟動代價不能被錄製 FPS 隱藏。

兩次完整 deterministic pass 各 300 幀、1,561.44／1,598.11 FPS，
有序處理像素 hash 均為 `6e97e079279a7ba00775a7a547347e48f6b21fecd7427aa730001534904dfa4a`。
60 秒 paced 循環為 1,801 幀／30.02 FPS，queue drops 0、high-water 1／capacity 2，
preprocess＋process 的 P50／P95／P99 為 0.160／0.251／0.327 ms，CPU 約 3.32%。
RSS 首末增 221,184 bytes，10 秒後介於 56,770,560–56,786,944 bytes；僅短時資源觀察。
證據在 Sim 的 `artifacts/camera-benchmark-{pass1,pass2,paced}/run/`，
各摘要包含版本、設定、輸入與程式 hash，沒有 RK3576 效能承諾。

另先核對共用 catalog 與四個 MP4 的 SHA-256，對
`rm27-real-camera-exposure-01` 的 clip01–04 各做兩次 60 幀回放，共 480 幀。
原檔前後 hash 不變；native 首幀像素符合 catalog，320×240 灰階處理摘要逐檔兩次一致，
queue drops 均為 0。設定、完整 hash、程式版本與命令留於
Sim `artifacts/shared-video-regression/verification.json`。
曝光集合 1500／2500／2500／3000 μs、LCG、1×；逐檔對應未知，沒有同步 IMU 或匹配標定。
這是有界離線資料鏈回歸，**不是 VIO、物理採集、硬體 timing 或實機驗收**。

相關 Sim 渲染／入口／場地回歸 32 項、通用影像既有 62 項測試通過；4 項既有完整飛行場景
檢查因未設定 `RM27_CURRENT_FIELD_SCENE` 為 NOT_RUN。實際 CLI 錯誤分支另補回歸；
完整命令、測試數量及結果以 Sim README 為準。本輪未 commit／push／merge，保留既有修改；
物理相機、實測標定與 RK3576 runtime 仍 BLOCKED，沒有擴大到飛控或馬達輸出。

### M3C 上游與本機路徑同步（2026-11-02，America/New_York）

依使用者要求，本機 checkout 由 `maixcam2_dart_vision` 改名為
`rm27_tracking_m3c`，完整路徑為 `rm27/perception/vision/rm27_tracking_m3c/`。
自 `ac83e8c` fast-forward 到上游 main 的
`c81456a21aac6d2b75410f09081549a4c0ac1aa4`，checkout 維持 detached HEAD、clean，
origin 仍為 `blademaster679/maixcam2_dart_vision`。父倉 gitlink 與 `.gitmodules`
路徑改動已 staged，尚未 commit／push；其他既有未提交改動保留。
PX4 內的另一份 Tracking 副本未變動。本輪僅核對 Git／路徑完整性，未重跑
影像演算法或板端測試；下列既有量測維持各自歷史驗證範圍。

### 獨立通用影像基線（S2 主機驗證，2026-10-29）

`components/rm27_tracking/` 是獨立本地 Git 倉庫；使用者已授權 commit／push，
`main` 的初始 commit 為 `e8a9ac85522c9ed70cb68722c6fec8459be3e653`（本地 clean）。
Author／committer 均核對為 `shiuhou <uhshi371@connect.hkust-gz.edu.cn>`，
origin 已設為 `https://github.com/shiuhou/rm27_tracking.git`。
先前發布嘗試因網路／審批服務問題未完成。2026-10-30（America/New_York）重試成功：
GitHub API 與 SSH 均核實登入 `shiuhou`；HTTPS TLS handshake 失敗後以 SSH push 成功。
GitHub API 確認 remote `main` 為 `e8a9ac85522c9ed70cb68722c6fec8459be3e653`，
與本地相同，child working tree clean；origin 的 HTTPS URL 保留。
本次 commit 只含該子倉庫 20 個 source／config／test／document 檔案，沒有發佈 parent 改動。
S1 的影像／影片回放與 resize／灰階處理已沿用；S2 改用同一共用 runner，
新增 direct V4L2 single-planar MMAP adapter、NV12/NV21 等 stride-aware 轉換、
核心序號／timestamp provenance、重啟 epoch、有限重試、有限佇列、丟幀統計、
每階段 P95、CPU／RSS 與合成影片 benchmark。沒有存取實體相機。
原 OpenCV 相機 adapter 已由取得核心 metadata 的路徑取代，沒有保留兩套 capture 實作。
回放時間仍為 sequence/FPS 合成值；相機時間保留 driver domain／boundary flags，
不宣稱曝光時序或硬體同步已驗證。未支援 multiplanar-only／tiled BSP 節點；
未知／非 BT.601 limited-range YUV 色彩格式會明確拒絕。

**本次驗證**：23 tests 在 OpenCV 5.0／NumPy 2.5.1 和 distro OpenCV 4.6／NumPy 1.26.4
均通過；0.2.0 wheel 建置、checkout 外安裝、相同 23 tests 與三個 CLI 入口通過。
已檢查本機 Linux UAPI 結構／ioctl ABI；這是 x86_64 結果，不是 ARM64 或實板驗收。
640×480 合成 MJPEG → 320×240 灰階＋統計：15 s unpaced 約 **1,437.20 FPS**；
60 s paced 共 **3,601 frames／60.02 FPS**、queue drops 0、P95 preprocessing＋processor
**0.251 ms**、CPU 約 **7.11%**、sampled RSS 最大約 **68.15 MiB**。
後者自第一個 sample 成長 1.23 MiB，10 s 後成長約 0.148 MiB；只是短時資源觀察。
兩個 host benchmark 前 15 s 同時執行，不是隔離效能增益比較，也不是相機 FPS／RK3576 數據。
證據：component `artifacts/s2-throughput/run/summary.json`、
`artifacts/s2-sustained/run/summary.json`（本地忽略生成資料）；
建置／執行／ARM64 native 指令與限制見 `components/rm27_tracking/README.md`。

**未驗證**：無可用 ARM64 sysroot／AArch64 GCC／QEMU 或 RK3576 runtime；
純 Python wheel 不代表板端相機相容。仍須辨識 BSP、sensor、media graph、driver、
實際格式／時鐘／同步、斷線重啟、板端吞吐與熱／記憶體穩定性。
本 component 沒有可供轉換的模型，因此 RKNN backend 暫不引入；CPU pipeline 不需 NPU。
沒有新增目標交戰、裝甲選擇、碰撞制導或攻擊重捕獲功能。

沒有複製 vendor 目標管線；原第三方 submodule 維持
`ac83e8c224214bc99483f9617450e110c5c16022`、既有 origin 與 clean 狀態。
主倉庫 `.gitmodules` 已登記 `components/rm27_tracking` 的 path／URL／main，
但父倉 index 仍無 gitlink；child 遠端 main 已核實可取得；2026-10-31 父倉 index 已建立版本釘選，待 commit／發布。
本次只同步倉庫認知、責任與登記資料，未修改 child 程式、stage、commit 或 push。
先前 `INTERFACE_INTEGRATION_PASS` 的六-worker 快照及重建包保持原驗收範圍，
不包含新 Tracking 管線，總倉 target worker 亦未遷入此 repo。
下文 Tracking 數據仍屬原 MaixCAM2 研究，不得轉作本方新模組或 RK3576 驗證結果。


主要來源是 upstream `IMPLEMENTATION_STATUS_V0.2.md` 與 `GREEN_DETECTION_PLAN_FULL180.md`（2026-09-09）；不是今天重新量測的結果。

已有 CPU 多尺度綠燈／裝甲檢測、LOS Kalman、track/state、`safe_for_control` 邊界；PnP／NPU 接口存在不等於標定／權重驗收完成。

- 480×360@60 的 185 min 紀錄約 60.001 FPS、P95 17.37 ms。
- 1344×760@180 的 30 min 紀錄：324,245 frames、180.135791 FPS，無序號異常；綠燈／裝甲調度約 88.672／59.792 Hz，估計輸出約 179.998 Hz，接收後 source age P95 22.992 ms。
- 自報 direct observation 約 10.433 Hz，沒有標註真值；不能解讀為召回率，也不能說 180 Hz 有效目標檢測。

全視野模式未標定，`fx=fy=1344` 是占位；曝光到控制的完整延遲未知。目前無效角度／control-safe=false 是正確拒絕。高速入口仍是隔離驗證程序，普通 APP >60 fps 會拒絕，NPU/Pose 未接入該高速入口。

2026-11-04 補做本機實測：共用 `rm27-real-camera-exposure-01` 四片的原始檔先後
SHA-256 均符合 catalog，**45,462 幀全部逐幀**通過現有 C++ RGB
`GreenLightDetector::process()`，不是先前僅做解碼／灰階處理的結果。
clip01–04 的新鮮確認像素觀測分別為 3,604/5,505、11,866/17,987、
8,852/10,079、8,716/11,891；預測輸出另計 126／375／167／341 幀。
最長無新鮮觀測分別為 2.494／10.083／2.517／9.144 秒（標稱影片時間）。
這些是輸出可用性，不是 precision/recall 或感測器掉幀。
使用原生 1344×760、frame-step 1、既有 simulation CPU 門檻，關閉未知標定的
capture cone，fx/fy 僅作數值正規化占位；未針對影片調參。
序號、標稱 FPS 時戳及新鮮觀測 metadata 檢查 PASS，角度／位姿／control-safe
均保持 false。帶真值識別品質為 BLOCKED；板端 NV21/NPU、真實飛行為 NOT_RUN。
M3C HEAD `c81456a` 加既有未提交 RGB 工作；完整來源／設定／binary hash、
逐片命令、逐幀 JSONL、P50/P95/P99 與抽查影像留於該元件
`artifacts/shared-real-green-replay/`。兩個影片程序並行，效能只作主機診斷；
未改算法、未操作硬體、未 commit／push。

另抽查 12 張實際畫面：clip01 第3160幀、clip03 第5535幀、clip04 第8681幀，
中央支架上的綠色亮點仍可見，但輸出 `LOST`／候選0，是具體定性漏識別案例；
不能把整段無新觀測區間都算漏檢。clip02 第2112幀朝向側邊空地，支架未入鏡，
故不判為漏檢。接觸表 `qualitative.png` 保留上述畫面及確認輸出對照。

同日先查辨識結果到指令層的接口：45,462 條真片結果原樣通過既有
`measured_pixel` 准入檢查後全部拒絕；四片實際 RGB 首幀也在
`RGBObserver.submit` 的非仿真來源門檻拒絕，未啟動子程序、未排隊。
該次輸入契約保護 PASS，沒有呼叫指令產生／傳輸，不能當作指令輸出通過。
原因包括缺少 adapter 要求的已驗證來源／仿真時戳，
以及未提供匹配內參、安裝標定、同步姿態。未補造仿真 metadata 或姿態。
Sim `artifacts/shared-real-command-admission/summary.json` 保留完整判定和 hash；
該次沒有改動算法／控制器／接口契約，先前 SITL 閉環結果不能代替本項驗證。

隨後在 Sim 新增 `replay-heading-commands` 明確離線入口，直接讀取原始
`pixel_measurement`，重用既有 `pixel_heading` 與抽出的 `rate_limited_yaw`。
**數值假設下的辨識／追蹤→航向輸出軟體驗證 PASS**：四片全部 45,462 筆輸出，
33,038 次新鮮確認觀測；1,009 筆預測不更新觀測，7,799 筆無觀測／過期時保持。
全部檢查方向、有限角度、30°/s 限速、0.3 秒到期與序號／名義時間，
各片再做一次完整回放，指令檔有序 SHA-256 一致；獨立解析公式及精確限速步進再審 PASS。
輸出逐行處理，四片約 25.78 MB，不積存影片。原片、辨識 run/config、23 個
source/binary/header 及原有語意摘要核對後重用，未另跑 detector 或調門檻。

相機 fx=fy=1344 px、中心主點、零畸變是既有 normalization 假設；前向水平
mount 與固定單位姿態也是測試參考。沒有把指令回填為「實測姿態」、把真片改標
仿真，或提高 `safe_for_control`。輸出為假設 local NED 的 yaw radians，
來源仍是 encoded-video／nominal-FPS，不是曝光時戳或已驗證延遲。
**實際飛機指令正確性 NOT_RUN**：需匹配的相機／安裝標定與同步飛機姿態；
沒有 PX4 transport、固件或硬體操作。完整命令見 [Sim README](components/rm_uav_mujoco/README.md)，
假設見 [診斷設定](components/rm_uav_mujoco/configs/real_video_heading_diagnostic.json)，
原始標記與每片結果見 [四片離線航向摘要](components/rm_uav_mujoco/artifacts/shared-real-heading-commands/summary.json)；來源鏈及第二次回放的驗證入口是
`.agents/runs/sitl-camera/check_real_heading_commands.py`。
M3C 的 [實作進度](rm27/perception/vision/rm27_tracking_m3c/docs/IMPLEMENTATION_STATUS_V0.2.md)
已同步此里程碑；原始辨識與下游診斷保持各自驗證範圍。上述 artifacts 僅保留在本機，未納入 Git。
原有仿真接口保護及非接觸 SITL 實驗保持原範圍，不用這次離線結果重新宣稱實機可用。
相關航向／離線測試 110 passed；完整 Sim 回歸 428 passed、5 skipped，147.40 秒，
包含既有 M0–M3 生產矩陣；略過的是未配置的選用場景／完整資產檢查。
命令、程式 hash 及範圍在 Sim 既有
`.agents/runs/sitl-camera/affected-regression.json`。未 commit／push。

同日將上述既有辨識／檔案指令合成[可視化入口](components/rm_uav_mujoco/artifacts/shared-real-heading-visualization/index.html)，
四片完整影片為 [clip01](components/rm_uav_mujoco/artifacts/shared-real-heading-visualization/clip01/tracking_commands.mp4)、
[clip02](components/rm_uav_mujoco/artifacts/shared-real-heading-visualization/clip02/tracking_commands.mp4)、
[clip03](components/rm_uav_mujoco/artifacts/shared-real-heading-visualization/clip03/tracking_commands.mp4)、
[clip04](components/rm_uav_mujoco/artifacts/shared-real-heading-visualization/clip04/tracking_commands.mp4)。
45,462 筆來源仍逐幀對齊，展示每第六幀，共 7,578 幀，保留完整標稱時長。
四片 H.264／1280×800／30 FPS 均完整解碼、檔案雜湊核對 PASS；
[媒體摘要](components/rm_uav_mujoco/artifacts/shared-real-heading-visualization/summary.json)記錄來源與輸出。
綠色為當幀確認，琥珀為沿用未過期觀測，灰色為過期／無觀測保持；
青線為假設下的目標方位，粉線為限速後航向輸出。這是既有離線診斷的呈現，
沒有量測飛機實際響應，也沒有重跑 detector、新增飛行、修改核心算法或 commit／push。

視覺品質第一階段已在 M3C 元件完成工具與資料驗證，原始辨識／追蹤參數維持不變。
新增固定源序號抽樣入口及 green-only／`pixel_measurement` 評估模式，沿用既有 evaluator，
沒有新增處理 runner。核對原片／manifest／既有 23 個 source/config/binary 身分及全部 45,462 行記錄；
128 張均勻樣本（每 360 源幀）及 4 張診斷案例的原生 PNG 均與順序解碼逐像素相同。
AI 查看原圖作近似初標，無 detector 疊圖或跨幀插值，全部仍為 `ai_provisional`。
暫定均勻樣本 100 可見／24 不在畫面／4 不確定；100 可見中 82 確認位置相符、
12 無候選、6 有候選未確認、0 確認位置不符；24 不在畫面中有 5 次新鮮確認輸出。
診斷案例另計，仍為 3 個可見但零候選、1 個不在畫面且無輸出。

優先覆核的具體問題是背景門上方綠色小牌的 5 次確認、可見但無候選，及後驗位置偏移；
例如 clip02/10440 的確認像素誤差為 1.383 px，非預測 tracker 後驗為 52.037 px。
此為不同層輸出的具體差異，沒有據此宣稱閾值／關聯／曝光／濾波根因。
6 個有候選未確認樣本中有 3 個在首幀，也不能全當 tracker 故障。
現有匹配尺度不變；誤差對照近似 AI 參考，不是亞像素真值。
兩次完整稀疏評估有序 SHA 一致，獨立重算 PASS；33 項工具測試在 OpenCV 4.6／5.0 均 PASS，
舊 6 項測試及 HEAD 原版數值差分保持一致。

本機交付為 [原圖覆核頁](rm27/perception/vision/rm27_tracking_m3c/artifacts/shared-real-green-quality/index.html)、
132 張原生 PNG、4 份參考 JSONL、樣本清單及
[單份摘要](rm27/perception/vision/rm27_tracking_m3c/artifacts/shared-real-green-quality/summary.json)，
合計約 178.54 MiB（上限 512 MiB）。操作方式見
[M3C README](rm27/perception/vision/rm27_tracking_m3c/README.md#sparse-visual-quality-references-2026-11-04)。
工具／資料完整性 PASS，正式人工品質評估 BLOCKED、算法驗收 NOT_RUN；
使用者隨後明確選擇略過人工覆核，人工覆核記 NOT_RUN，不再等待使用者操作；
現有 AI 暫定參考及雜湊維持凍結，可作診斷回歸，不能升級為獨立真值。這四片仍缺獨立場景；
稀疏樣本不證明整段追蹤連續性，RGB 回放不驗收 NV21 或硬體速度。
保留所有原始影片、原辨識證據及既有未提交工作，未操作硬體、接入飛控或 commit／push。

待做的設計：多候選各自累積時序證據、全視野多尺度／背景正規化、原生資料與硬負樣本、motion transform 起訖時間、標定及帶真值驗收。單一確認器輪流處理五個候選，不能保證 100 ms 捕獲。下一輪實作先依上述人工覆核結果選定問題與固定比較方法，不以更多輸出直接當作改善。

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

以下是對照新 PDF 的系統級缺口，不替各 repo 安排算法或硬體實驗；具體任務由使用者另行安排。

1. **ASM2 非 RL 線上接觸與恢復**：從有效交接狀態比較固定參考與線上重規劃，保留連續機體與電機狀態。分開量測指定姿態接觸、分離、飛行恢復與估測恢復。先驗證真值和合成估測基線，再驗證由視覺辨識取得的板面相對估測。
2. **支承與接觸件合理性**：實物底部件尺寸、突出量及停放支承需確認，避免靠仿真凹槽與底球假設取得不可轉移的結果。
3. **動力辨識**：以原始 log／參數、推力測試等建立質量、慣量、推力、lag、yaw 模型；區分不可辨識項，不把一次仿真吻合當唯一真值。
4. **RL 新任務契約**：落實原生 Ver2 首次接觸 task/reset/evaluator，再決定有限訓練；保留舊任務結果與失敗，不能用新門檻改舊結論。
5. **導航和目標觀測**：建立新平台 Camera＋IMU 時序、Tracking 幾何及重獲能力；SLAM 局部地圖／重定位可並行研究，量測延遲、偏差及失鎖，不能用 source FPS 代替可用觀測率。
6. **固件整合**：在相同機型與場地用真 PX4，分清 truth、synthetic odometry、實際 VIO 的貢獻；舊 proxy PASS 不替代這一步。
7. **重複性與擾動**：保留所有嘗試、固定門檻、獨立 seed，分開 nominal repeat、paired perturbation 和 recorded replay；分別驗證真機遷移與同架次連續接觸；多機同時協同不列本季主線。

## 13. Git、本地資料與 GitHub 的差距

以下 HEAD 保持原工作區基線；本批檔案及 dirty 狀態以重建包 manifest 為準。dirty 包含修改或未跟蹤項，不能由 HEAD 單獨重現。乾淨只指這次 `git status --porcelain` 輸出為空，不代表分支已合併 main。

| 倉庫 | 分支 | HEAD | 狀態 |
|---|---|---|---|
| root | `astra/research` | `1fc395c433cf4574816047a362e464d6279fc0c9` | dirty |
| RL | `i4/informed-reset-arrival` | `ddf159395d5b1391011a0839732b6b7f5a3807b6` | dirty |
| Sim | `feat/moving-base-target` | `f414b4c7517db7eae79ffee112d60bc45f2ef302` | dirty |
| MJCF | `feat/moving-base-target` | `d9439a97cee80baba9d13a1de9882f2d8095fca5` | dirty |
| PX4 | `docs/progress-handoff-rule` | `539ce7412b98ae67c53bf93c46d2595683d2c38a` | dirty（本批接口／文件） |
| SLAM | `docs/progress-handoff-rule` | `3e94df8a85815ed6042de6d87b6dba9cda14e625` | dirty（本批接口／文件） |
| VIO | `docs/progress-handoff-rule` | `a8392c6f1e32e76532bd3ea8aa34ef98e0f06da6` | dirty |
| rm27_tracking_rk3576 | `main` | `1e59b3631e7cdf73ddf91c10c0e25cddcb5799b4` | 2026-10-31 已核對遠端 main／本地 HEAD 相同、child clean；父倉 gitlink 已建立於 index，待 commit／發布 |
| Tracking 上游 | detached HEAD | `c81456a21aac6d2b75410f09081549a4c0ac1aa4` | 2026-11-02：fast-forward；已改名 rm27_tracking_m3c，clean；父倉釘選 staged 待 commit |

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
| rm27_tracking_rk3576 | `components/rm27_tracking/README.md`、本節 S2 主機紀錄；本輪未重跑 |
| Tracking 上游 | `rm27/perception/vision/rm27_tracking_m3c/docs/IMPLEMENTATION_STATUS_V0.2.md`、`GREEN_DETECTION_PLAN_FULL180.md` |
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
| `components/rm27_RL/.agents/docs/SPEC.md` | `c8f2afd32af9bb9e9414af21ce84add08ef03c853f59092919bd7739f8b9eb0b` |
| `components/rm27_RL/.agents/docs/DESIGN.md` | `0c538ba29d90ded13fcacb9e02cbb7f45a84e081c16aa3a46625f867048ade9e` |
| `components/rm27_RL/src/rm27_contact_rl/diff_policy.py` | `9893a86499b3d68b52a75df3d3817fb36d70fcd8765a049b7ab73a0bdd332f64` |
| `components/rm27_RL/configs/agile_flight/diff_e1a.json` | `61ec71c3675a706ca4f18f9600ce35e438ff5c0445cf8b1234c213f3b7e049c9` |
| `components/rm27_RL/.agents/plans/plan_agile_terminal_arrival_01.md` | `f6f8434cbf6b1da3a8d8b46f16590bd2877ee28e3568586c7541d04391151ae1` |
| `components/rm27_drone_px4/docs/PX4_MIGRATION_CLOSEOUT.md` | `08c32fcca9fb7de15c09b3e5a2c740c4c5fafb0bd97b7c94af19e0d6fd459eaa` |
| `components/rm27_drone_px4/docs/PX4_VER2_TRIAL.md` | `f2ff4409a7b7b6979f0d1e8d7fa8facbc566df351a003be6f7cbcf6135f91cbf` |
| `components/rm27_vio/HANDOFF.md` | `3a08f18caae77088f370e7a81a286d1812615945a1ef0357be3d3b43a373d698` |
| `components/rm27_slam/docs/STATUS.md` | `b98055e3d512de09f0307414ceb462aed869f4dd8de6c90d6de8610b4c839e71` |
| `rm27/perception/vision/rm27_tracking_m3c/docs/GREEN_DETECTION_PLAN_FULL180.md` | `a0ed08c35b46a0b92c02f67762bc7d3a4ebffc00344ad7ec015f0e4ca65ffcfd` |

### 文件 PR 準備時的發布版本核對

本輪從遠端 main 建立獨立文件工作區，沒有把研發工作區整批提交。已重新 fetch 的 main：

| Repository | main SHA |
|---|---|
| root | `8abf971cc26d325090ad5eaa19a18a711ebc7835` |
| RL | `aff12170e0c0c84ce11554fcf49dbeb6a231eb85` |
| Sim | `57e73658186ccee886f8560011413ec3fcf605f3` |
| SLAM | `a6bb60de08a6cea33df9f1bc1db4db4dec22193f` |

關鍵差異：RL main 仍包含 `packages/physics/`，其安裝命令應使用同 checkout 內的套件；本地研發分支才改用外部 Sim Physics。Sim main 已有獨立 `rm27_sim/`，可動基地與 impact 工作另見 PR。主倉庫 main 的 `.gitmodules` 仍指向舊 `shiuhou/rm_uav_mujoco` URL，父 gitlink 也未因本文件 PR 升級；正文新架構描述的是研發工作區和現行所有權，不是聲稱每個 main 已完成同步。不能只把 README 命令換新，就假裝程式與版本釘選已遷移。

## 跨電腦 VIO／SLAM 回收核對（2026-10-31，America/New_York）

本次只讀 Git 歷史、fetch 遠端與檢查證據檔案存在性，未合併、checkout、push 或重跑測試。
可確認的 Windows 工作已回到組件，且已包含在主倉庫 HEAD 的 gitlink：

- VIO：`13c7a77` 明確合併 `/tmp/rm27-vio-windows-20261007.bundle`；後續 `af6e976` 保留 OpenVINS 主線並將單目 SLAM 歸檔。父倉庫釘選 `4bb4842` 包含這些提交。目前 child HEAD `a8392c6` 與此次 fetch 的 main `2cd05cc` committed tree 相同，差別是 merge commit；另有本地未提交研發，不受此結論涵蓋。
- SLAM：Windows 準備工作區及測試見 `components/rm27_slam/HANDOFF.md`、`docs/VALIDATION.md`；`b4a69a5` 匯入程式／小型測量，`11be5a0` 記錄乾淨 clone 的 176 項測試。父倉庫釘選 `d176c64` 已包含兩者。此次 SSH fetch 的遠端 main 已是 `b4cbccb`，相對 child HEAD `3e94df8` 僅多 README 修訂與合併紀錄（含先前文件 PR #2），沒有新增程式／實驗結果；本地尚未更新這份 README。不是今天重新跑過 176 項測試。
- VIO 的 `.maixpy/offline-vio-20261009/OFFLINE_VALIDATION.md` 與 `.codex/tasks/2026-10-13-vio-only/validation-report.md` 本機存在。但 handoff 引用的 `.maixpy/imu-ram-run-20261012/RESULT.md`、`analysis-reviewed.json`、`verification.json` 在該本地位置不存在。SLAM 文件明確說明原始影片、第三方 binaries／SDK 沒有隨 Git 發布。

結論：已知 Windows 階段的程式、整理和測試紀錄已回收；不能宣稱全部原始證據已搬回，也不能確認另一台電腦之後尚未 commit／push 的內容。尚未直接存取另一台電腦，且此核對不代表 VIO／SLAM 真實飛行整合驗收。

## 硬體選用更正（2026-10-31，America/New_York）

使用者明確指出羅盤也不使用；目前導航方案同時排除 GNSS 與羅盤。已同步 `configs/real_hardware.json` 的用途、`compass_navigation_enabled=false` 與來源，以及 `docs/REAL_HARDWARE.md`。M10G-5883 型號保留作歷史清單，不代表仍選用；未確認實物是否拆除或固件是否停用，本次未操作硬體或修改飛控參數。歷史 log 與凍結 baseline 保留原有證據範圍。

## 電池與最新參數表接收（2026-10-31，America/New_York）

使用者提供電池規格 **2S／520 mAh／95C**，品牌、重量及實測電壓仍未知；95C 是標稱資訊，未量測放電能力。收到目前最新 `1031.param`，961 個唯一參數，原檔保存於忽略目錄 `data/flight_logs/data/2026-10-31_parameter-intake/1031.param`；SHA-256 `55033df048b9a8c38305c82faa70d48ff6a82ee611f9eb28f2b0b9f6bb074219`。已同步共用硬體 JSON 與硬體說明。

檔內 `BAT1_N_CELLS=2`，**`BAT1_CAPACITY=470` 是使用者確認刻意採用約 90% 標稱容量的保守設定**（520 × 90% = 468 mAh，配置取 470 mAh），不是待修正錯誤或實測可用容量；電池標稱仍是 520 mAh。未修改參數或寫入硬體。檔內還記錄 `EKF2_GPS_CTRL=0`、`EKF2_MAG_TYPE=5`、`SYS_HAS_GPS=0`、`SENS_BOARD_ROT=0`；僅能證明收到的表格值，完整匯出、對應飛機、固件 build 與當下設備狀態仍待核對。本次只有原檔 hash／格式檢查，沒有飛行或硬體測試。

## Tracking 發布狀態核對（2026-10-31，America/New_York）

使用者回報推送 `1e59b36`。本次以 SSH `git ls-remote` 核對
`shiuhou/rm27_tracking` 的 main 為 `1e59b3631e7cdf73ddf91c10c0e25cddcb5799b4`，
與本地 HEAD 相同，child `git status --short` 無輸出。
提交標題為 `Prepare reproducible camera data and ARM64 validation`，包含錄製／校準工具、
平台驗證腳本、相關測試與說明；本次只核對提交及發布狀態，未重跑測試或驗證 ARM64／RK3576 實板。
父 repo HEAD 與 index 均尚無 `components/rm27_tracking` gitlink，故不能說已由主倉庫釘選；
本次僅更新總覽，未操作 child 或 parent 的 stage／commit／push。

## Tracking submodule 釘選建立（2026-10-31，America/New_York）

依使用者指示，父倉 index 已加入 `components/rm27_tracking` 的 mode 160000 gitlink，
釘選 `1e59b3631e7cdf73ddf91c10c0e25cddcb5799b4`，並暫存其 `.gitmodules`
path／GitHub URL／main 登記。保留 `.gitmodules` 其他既有未提交修改，未混入暫存。
本次未建立 parent commit 或 push；目前 GitHub 上的父倉版本尚不包含此新釘選。
先前「父倉尚無 gitlink」是建立前的歷史核對結果，以本節狀態為準。

## Tracking 父倉釘選發布（2026-10-31，America/New_York）

依使用者 push 指示，父倉提交 `651f8a2` 已推至 `shiuhou/rm27_drones` 的
`astra/research`。提交只包含 Tracking 的 `.gitmodules` 登記與指向
`1e59b3631e7cdf73ddf91c10c0e25cddcb5799b4` 的 gitlink；其他本地改動保留。
此狀態取代上一節「待 commit／發布」；尚未合併到 main。報告本身此次只在本地更新。

## Tracking 已同步 main（2026-10-31，America/New_York）

使用者澄清目標為 main。從最新遠端 main `f37cabc` 的隔離工作區僅 cherry-pick
Tracking 登記與 gitlink，產生 `d77fca0` 並正常 fast-forward push 到
`shiuhou/rm27_drones/main`。遠端 main 已核對；Tracking 釘選仍為
`1e59b3631e7cdf73ddf91c10c0e25cddcb5799b4`。先前「尚未合併 main」已由本節取代。
原研發工作區與其他未提交修改保留，本次報告狀態更新僅在本地。


## RK3576 Tracking 更名（本輪核對）

依使用者指示，GitHub 原倉庫已更名為 `shiuhou/rm27_tracking_rk3576`；
API 核對 repository ID 保持不變，main 仍為
`1e59b3631e7cdf73ddf91c10c0e25cddcb5799b4`，沒有新增提交或重寫歷史。
本機整個獨立倉庫移至 `components/rm27_tracking_rk3576/`；origin、父倉
`.gitmodules` 與本地 submodule 設定已同步。父倉 index 保留原版本釘選，僅移動
本元件 gitlink 與登記名稱；其他既有 staged／unstaged 改動保留。
本輪 README／AGENTS／整合引用修改尚未 commit／push；父倉路徑更名亦待發布。

Python distribution `rm27-tracking`、import `rm27_tracking` 和四個 CLI 名稱
保持相容，不另建重複套件。搬移後以 OpenCV 4.6 與 5.0 各跑 62 tests 全通過；
wheel 建置、checkout 外安裝、四個已安裝 CLI 的 help 全通過。
這是主機回歸與更名驗證，沒有新增 RK3576 硬體驗收結果。

當時使用者確認與 VIO 相同，M3C 與 RK3576 分開研發（此決定已於 2026-11-11 退役）；當時已寫入父／子倉庫
AGENTS、README 與責任／路徑表。此對話的通用影像準備由 RK3576 元件承接；
M3C 在 `rm27/perception/vision/rm27_tracking_m3c/`，保持既有第三方 upstream。
兩平台設定、錄製、標定與硬體證據分開管理，不因同用 OS04A10 而互認驗證。

兩條 tracking 線均執行 `git pull --ff-only origin main`，皆回覆 already up to date：
RK3576 為 `1e59b3631e7cdf73ddf91c10c0e25cddcb5799b4`，M3C 為
`c81456a21aac6d2b75410f09081549a4c0ac1aa4`。本輪沒有拉入新程式變更，
保留所有既有未提交內容與暫存更名，沒有 commit／push。
無相機存取的 host platform probe 通過，兩個相機 example config 以正式
configuration 型別驗證通過；主機仍無 video/media 節點，SDK 目錄仍只有聯絡 FAE
的 README。沿用上一輪主機回歸結果，不重跑完整影片效能或宣稱新硬體通過。
真機準備依 RK3576 README 現有流程：辨識板／BSP／media node、短時原始 payload
錄製、精確回放、實測標定與長時穩定性。實際板版號、BSP 與 OS04A10 驗證仍 BLOCKED。


## 共用真實相機影片入庫

使用者提供 Projects/ 頂層四段不同曝光的真實無人機相機 MP4。
已整理至 `data/vision/real_camera_exposure/raw/`，保留原始檔名、inode、大小、
mtime 與位元組；Projects 原路徑保留相對 symlink，沒有重複複製或重新編碼。
權威索引：`data/vision/real_camera_exposure/manifest.json`；使用說明：
`data/vision/README.md`。Tracking M3C／RK3576、VIO 主工作樹／M3C、SLAM、
PX4、RL、Sim、MJCF 與總倉共十個 checkout 均有 `data/shared_vision/` 入口，
已更新各 README／AGENTS。raw/ 忽略，metadata 可單獨版本管理；未 commit／push。

使用者確認曝光值整批為 1500、2500、2500、3000μs，LCG、1×gain；已忘記檔名
對應，逐檔曝光值保持未知，不依亮度或字典序指定。感測器／鏡頭／平台與收到前的
轉傳轉碼歷程未確認。未隨附 IMU、匹配標定、曝光時間戳、感測器序號或真值，
可測影像資料流，不能直接當作合格同步 VIO 資料或硬體 timing／精度驗收。

本輪驗證：原始四檔先 SHA-256 再分析；合計686571250bytes。OpenCV5.0/FFMPEG
回報1344×760、H.264、標稱180FPS，順序完整解碼45462幀（5505／17987／10079／11891），
每段讀取數與回報數相符，每幀尺寸一致。搬移後 hash 再驗證；四段各通過既有
rm27_tracking 0.3.0 runner 的8幀回放與320×240灰階處理。十個 checkout 均可讀同一
physical files；原路徑aliases、raw Git排除與diff whitespace檢查通過。
這是 host encoded-video intake/replay，沒有執行 VIO 後端、訓練、硬體或新增真機性能結論。
