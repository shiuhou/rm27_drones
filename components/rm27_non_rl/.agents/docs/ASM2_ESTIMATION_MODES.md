# ASM2 估測與恢復能力契約

整合仿真擴充：`CapabilitySupervisor.evaluate(..., simulation_execution=True)` 可以在隔離的原生模擬器中，依 `usable` 的合成通道進行控制研究。它仍保留所有來源／frame／timestamp／epoch／clipping 檢查，維持 `trusted=False`、`EVENT_INTEGRITY_UNSUPPORTED` 和明示 `SIMULATION_ONLY_UNCALIBRATED_EXECUTION`；不產生偽造 approval，也不改預設正式路徑。此選項不是硬體執行許可。`asm2_integrated.FlightRuntime` 消費判定並輸出命令；supervisor 本身仍不輸出命令。

新的運動對準從已送達 IMU＋external p/v 歷史推得世界框架旋轉，與舊帶噪真值姿態初始化分開。`world_alignment_observed` 表示有感測運動解，並不等同 `world_attitude_trusted`。撞後 invalid 平移及無 covariance 的限制繼續存在。排程分離是參考動作，沒有把計劃時間轉成確認的接觸事件；其結果必須與感測觸發分列。

本文定義可選 supervisor 的輸入與模式判定。它不估測狀態、不產生 CTBR／電機命令，也不替換既有控制器、偵測器、接觸評分或安全停止規則。既有 `RecoverySensors`、`EventAwareRecovery` 和 Brief Normal Release 各自保留原輸入契約。模式名稱描述當下可支持的工作，不能當作已實作控制律、穩定性保證或可達集。

## 能力與時鐘

每個狀態通道分別提供值的有限性、取得時間、送達時間、來源、座標系、適用 epoch，以及明確故障。量測須滿足 `acquisition <= delivery <= query` 和該通道已登記的有效期。外推或送達不會改寫取得時間。宣告的固定傳輸時程屬模型，不能標為實測。

姿態能力分開記錄機體角速度、重力方向與完整世界姿態。重力方向可用不代表世界航向可用；完整世界姿態需要聲明其世界座標定義與對準來源。平移能力分開記錄世界或表面相對的位置與速度，不能只因數值維度相同就跨 frame 使用。表面法向還需單位長度、指向分離側的符號、座標系、時間及來源。以世界法向控制時，必須具備該世界系的姿態；以機體／相對法向控制時，需要明確可用的轉換。重力方向不能替代表面法向。

接觸證據包含事件區間、偵測取得與送達時刻、來源及事件因果假設。接受新的 impact hypothesis 時，supervisor 開啟新 epoch，將先前平移樣本標為事件前或歧義。區間內樣本也不確定屬於事件後。只允許取得時間嚴格晚於事件上界、已送達且有效的樣本建立新的事件後平移能力。延後送達的舊樣本不能穿越 epoch；若將來要跨 epoch 傳播，須另有已驗證模型和完整性契約，目前不能自行補上。事件後樣本也可能仍在持續接觸中。

誤觸假設被否定時，不能把舊資料改時間戳後當作新樣本。恢復原能力仍須有明確的假設解除證據、適用 epoch 和通道驗證。新的接觸不得清除既有禁止接觸歷史或重設恢復截止時間。

## 資料來源與信任

| 來源標籤 | 意義與限制 |
|---|---|
| `GROUND_TRUTH` | 求解器／物理真值，只供已聲明的 oracle 對照或離線評分；不得進入此 supervisor 的 runtime capability，也不得補齊缺失狀態。 |
| `TRUTH_GENERATED_SYNTHETIC` | 真值驅動的感測模型或帶噪聲、延遲的外部量測；不是實際感測證據。 |
| `RECONSTRUCTED_SYNTHETIC` | 從已保存物理狀態／命令重播生成的合成序列，須記錄重建方法與雜湊。 |
| `REAL_SENSOR` | 實際儀器量測，仍須提供取得／送達時間、標定、座標和健康證據。名稱不保證可信。 |
| `IMPLEMENTED_VIO` | 已實作的視覺慣性估測輸出，須標明實際或合成影像／IMU、尺度／frame、重定位和失效條件。此標籤不等於可靠性資格。 |
| `DECLARED_MODEL` | 已聲明模型的預測或暫態命令假設，不能假裝是量測、後驗 covariance 或完整性證據。 |

能力的 `usable` 只描述有限性、時間、frame、epoch 和已知故障檢查通過。`trusted` 必須另有與所需通道／模式相符的獨立完整性證據，並明確涵蓋接觸後工作條件。沒有該證據時結果為 `UNSUPPORTED`。不存在 covariance 時記為不存在，不用有效旗標、設定噪聲或事後真值誤差偽造 confidence。若有 covariance，也不能單憑其存在宣稱校準或完整性。

現有合成流程的姿態依賴一次帶噪聲的真值初始對準，此後積分合成濾波陀螺。平移是由真值產生、加入噪聲與延遲的外部 p/v，沒有已實作的視覺估測。這些來源及假設在所有模式保留。`RecoverySensors` 尚無經校準可信度或獨立完整性判定，因此不能自行進入「trusted estimation」資格。

## Estimator modes

模式是由現有能力與缺口判定的狀態，可以跳過或重疊。例如姿態可用時可同時執行 translation reacquisition 和 estimate validation。不得為了走完順序而先丟棄可用能力，也不得把經過某模式當作下一模式的充分證據。

| Mode | 所需觀察與進入條件 | 離開／轉換條件 | 失敗與明確限制 |
|---|---|---|---|
| Normal estimation | 當前 epoch 的所需狀態具有有效時間、frame、來源和健康旗標，尚無新的 impact hypothesis。 | 接受接觸疑慮時轉入 impact suspected；個別通道失效時保留仍可用的能力並報缺口。 | Freshness 不代表衝擊後精度或 trusted；不能預先保證事件不會破壞估測。 |
| Impact suspected | 收到有效時間區間的接觸假設或明確感測故障證據，保留其來源和誤觸可能。 | 經獨立證據否定假設後重新驗證；接受接觸後進 post-impact transition。可同時保留角速／姿態能力。 | 事件不確定或晚到必須呈現；不得以求解器時間偷偷縮小 IMU 事件區間。接受新假設即隔離事件前平移。 |
| Post-impact transition | 新 epoch 已建立，盤點接觸後角速、姿態、平移和表面關係是否仍適用。 | 依實際能力直接進 attitude-only、translation reacquisition 或 estimate validation；完整有效能力可跳過不必要階段。 | 沒有新的 post-event 取得時間就不能恢復平移能力；不得把舊樣本延遲送達解讀成新的估測。 |
| Attitude-only | 有可用的 body rates 與已聲明的姿態能力，但平移缺失或未通過新 epoch 適用性。需區分只知重力方向和完整世界姿態。 | 平移樣本到達後可並行 reacquisition／validation；姿態失效則撤銷依賴它的恢復模式。 | 不產生虛構 p/v；不代表既有 Brief Normal Release 已能在此輸入下執行。陀螺長缺訊後不得靜默真值重對準。 |
| Translation reacquisition | 明確知道所需平移來源、frame、取得／送達時刻及缺失原因，等待或接收新 epoch 的適用樣本。 | 有效且取得晚於事件上界的樣本可交 estimate validation；仍保留通道來源及延遲。 | 有效期／既定等待上限到期、來源丟失、frame／尺度不明或數值失效皆明確報告；不新增隱藏等待期限。 |
| Estimate validation | 對已取得的狀態檢查時間、epoch、frame、健康與獨立完整性條件，記錄所用證據及未覆蓋條件。 | 通過基本檢查則開放相應 `usable` 能力；只有獨立完整性證據也合格才能宣告所需能力 trusted。失敗退回對應缺口。 | 不可用 controller 成功與否調整估測門檻；沒有 covariance／完整性證據時須說明未支援。 |
| Trusted estimation | 所需通道各有適用且獨立的完整性證據，涵蓋當前 epoch、frame、時效和接觸後工作條件。 | 任一必要證據過期、失效、frame／epoch 改變或新 impact hypothesis 都撤銷相應 trusted 能力。 | 目前合成 validity 旗標不足以進入此資格；不可從模式名稱、VIO 標籤或事後小誤差推得。 |

## Flight／recovery modes

下表為模式所需能力，並不新增 actuator sender 或控制器。Supervisor 只回傳允許集合、選中模式及拒絕原因。即使高層模式能力足夠，實際執行仍須通過被呼叫既有控制器的完整輸入與安全閘門。尚無對應 executor 的模式不得標為已執行。

| Mode | 所需觀察與進入條件 | 離開／轉換條件 | 失敗與明確限制 |
|---|---|---|---|
| Contact response | 接觸假設、其時間區間和所需感測／致動器能力已明示。可先作模式與記錄決策。 | 根據表面、姿態、平移能力轉入可支持的分離／穩定模式，或報輸入不足。 | 模式進入不等於已發命令，也不等於已偵測到真實首次接觸；不能忽略事件 transport／control tick 延遲。 |
| Separation | 分離法向和方向有適用來源；姿態與法向可放在相同 frame；致動器可用。執行 Brief Normal Release 還需其完整 p/v／R／body-rate 輸入與有效期。 | 以有來源的 clearance／contact 證據確認分離，並依能力可同時穩定姿態或減速。 | 沒有可信表面法向時不能用重力方向替代。模型 normal release 只能按原有限暫態契約使用，不代表分離已成功。 |
| Attitude stabilization | Body rates 及控制律所需姿態／重力方向可用，frame 與 actuator contract 一致。 | 有來源的姿態及角速判準合格後可開放依賴它們的模式；平移重取得可同時進行。 | Gravity direction 可用不代表 world yaw 可用；若既有 executor 需要完整姿態／平移，仍不得以低維能力強行執行。 |
| Vertical recovery | 可用的重力／垂直方向、該方向速度及需要時高度，與安全邊界 frame 相符；致動器與既有 executor 可用。 | 垂直運動達到原登記判準後可保留保持／轉入速度制動。 | 沒有垂直估測不能由 commanded thrust 推定高度恢復；本模式不新建 observer 或 controller。 |
| Velocity braking | 與制動目標相同 frame 的 post-event 速度適用，姿態、致動器和必要表面／安全資訊有效。 | 速度滿足原登記判準時可進 position hold；缺位置時不強行建立位置錨點。 | 延遲 pre-impact v 不可作新 epoch 制動初值；速度有限／fresh 不等於準確或可信。 |
| Position hold | 當前 epoch 同一 frame 的位置、速度、姿態和已明示位置目標可用，既有 executor 及安全條件合格。 | 按原判準報穩定保持，必要能力失效即撤銷；新事件重新盤點能力。 | 不藉重置機體／電機、改寫 frame 或清除接觸歷史取得成功；進入模式不代表已完成 hold。 |

可用模式可以並行，例如 separation 與 attitude stabilization、vertical recovery 與 velocity braking。選中模式需有聲明的優先規則與理由，不能用單一固定順序隱藏能力不足。Supervisor 不自選新的持續時間、等待時間、增益或容差，全部沿用已聲明配置及既有控制器契約。必要能力失效時，輸出拒絕／停止理由；實際安全處置仍屬原有 executor，不能由空模式回傳隱式新增控制命令。

## Supervisor 輸入輸出與失效所有權

建議輸入包含通道時間／frame／epoch／來源、姿態和平移能力、表面法向能力、接觸區間、致動器健康及所需模式的獨立完整性證據。輸出包含 estimator／recovery 允許集合、選中模式、各通道 `usable`／`trusted` 判定、缺口、transition 原因與 evidence IDs。兩類模式為不同維度，不能合併為一個隱式階段計數。

每次判定只使用當下已送達資料。不可因 supervisor 的模式名稱授予原先不存在的測量、忽略過期、擴大 actuator 權限或取用 runtime ground truth。Solver-derived GT、truth-generated synthetic、重建合成、實際感測及 VIO 證據分開報告。缺輸入、未嘗試控制、已發命令後控制失敗、估測完整性不足與接觸安全先前已失效各自保存，不能用後來的 mode transition 改寫較早原因。


`rm27_sim/experiments/asm2_estimation_integrity.py` 擁有可選的純能力判定，透過 `StateSample`、`ImpactEvent` 和 `CapabilitySupervisor` 收取明示的時鐘、frame、來源及 epoch。獨立完整性報告由呼叫端審查後，按 evidence ID、quantity、frame、epoch、source 五項共同綁定；單獨重用 ID 不會授權其他通道或世代。Synthetic／reconstructed／model 即使具有這種登記仍只屬 diagnostic，不能獲得實際感測 trusted 資格。模組不驗證外部報告內容，也不產生 covariance。

Supervisor 輸出的允許模式與 `brief_normal_release_eligible` 只是能力判定，實際執行還需通過既有 reference／controller 的幾何及數值前置條件。測試中手寫的 `REAL_SENSOR`／VIO 及 approval tuple 只驗證軟體契約，不能作為研究中的感測／估測完整性證據。安全失敗會跨 impact、confirmation 和 explicit reset 保留；reset 另要求姿態重新取得或對準證據，不能靜默恢復舊姿態能力。


取得／送達時鐘與估測所代表的 state time 分開。帶延遲的量測即使具有適用的獨立完整性報告，也可能只對其歷史取得時刻可信。用於當下控制時，運動學通道須明確代表目前 query time；若由歷史量測傳播到當下，必須聲明 propagation，保留原 acquisition 和年齡，並由相應完整性報告涵蓋該用途。單獨更新 state time 不建立新的量測或新 epoch。靜態表面資料可按其獨立有效期驗證，不必假造即時取樣。

Supervisor 的 control-ready 判定與歷史量測的 `trusted` 分開。trusted estimation 模式及 flight/recovery 的運動學 gate 使用當下對齊的能力；不能因一筆歷史 p/v／姿態量測可信就直接控制。Brief Normal Release eligibility 另外保留既有 reference 的法向幾何前置條件，包括能定義其水平退避方向；一般單位世界法向合法不代表該 executor 支援全部方向。


## 研究來源與實作邊界

Faessler et al. (2015), *Automatic Re-Initialization and Failure Recovery for Aggressive Flight with a Monocular Vision-Based Quadrotor*，DOI `10.1109/ICRA.2015.7139420`（[作者機構原文](https://www.ifi.uzh.ch/dam/jcr%3A9d7d6f7f-19a1-45db-a1fb-0d78cb0f7f82/ICRA15_Faessler.pdf)，§II.E–F、§III、§IV），明確分成五階段：**Attitude Control → Attitude and Vertical Velocity Control → Attitude and Height Control → Velocity Control → Position Control**。其垂直階段依賴下視測距與 IMU 融合，水平速度及位置階段依賴視覺慣性初始化；所需能力已可用時可跳階。最初姿態階段在缺少高度／垂直速度時使用固定、按質量正規化的重力補償 collective，但不能據此斷言 ASM2 在任意衝擊後具備有效姿態或安全推力。其投擲／自由落體偵測也不是接觸起點偵測。ASM2 尚未實作該文的高度濾波器、視覺流程或五階段控制器。

Liu and Karydis (2021), *Toward Impact-resilient Quadrotor Design, Collision Characterization and Recovery Control to Sustain Flight after Collisions*，DOI `10.1109/ICRA48506.2021.9561089`（[NSF 原文](https://par.nsf.gov/servlets/purl/10343063)，§III–VI），使用柔順機臂及 Hall 感測器取得碰撞變形，待變形峰值後接近機臂原長時啟動恢復；等待期間原飛行控制器仍計算推力，不能稱為零推力 idling。其恢復是依賴初始位置／速度及外部狀態回饋的**單階段 minimum-snap 參考**，並非上述五階段方法。ASM2 沒有相同的機臂／Hall 事件；目前同狀態參考與 CTBR 連續性檢查只對應介面需求，不是該文 minimum-snap 演算法的實作，也不能證明 IMU-only 平移恢復。

Li, Ma, Wang and Sun (2020), *Reachability-Based Collision Recovery Strategy of a Quadrotor*，DOI `10.1016/j.ifacol.2020.12.2415`，*IFAC-PapersOnLine* 53(2), 9440–9445，書目已由出版者登記的 Crossref 資料核對。**原文方法審查為 BLOCKED**：IFAC 原文主機無法連線，出版者替代入口回覆存取限制。尚未核實其 idling 條件、狀態可用性及可達性模型／目標／控制／擾動假設；因此本 supervisor 不宣稱實作該文方法或具有已驗證可達集。任何等待／失推試驗須先界定致動器響應、間隙及估測有效性，這是本地工程要求，並非已核實的該文結論。

以上文獻僅支持能力、事件時間與介面依賴的審查；本 supervisor 不實作其控制器或估測器，不移植數值門檻、時長、接觸參數或成功率。小於控制週期的禁止接觸與電機響應仍須由本地物理證據判定，不能被模式切換消除。原文、文字抽取、出版資料、取得失敗紀錄及 SHA-256 保存於本地 `artifacts/asm2_r22/literature_01/primary_review.json` 所列產物；該產物不是實際感測或本機恢復成功證據。


## 量測、傳播及事件完整性分開授權

`StateSample` 的 `integrity_id` 只對應量測完整性。若 `state_time_s != acquisition_s`，單獨設定 `propagated=True` 或把 state time 改成 query time 不會取得 propagated-state 的 trusted 資格。還須明示 `propagation_method` 與 `propagation_integrity_id`，並在 `approved_propagation` 登記獨立審查的 `PropagationApproval`，共同綁定 evidence ID、method、quantity、frame、epoch、source 及精確 acquisition → represented-state 時間端點。它不延長量測有效期、不救回事件前平移，也不由本模組創造 propagation、covariance 或 confidence。`Capability.measurement_trusted` 與 `propagation_trusted` 分開呈現；`trusted` 及當下 control-ready 須同時滿足所需範圍。

`ImpactEvent.confirmed=True` 只宣告呼叫端接受該假設，不能自行授予接觸事件完整性。Contact response、separation 及 Brief Normal Release eligibility 另外要求 `REAL_SENSOR`／`IMPLEMENTED_VIO` 來源、明示事件 `integrity_id`，以及 `approved_events` 中共同綁定 evidence ID、event ID、lower／upper／delivery 時刻與 source 的 `EventApproval`。來源為合成／重建的事件即使帶有同名報告仍不能取得此 runtime 事件資格；仍可開啟診斷 epoch、撤銷舊資料適用性並顯示 impact suspected。既有姿態／速度等能力不因事件資格缺失而被偽造或清除，其他模式仍須逐項通過原能力 gate。

兩類 approval 都是呼叫端提供的獨立報告審查邊界，不是本模組對報告內容的驗證。手寫的測試 report、propagation method 及完整性登記仍只屬人工軟體契約案例，不能作為實際感測或無特權恢復證據。

## 實際感測估測輸入

`python -m rm27_sim.experiments.asm2_estimation_replay --sensors <sensors.jsonl.gz> --output <新目錄> --mode external_aided` 使用M3 predict及感測取得時刻的互補更新，而非延遲真值包裝。`--mode imu_only`不讀外部p/v。runtime入口沒有truth/model參數；離線評估另讀真值。尚未觀測的初始姿態、未知世界航向、缺失／裁剪IMU與撞後平移重取得都保留為明確能力缺口。即使本地感測傳播正確，未觀測的local→world registration與不存在的完整性證據仍不能由adapter提供。此入口不接入恢復executor。
