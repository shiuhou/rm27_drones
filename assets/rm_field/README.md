# RM 場地 CAD 資料（RMUC 2025）

本目錄是 RMUC 2025 場地 CAD 檔的**指標與說明**。原始檔因為太大
（265 MB）**不進 git**，放在 repo 外：

```text
~/rm_uav_lab/cad_raw/RMUC 2025复活赛&全国赛_asm.stp
```

## 檔案資訊（從 STEP header 讀出）

| 項目 | 值 |
|---|---|
| 內容 | RMUC 2025 復活賽 & 全國賽場地 assembly |
| 內部名稱 | `RMUC2025_ID_0920_ASM` |
| 來源軟體 | Creo Parametric (PTC) |
| 建立日期 | 2025-06-13 |
| 大小 | 265 MB，約 2250 個 solid bodies |
| **單位** | **mm**（`SI_UNIT(.MILLI.,.METRE.)`）→ 轉檔時要乘 **0.001** |
| 座標原點 | 未驗證——轉檔時必須先確認原點與朝向再使用 |

## 用途規劃

- 主要：之後做 Gazebo 場地模型（配合 ArduPilot SITL）。
- 次要：未來 MuJoCo 若要 RM 場地幾何可共用同一份原始檔。
- **目前不接入任何模擬**；M0–M4-A 驗證 baseline 不依賴場地。

## 真正要使用時的處理清單（到那個 milestone 才做）

1. 用 FreeCAD / CadQuery / OpenCASCADE 把 STP tessellate 成 mesh。
2. **縮放 0.001**（mm → m），並驗證原點與 +Z 朝向。
3. 拆成兩份：
   - 視覺 mesh：DAE（Gazebo 慣用，可帶材質）或 STL，抽稀到合理面數。
   - 碰撞體：低面數簡化 mesh 或直接用 box/plane primitives
     （整顆 265 MB CAD 當碰撞體會讓接觸解算又慢又不穩）。
4. Gazebo 側產出標準 model 包（`model.config` + `model.sdf` +
   `meshes/`），放 Gazebo 工作區；本 repo 只保留原始檔指標。

## 規則

- 原始 STP **永遠不修改、不覆蓋**；所有處理產物另放。
- 檔名含空白、中文與 `&`，shell 操作記得加引號。
