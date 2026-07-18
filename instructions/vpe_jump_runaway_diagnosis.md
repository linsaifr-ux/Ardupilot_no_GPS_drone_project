# VPE 位置跳動導致飛機暴衝 — 問題診斷報告

> 日期:2026-07-17(診斷)/ 2026-07-18(6-1 已實作並離線驗證,實飛驗證待做)
> 狀態:**6-1 slew limiter 已實作**(`control/vpe_slew.py` + commander 接線,離線驗證見第 8 節);其餘方案未做
> 相關檔案:`control/ardupilot_commander.py`、`control/vpe_slew.py`、`anyloc/ros2_node_vo_primary.py`、`control/real_hw.parm`

---

## 1. 問題現象

- Jetson 視覺定位器(AnyLoc + VO)輸出的位置有時會**跳動**(例如突然往右偏移數十公尺)。
- 飛控(ArduPilot EKF3)收到跳動後,認為飛機「被吹到右邊」,於是**猛烈往左修正**。
- 修正速度**遠超過 `WPNAV_SPEED` 設定值**,看起來像暴衝。
- 修正飛行本身又讓誤差越來越大,形成惡性循環。

## 2. 資料流(現況)

```
plan-B 定位器 (ros2_node_vo_primary.py)
    │  寫入 anyloc/latest_estimate.json
    ▼
commander VPE 執行緒 (ardupilot_commander.py, 5 Hz)
    │  直接把最新估計原封不動發佈
    ▼
/mavros/vision_pose/pose_cov → VISION_POSITION_ESTIMATE
    ▼
ArduPilot EKF3 (EK3_SRC2_POSXY=6 ExternalNav)
    ▼
位置控制器 (PSC_NE_*) → 姿態 → 飛機動作
```

## 3. 根本原因分析(三個環節疊加)

### 3-1. 修正量以「瞬間階躍」進入 EKF,而且被高度信任

- Commander 發佈的 covariance 是 `max(1.0, err_m²)`(`ardupilot_commander.py:428`)。
  `err_m` 平常很小,所以 EKF 收到的是「誤差約 1 m」的量測 → 幾百毫秒內就完全跳過去。
- `err_m` 本身是「估計值與 GPS 的距離」——**真正 GPS 被干擾時 GPS 會凍結,這個數字毫無意義**
  (已知地雷,見 memory `anyloc_vpe_errm_gate_landmine`)。
- `real_hw.parm` 中 `EK3_GLITCH_RAD=50`:50 m 以內的跳動會被 EKF **融合(fuse)而不是拒絕**。

### 3-2. plan-B 的防跳閘門只限制「單步」,不限制「連續序列」

- 單次接受的修正量 = blend 0.4 × jump gate 45 m ≈ **最多一次 18 m**。
- 連續多次 accept 往同方向拉 → 疊成階梯,控制器永遠追不完。
- **RE-ACQUIRE(重新定位)是刻意的一次性全量跳躍,完全沒有上限**
  (`ros2_node_vo_primary.py:207` 附近)。
- 另外注意:`launch_real_hw.sh` 啟動的仍是 **plan-A 節點(完全沒有 jump gate)**;
  只有 `full_run.sh` 用 plan-B。每次飛行前要確認實際跑的是哪一套。

### 3-3. 控制器全力追趕,反過來毀掉感測 → 正回饋發散

- 快速修正飛行 → 動態模糊、光流位移過大 → LK-VO 與 AnyLoc 比對品質下降。
- VO 會如實積分「修正飛行」的移動量,所以估計值跟著飛機跑,
  控制器等於在追一個會後退的目標 → 誤差持續擴大。
- 一次錯誤的 AnyLoc accept 就足以啟動這個循環;激烈的控制反應讓它持續。

## 4. 為什麼 `WPNAV_SPEED` 管不住暴衝速度

這是關鍵觀察(2026-07-17 實際目擊:飛機猛烈往左修正,速度遠超 WPNAV_SPEED):

1. **`WPNAV_SPEED` / `LOIT_SPEED` 只限制「規劃軌跡」的速度**——控制器打算飛多快去目標點。
2. 位置跳動被控制器解讀為**外部擾動**(像被陣風吹走)。擾動修正**不受導航速度限制**,
   直接輸出傾角,上限只有 `ANGLE_MAX` 和 PSC 增益。25–30° 傾角 ≈ 5 m/s² 加速度,
   看起來就是暴衝。
3. **幽靈速度尖峰(phantom velocity spike)**:EKF3 融合位置階躍時,
   位置-速度交叉共變異數會把速度狀態一起拖走——濾波器不只認為飛機「位移到右邊」,
   還認為它「正以數 m/s 往右移動」。控制器於是同時修正一個位置誤差和一個
   **從未實際存在的速度**,反應加倍猛烈。
4. 此現象被自家設定放大:`EK3_SRC2_VELXY=0`(因為 vision_speed 循環回饋問題而關閉,
   本身是正確的)→ **GPS 拒止模式下完全沒有速度量測**,速度狀態完全由位置量測驅動,
   沒有東西能快速拉回幽靈速度。
5. 諷刺的一點:`EK3_GLITCH_RAD=50` 讓**中等大小的跳動變成最糟情況**——
   - 超過 50 m 的跳動 → EKF 做位置 **reset**,而 ArduPilot 控制器會刻意
     平移目標吸收 reset,相對溫和;
   - **50 m 以內**的跳動 → 被融合,控制器全力對抗。把此參數從 25 調到 50,
     等於加寬了暴衝的觸發窗口。

## 5. 整合 OpenVINS(VIO)能不能解決?

**不能單獨解決。** 跳動來自「絕對定位修正層」(AnyLoc accept / RE-ACQUIRE)
與「修正量的餵入方式」,不是來自航位推算層(VIO 要取代的是後者)。

| 面向 | 評估 |
|---|---|
| 跳動問題 | 不解決——OpenVINS 會漂移、沒有地圖,仍需 AnyLoc 做絕對修正,修正依然是跳躍,問題只是搬到別行程式碼 |
| 錨點間漂移 | 大幅改善 → jump gate 的先驗更準、誤判更少、RE-ACQUIRE 更少發生 |
| 幽靈速度 | **真正有幫助的地方**——OpenVINS 提供真實速度量測(VISION_SPEED_ESTIMATE),可固定 EKF 速度狀態,位置階躍無法再偽裝成移動 |
| 整合成本 | 高:需 200 Hz IMU + 相機時間同步(IMU 在飛控內、經 MAVLink、無硬體同步)、IMX219 rolling shutter、Kalibr 內外參標定、與 YOLO/DINOv2 搶 Jetson 算力;實際上是數週工程 |

結論:先做低成本修正(第 6 節),之後若 replay 顯示瓶頸是 VO 漂移
(而非 AnyLoc 誤差)再考慮 VIO。survey13 replay(plan-B 平均 13–15 m)
顯示目前瓶頸不在 VO。

## 6. 修正方案(依投資報酬率排序)

### 6-1. ★ 在 commander 加 VPE 斜率限制(slew limiter)——首要修正

**做什麼:** commander 內部維護一個「已發佈位置」狀態;每個 5 Hz 週期,
把已發佈位置朝定位器最新估計**以限速移動**(例如上限 2.5 m/s →
每 tick 最多 0.5 m),而不是直接複製。

**效果:**
- 20 m 的跳動變成約 8 秒的平滑滑移。
- EKF 看到的是緩慢漂移 → 沒有位置階躍、沒有幽靈速度尖峰,
  兩個暴衝來源同時消失。
- RE-ACQUIRE 的無上限跳躍也自動變安全。
- 不損失資訊——最終收斂位置相同,只是把修正攤在時間上。
- 錯誤的 accept 只會造成有限傷害(還沒滑到就被下一個好錨點拉回)。

**代價:** 大幅修正的生效延遲(20 m ÷ 2.5 m/s = 8 秒)。可接受——
AnyLoc 輸出本來就是約 50 m 網格量化,飛機真實漂移速率也遠低於此。

### 6-2. 降低 `WPNAV_SPEED`(1200 → 400–500)

只管「追航線」階段的上限,**管不住擾動修正暴衝**(見第 4 節),
但能降低最壞情況並減少動態模糊,對 VO/AnyLoc 品質有直接幫助。

### 6-3. Covariance 改成與「修正量大小」成正比(而非 err_m²)

大幅修正 → 大 covariance → EKF 自己慢慢融入。若已做 6-1 則優先度低
(slew limiter 行為確定性更高、更好推理)。

### 6-4. RE-ACQUIRE 視為「事件」而非「資料點」

重新定位期間 commander 先暫停任務(Loiter/Brake),等斜率限制滑移完成再續飛,
避免在參考座標系變動中追航點。

### 6-5. 備援保險:降低 `ANGLE_MAX`

物理上限制任何修正的猛烈程度,代價是抗風能力。當保險用,不是主要修正。

## 7. 驗證方式

1. **離線 replay:** `anyloc/test_vo_fusion_compare.py` 重播 survey13 影片,
   比較加 slew limiter 前後的每步位移(之前 jump gate 驗證:最大步 19.3 → 7.6 m)。
2. **板凳測試:** 對 commander 餵人工階躍估計,確認發佈出去的 VPE 是限速滑移。
3. **實飛驗證:** 觀察 Mission Planner 中位置修正是否變為緩慢平移、
   飛機不再暴衝。

## 8. 實作與驗證結果(2026-07-18)

### 實作(最終版 2026-07-18,經兩次閉環失敗迭代)

- **`control/vpe_slew.py`** — `VpeSlewLimiter`(純 Python,無 ROS 相依,可離線測試):
  發佈點以「**目標自身移動速度**(對估計值更新率取 robust 中位數,
  跳躍級離群值排除)+ `corr_rate` 2.5 m/s」的配額朝定計值滑移。
  巡航運動 tick-for-tick 跟上;只有超出真實運動的修正被限制在 2.5 m/s。
  另有 dt 夾限(執行緒卡頓時單 tick 最多滑 1.25 m)。
- **`control/ardupilot_commander.py`** — VPE 執行緒接線:
  Phase 2 AnyLoc 目標經 limiter(**不需要也不可使用 EKF 速度**,見下);
  Phase 1 每 tick `reset()`,交接時從最後 Phase-1 位置平滑滑入;
  修正量 >10 m 時印出 `VPE correction xx m — gliding in, not stepping`。

**⚠ 兩個被 SITL 閉環測試否決的設計(勿再引入):**
1. 以 EKF **速度向量**對發佈點做航位推算 → 正回饋發散
   (發佈跟隨 EKF 速度、EKF 位置又跟隨發佈,速度誤差自我證實;
   SITL 中無界暴走)。
2. 以 EKF **地速純量**當配額 → 滯後死鎖
   (無速度源時 EKF 速度來自發佈的 VPE:落後→低速→配額更小→更落後;
   SITL 中穩定落後 ~60 m)。
   教訓:**唯一不在此 limiter 下游的速度參考,是估計值自身的移動**
   (VO 由相機錨定,與 EKF 無關)。

### 離線驗證(`python3 control/test_vpe_slew.py`,survey13 資料)

單元測試全過(懸停 30 m 階躍 12.0 s 滑入、巡航零落後、
巡航中修正率 ≤ 2.5 m/s、卡頓夾限)。survey13 重播(20 Hz commander 節奏,
EKF 速度以 GPS 真值速度模擬):

| 資料 | 軌跡 | 模式 | 最大單tick步 | 最大修正速度 | 誤差 mean/max |
|---|---|---|---|---|---|
| replay(gate 0.32) | plan-B | 原始 | 7.49 m | 30.7 m/s | 15.3 / 35.6 m |
| replay(gate 0.32) | plan-B | **slew** | **0.33 m** | **2.6 m/s** | 10.7 / 22.8 m |
| stress(gate 0.25,含 teleport) | plan-B | 原始 | 67.7 m | 85.5 m/s | 20.0 / 102.9 m |
| stress(gate 0.25,含 teleport) | plan-B | **slew** | **0.35 m** | **2.6 m/s** | 14.7 / 31.9 m |

- 發佈階躍 67.7 → 0.71 m;修正速度(1 s 窗)85.5 → ~16 m/s(暫態幾何最壞值);
  誤差統計不變。暴衝的兩個來源(位置階躍、幽靈速度尖峰)都消失。
- **實飛驗證仍未做。**

## 9. SITL 閉環驗證(2026-07-18,`control/test_vpe_slew_sitl.py`)

方法:ArduPilot 4.7 SITL(內建四軸物理、無 GPS、ExternalNav、
real_hw.parm 等效 EK3/PSC 參數)起飛後沿 survey13 GPS 真值航線飛行;
20 Hz VPE = SITL 真實位置 + survey13 重播的定位器誤差序列
(plan-B VO+AnyLoc 軌跡 − GPS 真值),raw 直發 vs slew 經 limiter。
四趟飛行紀錄真實航跡/EKF/VPE → `anyloc/logs/survey13_sitl_*.json`,
航跡圖 `anyloc/logs/survey13_sitl_slew.png`。

結果與解讀:

| 情境 | raw(現行) | slew |
|---|---|---|
| 航跡形狀 | 跟著航線但有急轉/急折(<50 m 被融合的跳點 = 實機的暴衝) | 完全平滑,無急轉 |
| 橫向偏差 mean/max | stress 14.8/39.7 m;normal 7.9/26.8 m | stress 42.5/109.8 m;normal 21.1/77.7 m |
| 穩定性 | — | 閉環穩定(前兩版設計在此發散/死鎖,被 SITL 抓出) |

**解讀(重要):**
- slew 的橫向偏差較大是「忠實飛出定位器誤差滑移」的代價:錯誤的
  re-acquire(~103 m)被以巡航速度平滑飛出去再飛回來(110 m 綠色迴圈)。
  raw 在 SITL 裡看起來偏差小,是因為兩個「SITL 才有的優勢」:
  (a) 注入誤差是錄好的,不會因快速機動而惡化——**實機上 raw 的暴衝會
  造成動態模糊 → VO/AnyLoc 惡化 → 發散**(實際觀察到的失控機制),
  SITL 無法重現這個回饋;(b) >50 m 的 teleport 觸發 EKF reset,
  ArduPilot 控制器會平移目標吸收 reset,等於「免費」擋掉大跳。
- 因此 SITL 證明的是:**穩定性 + 飛行平滑性(暴衝機制消除)**;
  真正的效益(保住視覺輸入品質)只能實飛驗證。

**後續建議(新增):**
1. slew 部署後,考慮把 `EK3_GLITCH_RAD` 從 50 調回 25:>25 m 的殘餘
   跳點讓 EKF reset 吸收(SITL 顯示這條路徑很溫和),<25 m 的由 slew
   平滑化——兩個機制互補。
2. `corr_rate` 可調低(1.0–1.5)換取更小的錯誤修正飛行距離
   (暫態誤差在被飛完之前就被 AnyLoc 修回),代價是真修正較慢。
3. 第 6-4 節的 RE-ACQUIRE 暫停任務建議仍然值得做——
   green 110 m 迴圈就是它要防的事。

## 10. OpenVINS 等級 VIO 誤差模型的 SITL 對照(2026-07-18)

**真實 OpenVINS 無法直接跑這個實驗:** SITL 內建物理沒有相機影像;
survey13 錄影有影片但**完全沒有高頻 IMU**(telemetry.csv 只有 5 Hz
lat/lon/alt/heading/RC),而 OpenVINS 需要 ~200 Hz IMU + 相機時間同步。
因此改為「**誤差模型代換**」:`control/gen_vio_track.py` 用
OpenVINS 等級的里程計誤差(漂移 = 行進距離 1% + 0.3 m 抖動、無跳躍)
搭配 **survey13 錄到的真實 AnyLoc 候選流**(score+誤差,來自
survey13_vo_fusion.json),以 plan-B 邏輯融合——但 jump gate 因為
VIO 漂移率小,可收緊到 10 m + 0.2 m/s。輸出
`anyloc/logs/survey13_vio_synth.json`(情境 "vio"),跑同樣的 SITL 閉環。

結果(對照第 9 節表格):

| 情境 | xtrack mean / max |
|---|---|
| LK-VO+AnyLoc stress raw / slew | 14.8/39.7 m / 42.5/109.8 m |
| LK-VO+AnyLoc normal raw / slew | 7.9/26.8 m / 21.1/77.7 m |
| **VIO 模型 raw / slew** | **1.2/3.8 m / 1.2/3.7 m** |

- 產生誤差軌時就已看出關鍵機制:**里程計品質改變了 AnyLoc gate 能收多緊**。
  10 m 緊 gate 把 3 個過 score-gate 的候選(誤差都 >10 m)全數擋掉,
  估計全程靠 dead-reckoning 維持 ~1 m 誤差。
- 航跡幾乎完美貼合航線,raw 與 slew 無可分辨——**跳點問題在源頭消失,
  slew 變成便宜的保險**。
- 誠實警語:此為誤差「模型」。實機 OpenVINS 要達到 1% 漂移需跨過:
  200 Hz IMU 串流(IMU 在 FC 內、走 MAVLink、無硬體同步)、IMX219
  rolling shutter、相機-IMU 標定(Kalibr)、震動、與 YOLO/DINOv2 搶算力
  ——數週級整合工程。長航程(>幾分鐘)漂移累積後仍需 AnyLoc 修正,
  gate 會隨時間放寬,屆時 slew 的保護重新變得重要。
