# VPE 位置跳動導致飛機暴衝 — 問題診斷報告

> 日期:2026-07-17(診斷)/ 2026-07-18(6-1 已實作並離線驗證,實飛驗證待做)/ 2026-07-22(第 11 節:真實 OpenVINS 首次離線結果;第 12 節:路線建議、氣壓計尺度修正、標定 FAQ;第 13 節:Kalibr 逐步操作)
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

## 11. 真實 OpenVINS 首次離線結果 — 對第 10 節模型的現實檢驗(2026-07-22)

第 10 節的「1% 漂移、無跳躍」是**誤差模型**;survey17(2026-07-21 傍晚,
真實 17.5 分鐘飛行、3.8 km、200 Hz IMU 全程)是第一份能跑真 OpenVINS 的
資料集。結果(**未標定**:內參由 FOV 推算、零畸變、外參手推 nadir 猜測;
完整細節 `field_data/survey17/vio_eval/README.md`):

| 段落 | 結果 |
|---|---|
| 前段 225–400 s(744 m、20 m AGL) | 漂移 <1%(3–6 m);高度誤差 rmse **1.8 m** / max 5.1 m — **達到第 10 節模型等級** |
| 垂直爬升 400–440 s(20→104 m 全油門) | **濾波器崩潰**:垂直 1.45x 高估、加速度計 bias 暴走、隨後全面發散 |
| 巡航 448–775 s(空中動態重新初始化,2.7 km) | 形狀好,但 XY **尺度塌縮到 0.43x**(XY rmse ~109 m);高度 rmse 6.3 m,但緩慢下漂 −13 m,**777 s 即發散**(比 850 s 開始下降還早) |
| 下降 850 s+ | 再次崩潰 |

**對本報告論點的影響:**

1. **第 5 節「VIO 不能單獨解決跳動」維持成立,且多了一個新理由:**
   未標定的真實 OpenVINS 自己就會產生尺度塌縮與發散——比 AnyLoc 跳點
   更糟的輸入。VIO 取代 VO 之前,slew limiter 與 jump gate 一個都不能拆。
2. **第 10 節的 1% 模型「前段」已被真實資料證實**,但只在低空穩定段;
   「收緊 gate 到 10 m」的前提(全程 1% 漂移)目前只有部分段落成立。
   **Kalibr 標定是驗收門檻**——尺度與爬升發散的判決要等標定後重跑。
3. **高度通道是 VIO 表現最好的軸**(1.8–6.3 m rmse vs 巡航 XY ~109 m),
   但仍會漂(巡航 −13 m);氣壓計 AGL 是天然的外部約束。
4. **氣壓 vs GPS 高度基準在 survey17 上不可分辨**:telemetry 的
   `alt_amsl` 與 `alt_agl` 是同一個 EKF 垂直狀態(差固定 home 高 ~91 m,
   全程相差 ≤0.8 m);FC 出廠預設 `EK3_SRC1_POSZ=1` 代表這個高度
   **本來就是氣壓計主導**。要做真正的氣壓/GPS 對照,錄製端需加錄
   `/mavros/global_position/raw/fix`(原始 GPS 高度)。
5. 餵資料必須從乾淨的靜止窗開始(手提移動中初始化立刻毀掉);
   若標定後爬升段仍發散,首要嫌疑是 200 Hz RAW_IMU **未濾波震動混疊**
   (FC EKF 用的是濾波後 delta-velocity,我們拿到的是原始樣本)。

分析工具:`~/openvins_ws/`(ROS-free OpenVINS v2.6.3 + `run_video_msckf`
餵料器 + `compare_vio_gps.py`);高度對照
`field_data/survey17/vio_eval/vio_alt_baro_compare.py` + 同名 .png。

## 12. 路線建議與常見問題(2026-07-22 討論整理)

### 12-1. 目前的最佳解:分層架構,不是單一技術

| 時程 | 內容 |
|---|---|
| **下次飛行(現在就能飛)** | plan-B + slew limiter(唯一同時消除位置階躍與幽靈速度、且已離線+SITL 驗證的組合)。搭配:`EK3_GLITCH_RAD` 50→25(>25 m 走 EKF reset 溫和路徑、<25 m 由 slew 平滑,互補)、調低 `WPNAV_SPEED`、確認跑的是 `full_run.sh`(plan-B)而非 plan-A 鏈 |
| **下個工作天(最高價值行動)** | **Kalibr 標定**——尺度塌縮、爬升發散判決、gate 能否收緊,全部卡在它;一張 AprilGrid + 一次桌上錄製而已(見 12-3) |
| **目標架構** | OpenVINS 做里程計層 + 氣壓計 AGL 綁高度/尺度 + AnyLoc 走收緊的 jump gate 做絕對修正 + slew limiter 留守餵入端 |
| **明確不做** | 現在就讓 OpenVINS 進迴路、或設計 VIO-only 導航(survey17 證明未標定 VIO 每個大油門垂直段都發散) |

各層互補的理由:OpenVINS 補 LK-VO 做不到的(真實速度輸出 → 從源頭消除幽靈速度;~1% 漂移 → gate 可收到 ~10 m);氣壓計補單目 VIO 做不到的(尺度與高度漂移,成本為零);AnyLoc 補兩者都做不到的(絕對位置);slew limiter 留著,因為 survey17 證明壞掉的 VIO 產生比 AnyLoc 跳點更糟的輸入——護欄永遠不拆。

### 12-2. 氣壓計能修正 VIO 尺度嗎?——分軸、分飛行階段回答

survey17 的巡航段尺度修正(第 11 節的 0.43x)是**用 GPS 真值離線擬合**出來的
單一常數(similarity 對齊解一個 k,見 `vio_eval/vio_path_compare.py`)——
診斷用,實飛時沒有 GPS 可擬合。實飛的尺度約束來源:

| 情境 | 氣壓計的作用 |
|---|---|
| **高度通道(任何時候)** | 直接取代——機上本來就是氣壓主導(`EK3_SRC1_POSZ=1`),VIO 只餵 XY;VIO 的 −13 m 下漂與 1.45x 爬升高估根本到不了飛機 |
| **爬升/下降段** | **真正的尺度修正器**——VIO 說爬了 120 m、氣壓說 84 m → 1.4x 尺度誤差直接可觀測;單目尺度是全軸共用一個因子,修正會連 XY 一起拉回 |
| **等高巡航段** | **完全沒用**——垂直零運動 = 高度感測器拿不到任何尺度資訊;survey17 的 0.43x 塌縮正是發生在 2.7 km 等高巡航,氣壓計看不見它 |

結論:任務剖面是長距離等高航線 → 巡航中唯一能綁 XY 尺度的是 **AnyLoc 絕對修正**
(每個被接受的錨點都隱含量測了「距上個錨點以來的累積尺度誤差」)。
另一個前提:爬升段目前會直接毀掉濾波器——「氣壓當尺度修正器」要等
Kalibr(或 IMU 濾波)先把爬升發散治好才用得上,又一個標定是門檻的理由。

### 12-3. Kalibr 標定 FAQ

**Q:每次飛行都要標定嗎?** 不用,**一次/每次機構變動**:
- 內參(焦距/畸變)= 鏡頭屬性,鏡頭不動就永久有效。
- 相機-IMU 外參 = 安裝屬性,**重新拆裝相機、動 FC、硬著陸**才需重做
  (memory 既有原則:one calibration session per camera remount)。
- 時間偏移 OpenVINS 線上估(輸出的 `cam_dt`),IMU 噪聲參數靜置錄一次即可。
- 觸發事件之外,若 VIO 品質無故明顯變差,做一次 sanity 重標。

**Q:survey17 的資料夠不夠拿來標定?** 不夠——不是量的問題,是**種類**錯了:
1. 標定需要**已知幾何的標靶**(AprilGrid 角點 = 對已知真值的量測);
   survey17 拍的是 3D 位置未知的田野道路,無公制參考。
2. 需要的運動是**近距離、三軸激烈旋轉**(手持畫 8 字);survey 航線刻意
   平滑等速——跟尺度塌縮同一個物理:無激勵就無可觀測性。
3. 20–104 m AGL 視差相對景深太小,焦距/平移外參幾乎無約束
   (標定要 0.5–2 m 近距離,讓參數誤差放大成看得見的像素誤差)。

**實際成本**:印一張 AprilGrid(A3+ 平板),**飛行構型**的機體
(相機裝好、`frame_rotation_deg=180` 錄製方向、IMU 率跑著)在標靶前
揮舞 ~5 分鐘,`record_field.py --calib` 錄下——桌上作業,不用飛、
不用出門;Kalibr 在 PC 跑(Jetson 磁碟 ~95% 滿)。

**低期望值的順手實驗**:OpenVINS 有線上標定 flag(濾波內精修內外參),
可在 survey17 前段試開——但從 FOV 猜測起點 + 弱激勵通常只收斂一部分,
是迴路內精修、不是門檻級標定的替代品。

### 12-4. 軌跡對照圖

`field_data/survey17/vio_eval/vio_path_compare.png`(產生腳本同目錄
`vio_path_compare.py`):左 = XY 航跡(GPS 真值 vs 對齊後的 VIO 前段/巡航段,
含尺度修正後的虛線版——survey 航線幾乎疊回真值,證明是尺度問題不是形狀問題);
右 = 高度(氣壓 AGL vs VIO,爬升段紅色陰影,777 s 發散尾巴可見)。

## 13. Kalibr 標定 — 逐步操作程序(2026-07-22)

精簡版在 `instructions/vio_data_collection.md` §4;本節為完整可執行版。
原則回顧(12-3):一次/每次機構變動,不是每次飛行。

### 步驟 A. 準備(一次性)

1. **印 AprilGrid**:Kalibr 官方 `aprilgrid.pdf`(6×6),**A1 以上**,
   貼在平整硬板上(翹曲會直接變成內參誤差)。
   檔案已存 repo:`instructions/april_6x6_80x80cm_A0.pdf`
   (wiki 的 Google Drive 連結已死;此檔取自
   https://github.com/ethz-asl/kalibr/files/8514447/april_6x6_80x80cm_A0.pdf,
   kalibr issue #514)。A0 100% 列印時 tagSize=0.088、tagSpacing=0.3
   (=下方 target.yaml 範例值);或建好 Kalibr Docker 後用
   `kalibr_create_target_pdf --type apriltag --nx 6 --ny 6 --tsize 0.08
   --tspace 0.3` 產生合乎自家印表機紙張的版本。**務必 100% 原尺寸列印
   (勿「縮放至頁面」),印完用尺量實際尺寸**填 target.yaml。
2. **量實際尺寸**填 `target.yaml`(印表機會縮放,務必用尺量):
   ```yaml
   target_type: 'aprilgrid'
   tagCols: 6
   tagRows: 6
   tagSize: 0.088        # 單一 tag 邊長(公尺),量實物!
   tagSpacing: 0.3       # 間距/邊長比,量實物!
   ```
3. **PC 裝 Kalibr**(不在 Jetson 跑):官方 repo(ethz-asl/kalibr)的
   Docker 路線最省事,`Dockerfile_ros1_20_04` build 一次。

### 步驟 B. 錄製標定段(Jetson,整機飛行構型)

1. **FC 上電**(IMU 來自 FC——手持時整機一起動)、相機裝在飛行位置。
   若相機開不起來:`sudo systemctl restart nvargus-daemon`(殘留
   CaptureSession 地雷)。
2. 啟動錄製,確認顯示 **imu=200Hz** 再開始動作:
   ```bash
   source control/ros2_env.sh
   python3 tools/record_field.py --calib --duration 120
   ```
3. **激發動作**(全程標靶保持在畫面內、動作平滑——rolling shutter 怕急動):
   - 靜置 5–10 s(bias 初始化);
   - 距標靶 **0.5–1.5 m**(標靶接近滿框);
   - 每軸旋轉 ±30–40°:pitch 上下點頭、yaw 左右搖、roll 側傾,各 2–3 回;
   - 三軸平移:前後(對焦深)、左右、上下,各 2–3 回;
   - 綜合 8 字揮舞 20–30 s;
   - 結尾靜置 5 s。
4. **當場驗收**(不合格重錄,成本只有兩分鐘):
   - `meta.json` 的 `imu_achieved_hz_at_stop` = 200.0;
   - 抽看影片:影格清晰無拖影、角點銳利、標靶極少出框。

### 步驟 C. 轉 Kalibr 輸入格式(PC)

1. 把 `field_data/calib_<時間>/` 整包拷到 PC。
2. 抽影格(**用錄好的方向,絕不要自己再轉 180°**——錄影存檔前已轉,
   `meta.json frame_rotation_deg: 180`)。
   ⚠ `tools/extract_frames.py` **不能用**——它是 AnyLoc 資料庫建置器,
   會按 GPS 距離(30 m)+ AGL≥50 m 過濾,手持標定錄影會被濾成 0 張
   (vio_data_collection.md §4 寫它可用是錯的,已知勘誤)。
   實際做法(kalibr_bagcreater 要求 `cam0/<奈秒時戳>.png`):
   ```bash
   mkdir cam0 && ffmpeg -i calib_<時間>/video.mkv -vsync 0 tmp_%06d.png
   # 再用 frame_times.csv 逐張改名成奈秒時戳(第 n 張 ↔ 第 n 列 unix_time×1e9)
   # 順手驗證:張數 == frame_times.csv 列數,不等表示掉幀,按缺口切段
   ```
3. `imu.csv` → Kalibr 的 `imu0.csv`(用 `stamp_ros` 欄,轉奈秒:
   `timestamp, omega_x, omega_y, omega_z, alpha_x, alpha_y, alpha_z`)。
4. 打包:`kalibr_bagcreater --folder . --output-bag calib.bag`

### 步驟 D. 相機內參

```bash
kalibr_calibrate_cameras --bag calib.bag --topics /cam0/image_raw \
    --models pinhole-radtan --target target.yaml
```
驗收:重投影誤差 **< 0.5 px**(報表 PDF 會給);畸變係數非零但不誇張。
產出 `camchain.yaml`。

### 步驟 E. IMU 噪聲檔 `imu.yaml`

```yaml
accelerometer_noise_density: 2.0e-3   # Pixhawk 級通用值起步
accelerometer_random_walk:   3.0e-3
gyroscope_noise_density:     1.7e-4
gyroscope_random_walk:       2.0e-5
update_rate: 200
rostopic: /imu0
```
註:MAVLink RAW_IMU 是未濾波原始樣本,實務上把 noise density **放大 5–10x**
餵給 OpenVINS 常更穩;Kalibr 這關先用上表即可。

### 步驟 F. 相機-IMU 外參 + 時間偏移

```bash
kalibr_calibrate_imu_camera --bag calib.bag \
    --cam camchain.yaml --imu imu.yaml --target target.yaml
```
驗收:
- `T_cam_imu` 旋轉部份接近手推的 nadir 構型(vio_eval/config_used 的猜測值
  可當 sanity 對照,差太遠=座標系搞反);
- 時間偏移量級毫秒、且報表中穩定(不漂);
- 重投影誤差 < 1 px,加速度/角速度殘差圖是白噪聲(有結構=激發不足,重錄 B)。

### 步驟 G. 套用 + 驗收(回 Jetson,判決時刻)

1. 把 Kalibr 產出寫進 OpenVINS 設定(取代 FOV 猜測版):
   `kalibr_imucam_chain.yaml` + `kalibr_imu_chain.yaml`
   (格式同 `field_data/survey17/vio_eval/config_used/`)。
2. **重跑 survey17**:`~/openvins_ws` 的 `run_video_msckf` 三段
   (前段 210 s 起、全程、巡航 448 s 起)+ `compare_vio_gps.py`。
3. 判準(對照第 11 節未標定基線):
   | 指標 | 未標定 | 標定後期望 |
   |---|---|---|
   | 前段尺度 | 0.79x | → ~1.0 |
   | 巡航尺度 | 0.43x | 大幅改善(殘餘漂移仍在,由 AnyLoc/氣壓綁) |
   | 爬升段 | 全面發散 | **若仍發散 → 病因是 IMU 震動混疊,轉向 INS_ notch/濾波 或獨立 IMU,標定已排除** |
4. 達標 → 依 vio_data_collection.md §5.4 評估即時整合
   (plan-B + slew 不動,gate 收緊到 ~10 m + 0.2 m/s)。

### 常見地雷(本專案已知)

- 影像方向:標定與飛行推理必須同方向(錄檔已 180°,別重複轉)。
- 手持錄製 FC 沒上電 → 沒有 IMU,白錄。
- 新開終端跑任何節點前 `source control/ros2_env.sh`(64MB SHM,否則掉幀)。
- 標靶出框太久、動作太猛(拖影)、距離太遠(>2 m)= 激發不足,
  Kalibr 會收斂但共變異數大——報表的參數不確定度要看。
- Jetson 磁碟已清到 77%(2026-07-22),但 Kalibr 本體仍建議在 PC 跑
  (ROS1 相依 + 記憶體)。
