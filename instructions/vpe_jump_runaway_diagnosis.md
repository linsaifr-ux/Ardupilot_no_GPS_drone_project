# VPE 位置跳動導致飛機暴衝 — 問題診斷報告

> 日期:2026-07-17(診斷)/ 2026-07-18(6-1 已實作並離線驗證,實飛驗證待做)/ 2026-07-22(第 11 節:真實 OpenVINS 首次離線結果;第 12 節:路線建議、氣壓計尺度修正、標定 FAQ;第 13 節:Kalibr 逐步操作)/ 2026-07-23(第 14 節:Kalibr 結果 + 標定後重跑判決——標定不是瓶頸,IMU 震動混疊已實證,解法排序;14-6 實機量測:333 Hz 串流驗證、batch logging 已上機;14-7 外場飛行計畫;14-8 外場飛行完成 + FFT 找到 134.3 Hz 震動線 + Stage 2 notch 已填;14-9 notch 已上機驗證 + survey25 OpenVINS 評估——爬升不再發散,巡航尺度仍塌縮;14-10 問答:巡航尺度解法不是 notch,是 AGL 景深先驗/物理減震)/ 2026-07-24(14-11 巡航尺度診斷:病灶在轉彎時的視覺端,非泛用 IMU 故障,accel bias 凍結+航向殘差實測佐證;14-12 陀螺閘門實作——硬閘門大勝、軟閘門修 cache 地雷後仍不如硬閘門,rolling shutter 判斷再獲強化,兩者最終仍頂不住多轉彎巡航;14-13 AGL 景深先驗修好了(4 次失敗是建置系統沒真的重編,不是幾何算錯)+ 與陀螺閘門合併評估——合併不是贏家,AGL 先驗單獨最強但仍在 ~470s 前發散;14-14 換路線:輸出端因果尺度修正(baro+AnyLoc-proxy 弧長)+ 錨點拉回——全程 170-472s rmse 76m/max 130m,第一個全程有界的結果,含誠實 caveat(理想錨點單獨 14m、跨轉彎斷訊時橋接失效);14-19 真的接上 AnyLoc(非模擬)——DB 重建 zoom20 是真上限(620→286m mean 大幅進步)、value facet 修正對此場地不成立(mini-ablation 輸給預設特徵)、端到端 231m 反而比模擬錨點差,域差距在確認過的解析度上限仍是主導瓶頸,離 FoundLoc 16.4m 還很遠;14-20 對照實驗實證域差距是主因——同域(drone vs drone)資料庫誤差只有 8.6m,比 zoom-20 衛星結果低 33 倍,AnyLoc/VLAD pipeline 本身沒問題,瓶頸 100% 在衛星影像來源;14-15 FoundLoc(CMU AirLab,同 AnyLoc-DINO VPR)兩機制移植——DBSCAN 假陽性過濾+陀螺輔助 degeneracy-aware 鎖定,用真實雜訊等級錨點誠實測試 rmse 137m;追問「為何不是 FoundLoc 的 20m」查出 pull 增益太弱太慢(掃描後 0.60 取代 0.30),乾淨錨點 76→35.3m 逼近 FoundLoc、雜訊錨點 137→93.8m;14-16 追問「真的是 rolling shutter 嗎」——直接測試結論不成立,損傷跟累積轉彎時間/角度(r=0.82)而非峰值角速度(r=0.35)相關,病灶更像追蹤累積失能;14-17 對症下藥:陀螺輔助 KLT 追蹤(15px 窗口零運動預測是真凶)——原始 VIO rmse 7703→2722m,峰值角速度相關性歸零,但下游融合層打平;14-18 重調融合層(eps 300→200)+ 上線前驗證(回歸測試逐位元組通過、機制結論獨立複現、12-seed 穩健性)——最終 rmse 77.9m/max 164m,今晚最佳且唯一經完整驗證的版本,下降段發散+殘餘 duration 相關性仍是已知未解限制;14-21 100m AGL 限定重測是乾淨的 null result,互動圖尾端漂移查出是懸停期間 frame extractor 沒取樣造成,加 `--max-time-gap` 修好)/ 2026-07-25(14-22 SITL AUTO 模式接上完整 pipeline 飛 survey25 航線——raw VPE 真實重現 EKF 暴衝(單 tick 跳 341.8m,觸發 GLITCH_RAD),slew limiter 消除跳動但飛出 11.5km 繞路、任務逾時未完成,證明 slew 只解決急性暴衝不解決精度問題;14-23 覆蓋範圍假說架構上測不出因果(先列為待辦)、照抄 AnyLoc 論文 Nardo-Air-R 旋轉對齊手法(意外挖出並修正一個方向 sign bug,286.5→228.6m mean)、系統化驗證「衛星圖不夠清楚」假說不成立(清晰度 vs 誤差相關性弱且非單調,r=-0.14~-0.23);14-24 2026-07-26 真正即時依位置查詢的 SITL 閉環+postview——找到並修好方向 bootstrap bug 與 SPEEDUP 配速 bug 後,最終結論:一旦偏離錄製航線,真實 pipeline 閉環不會自我修正(兩輪獨立測試方向一致,一個乾淨暴衝到 7000m+,一個 noisy 有界螺旋穩定在 2500-4250m);14-25 總結:瓶頸是資料庫 domain gap 不是 VIO(33倍對照實驗證據),下一步建議實機建圖飛行,附三個不能跳過的落差(跨飛行未測、需涵蓋整個任務區非單一航線、需獨立驗證飛行))/ 2026-07-27(14-26 建圖+驗證飛行真的飛了(survey33/survey32),跨飛行同域誤差 24.1m,符合預測;14-27 OpenVINS 執行檔過期地雷發現+修復(執行檔與 library 沒同步重連結,heap corruption);14-28 survey32 真實 AUTO 模式 SITL 閉環測試——第一個零暴衝結果,因航線全程在資料庫涵蓋範圍內;14-29 確認本專案人工起降、pipeline 只管巡航,任務改用 LOITER 取代 LAND;14-30 `--stride 1` 全影格率實測反而讓 raw VIO 更差,維持 stride=2 預設)/ 2026-08-11(14-45 相機改回 AP-IMX900 global shutter 後實測 survey38/41/42——三趟全部災難性發散,診斷為這個機身+新鏡頭從未做過 Kalibr 標定,不是 shutter 類型問題,結論目前答不了;14-46 後續追問排除旋轉觸發、排除掉幀/資料損毀,把嫌疑縮小到外參平移(lever arm)或內參,標定分階段計畫已寫好、Phase 2 工具已備妥待 Frank 錄影;14-47 Phase 1 錄影完成(calib_20260811_232301 為主要 session)+ 發現 `--trim-head` 預設值 12.5s 過期地雷(這兩趟實際約 23-26.5s),session 專屬 KALIBR_PC_README.md 已備妥,下一步待 Frank 在 PC 上跑 Kalibr Docker)/ 2026-08-12(14-48 Kalibr Phase 4-6 完成:PC 端標定結果 reprojection error 略高於驗收線,但外參平移 |t|=0.259m 經 Frank 現場尺量確認完全吻合,接回 OpenVINS 新 config(ap_imx900_kalibr)重跑 survey38/41/42——過程中另外發現一個評測方法地雷(把 `run_video_msckf` 的擷取範圍直接設成 RMSE 評測窗會跳過起飛前靜置段,靜態初始化永遠觸發不了)並修正;結果:10m/65m 仍然災難性發散(rmse 26320m/9445m,速度暴衝模式與近似標定時幾乎一樣只是稍晚出現),100m 大幅改善(1753m→301.9m,5.8 倍進步,速度不再暴衝),但仍比舊 IMX219 相機同高度基準(75.8m)差約 4 倍——真標定是必要但不充分,結論收斂到「AGL 才是主因」呼應 §14-35/14-41,shutter 類型這個原始問題目前仍無法乾淨隔離驗證)/ 2026-08-12 稍晚(14-49 用新相機建同域 AnyLoc 資料庫(survey40→survey42,跨飛行 ~49 分鐘)實測完整 VPE 融合管線——同域檢索本身只有 ~159m 平均誤差、信心分數不分好壞,比 2026-07-27 舊相機的 24-25m 差很多;production anchor-chain(161.6m)、FoundLoc 風格 DBSCAN+陀螺鎖定 corrector(123.8-130.1m)都測了,結論意外:任何一種融合都打不贏乾脆只用 VO(36.5m)——錨點太爛時,融合邏輯越聰明反而讓壞錨點滲入越深;根因未定案,49 分鐘的光影落差是頭號嫌疑)
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
   python3 tools/record_field.py --calib --duration 120 \
       --stream-server 118.232.160.227
   ```
   `--calib` 不會自動開串流;`--stream-server` 才有即時預覽
   (瀏覽器 http://118.232.160.227:8889/drone),用來確認標靶滿框、
   不出界。串流是 4:3 中裁 16:9,比錄影檔窄——串流內可見即安全。
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

> ✅ 2026-07-23 已執行完畢——結果與判決見第 14 節(走到了「仍發散 →
> IMU 震動混疊」那條分支,且已用頻譜實證)。

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

## 14. Kalibr 結果與標定後重跑 — 判決:標定不是瓶頸,IMU 震動混疊是(2026-07-23)

第 13 節的程序已全部走完:錄製(第 5 次通過,`field_data/calib_20260723_001209`)
→ PC Kalibr → 結果拷回 `calib_20260723_001209/kalibr_output/` → 寫入 OpenVINS
→ survey17 重跑。本節記錄結果與新診斷。

### 14-1. Kalibr 標定結果(全部通過驗收)

| 項目 | 結果 | 驗收對照 |
|---|---|---|
| 內參 fx/fy | 1339.3 / 1335.9 | FOV 推算值 1359 的 1.5% 內 ✓ |
| 主點 cx/cy | 818.6 / 639.8 | 接近影像中心 ✓ |
| 畸變 radtan | k1 0.064、k2 −0.204、p1 0.0003、p2 −0.0022 | 非零且合理 ✓ |
| 相機內參重投影 | std ~0.40/0.49 px | < 0.5 px ✓ |
| cam-IMU 重投影 | mean 0.65 px | < 1 px ✓ |
| 外參旋轉 | 與手推 nadir 猜測差 ~1° | ✓(座標系沒搞反) |
| 外參平移 | **\|t\| = 0.358 m**(多在光軸方向) | 手冊寫的「~10 cm」是錯的——**實機量測確認相機離 FC IMU 就是 35.8 cm**(2026-07-23 Frank 確認) |
| 時間偏移 | **−0.056 s**(影像時戳比 IMU 晚 ~56 ms) | CSI/ISP 管線延遲量級合理;OpenVINS 直接吃 camchain 的 `timeshift_cam_imu` |

**交叉驗證**:OpenVINS 線上精修(calib flags 開著)自己收斂回 Kalibr 值——
時間偏移 −0.059 s、外參 z −0.363 m。兩套獨立方法互相印證,標定本身可信。

套用位置:`~/openvins_ws/config/survey17_kalibr`(靜態初始化)與
`survey17_kalibr_dyn`(動態初始化);`sigma_px` 1.5→1;IMU 噪聲沿用實測
MAVLink 路徑值(非 Kalibr 模板值)。

### 14-2. 標定後 survey17 重跑(方法與第 11 節完全相同)

| 段落 | 未標定 | 標定後 |
|---|---|---|
| 前段 225–430 s ATE2D | 16.7 m(尺度 0.80,修正後 4.7 m) | 18.3 m(尺度 0.78,修正後 5.9 m)— **不變** |
| 爬升 400–440 s | ~483 s 發散 | **仍在 ~473 s 發散** |
| 巡航 448–775 s ATE2D | 108.8 m(尺度 0.34x,修正後 35.3 m) | **85.9 m**(尺度 0.48x,修正後 27.5 m) |
| 巡航到下降 448–850 s | 348 m(777 s 中途發散) | **88.7 m(活到下降 ~815 s)** |
| 巡航高度 vs 氣壓 rmse | 6.2 m(−13 m 持續下漂) | 9.3 m(無下漂趨勢,810 s 附近短暫 −25 m) |

**判決(第 13-G 節預告的分支走到了「仍發散」那條):標定不是瓶頸。**
- 買到的:巡航強健性(不再 777 s 無故中途發散)+ 巡航 ATE 改善 ~20%。
- 沒買到的:前段精度不變、爬升照樣崩、巡航尺度仍塌一半(0.48x)。
- 依第 11 節第 5 點的預告,病因指向 **200 Hz RAW_IMU 震動混疊**(次要:
  rolling shutter)。標定這個變因已徹底排除。

工具:統計 `field_data/survey17/vio_eval/vio_kalibr_stats.py`、
圖 `vio_path_compare_kalibr.png`(+ 同名 .py)、
軌跡 `vio_full_kalibr.csv` / `vio_cruise_kalibr.csv`、log `run_*_kalibr.log`,
細節 `vio_eval/README.md`「Re-run with real Kalibr calibration」節。

### 14-3. 震動混疊:已用 survey17 資料實證(不再只是嫌疑)

`vio_eval/imu_vibe_spectrum.py`(圖 `imu_vibe_spectrum.png`)對四個飛行
階段做 Welch 頻譜:

| 窗 | 加速度計 std (m/s²) | 特徵 |
|---|---|---|
| 地面靜置(馬達關) | 0.004–0.007 | 乾淨——**感測器與 MAVLink 傳輸鏈本身無罪** |
| 低空航線 300–360 s | 0.38–0.87 | 頻譜地板抬升 ~60 dB,**平坦延伸到 100 Hz Nyquist** |
| 全油門爬升 400–440 s | 0.06–0.42 | 78–79 Hz 游走譜線(如真實 278 Hz 摺到 78 Hz) |
| 巡航 500–700 s | 0.19–0.59 | 同低空:平坦寬帶地板 |

判讀:真實機械頻譜應隨頻率衰減;「平坦到 Nyquist 的寬帶地板」是混疊的
簽名——槳/馬達諧波在 100 Hz 以上,被 200 Hz 取樣摺下來,又因轉速變動
抹成寬帶。飛行中加速度計 std 0.3–0.9 m/s² 是濾波器噪聲模型假設的 ~10 倍,
而且正好落在 VIO 積分求尺度的頻帶——**這就是尺度塌縮與爬升發散的機制**。

### 14-4. 解法(核心原則:混疊「取樣後不可逆」,只能在取樣前/取樣時消滅)

摺下來的能量與真實運動不可分辨,Jetson 端事後濾波**無效**。可行選項
(由便宜到貴):

1. **FC 濾波鏈(純參數)**:FC 目前是出廠預設(參數包 2026-07-21 已回退,
   harmonic notch 是關的)。開 `INS_HNTCH_ENABLE=1`(throttle 模式,或
   6C 的 ICM-42688 支援的 FFT 模式 4)、考慮 `INS_ACCEL_FILTER` 20→10 Hz。
   注意:notch 只作用在陀螺儀。
   ✅ 已從 ArduPilot 原始碼確認(2026-07-23,`GCS_Common.cpp
   send_raw_imu` → `ins.get_accel/get_gyro` = backend 濾波後的 loop-rate
   值):**RAW_IMU 串流的是「過完濾波鏈」的樣本**——所以開 notch 一定
   會反映到我們的串流;殘餘混疊來自 (a) 震動能量漏過 2 階 20 Hz LPF、
   (b) 400→200 Hz 串流減採樣無抗混疊(→ 選項 3 正中要害)。
   第 11/13 節寫的「未濾波原始樣本」是**錯的**,以本節為準。
2. **先看真頻譜:開 IMU batch logging**(`INS_LOG_BAT_MASK=1`)飛/槳測一次。
   FC 內部以 ~1 kHz 寫入 dataflash,直接看到 >100 Hz 的真實譜線位置
   ——notch 往哪擺不用猜。順便讀 VIBE 值。
3. **串流升到 400 Hz**:FC 內部 1 kHz→400 Hz(loop rate)那段有正確的
   抗混疊;沒有濾波的是 400→200 的串流減採樣。把 RAW_IMU 要到 400 Hz,
   Nyquist 變 200 Hz,OpenVINS 直接吃或在 Jetson 端做乾淨的軟體低通再減。
   頻寬:SERIAL1 本地線沒問題,LTE relay 本來就把 RAW_IMU 濾掉了。
4. **物理減震**:槳平衡、FC 軟墊(6C 無內建 IMU 隔震,6X 才有)。
   縮小要摺的能量本身,FC 自家 EKF 也受益。
5. **獨立 IMU(最後手段)**:Jetson 接 SPI/I2C kHz 級 IMU(晶片內建
   抗混疊濾波)。工程量最大;靜置資料證明現有鏈路乾淨,預期 1–3 就夠。

**建議順序**:桌上槳測(batch logging 開)→ 按真頻譜擺 notch →
串流改 400 Hz → 重飛一趟 survey → 重跑離線評估,看爬升發散與 0.48x
尺度是否移動。

### 14-6. 已備妥的工具(2026-07-23,等 FC 上電即可執行)

| 工具 | 用途 |
|---|---|
| `tools/imu_logger.py --imu-hz 333`(`record_field.py --imu-hz 333` 直通) | 選項 3:RAW_IMU 提速消除串流減採樣摺疊;OK 門檻自動按請求率的 40% 縮放,meta.json 記錄請求值 |
| `control/imu_aliasing_fix.parm` | Stage 1 = batch logging(`INS_LOG_BAT_MASK=1` + `INS_LOG_BAT_OPT=5` sensor-rate+前後濾波對照);Stage 2 = notch 參數(註解狀態,FREQ 等槳測頻譜填,REF 已填實測 hover 油門 0.22);`load_fc_params.py --file` 載入 |
| `vio_eval/imu_vibe_spectrum.py` | 前後對照:任何新錄的 imu.csv 換路徑即可比頻譜 |

**實機量測(2026-07-23,FC 上電後):**
- **400 Hz 申請被拒**:`cap_message_interval` 要求 `interval_ms*800 ≥
  loop_period_us`,`SCHED_LOOP_RATE=400` 下最小間隔 3 ms →
  **333 Hz 是可批准上限,實測達成 ~346 Hz**(≈ 87% 的 loop-rate 樣本;
  非整數倍抽樣不會產生同調摺疊,OpenVINS 用實際時戳積分,沒問題)。
  工具說明已改為建議 `--imu-hz 333`。250 Hz 申請實測 ~260 Hz 也可。
- FC 現況:`INS_ACCEL_FILTER` 已是 10 Hz(非預設 20)、`INS_GYRO_FILTER`
  26 Hz、gyro backend 2 kHz(`INS_GYRO_RATE=1`)、`MOT_THST_HOVER`
  已學得 0.218(→ stage 2 的 `INS_HNTCH_REF`)。
- **Stage 1 已載入 + 重開機 + 讀回驗證**(`INS_LOG_BAT_MASK=1`、
  `INS_LOG_BAT_OPT=5`)——batch logging 待命,arm 即開始寫。
- Jetson 端 FC 序列埠是 `/dev/ttyUSB0`(FTDI 轉接),不是 ttyTHS1;
  `load_fc_params.py` 預設值就對。

**取頻譜的方式(2026-07-23 定案:戶外實飛,不做綁機槳測):**
- **一定要上槳**:要量的是「槳」造成的震動——blade-pass 諧波 + 槳/馬達
  不平衡,且要在飛行 RPM + 氣動負載下。空轉馬達(無槳)轉速完全不同、
  震動小得多、譜線位置全錯 → notch 會擺錯地方。
- 綁機室內槳測有安全風險,且實飛的頻譜本來就更真(RPM、負載都對)——
  改為戶外短飛取樣。batch logging 是 arm 觸發,任何飛行都會寫。

### 14-7. 外場飛行計畫(下次出門,一趟收兩份資料)

**Flight 1 — 頻譜短飛(~2 分鐘,獨立一個 arm 週期):**
arm → 懸停 ~1 分鐘 → 一段明快的 20 m+ 爬升 → 降落 → disarm。
獨立 arm 週期很重要:每次 arm 開新 .bin,頻譜檔才小而無縫。

**Flight 2(選做但很值)— 正常 survey 航線:**
桌面 `field_data_collection.sh` 已改為帶 `--imu-hz 333`(2026-07-23)——
狀態列會顯示 `imu=346Hz` 左右,**這是正常值不是故障**。此錄製已消除
串流減採樣摺疊,回家即可重跑離線評估,單獨隔離出「333 Hz 串流」
這一項買到多少改善。

**回家後:** 拉 FC SD 卡的 .bin(flight 1 那個)→ FFT 找真譜線 →
填 Stage 2 notch(REF 已知 0.22)→ 載入 → **下一趟**飛行驗證完整修正。

**預期管理:** flight 2 的錄製仍含 notch 前的陀螺儀震動,是中間資料點、
不是最終判決;但它能單獨回答「光提高串流率值多少」。完整判決 =
notch 上機後的那一趟。

### 14-5. 補充 12-2:氣壓計在等高巡航仍有一條路——「當景深先驗」而非「當高度量測」

12-2 的結論(等高巡航氣壓計綁不了 XY 尺度)是針對「氣壓當**高度量測**融合」
——該結論不變,且標定後資料再次佐證(巡航高度已貼氣壓 ~9 m rmse,XY 尺度
照樣塌一半:等高飛行時垂直吻合與水平尺度幾乎解耦)。

但同一顆感測器換個用法就不一樣:**nadir 相機 + 已知 AGL = 每個地面特徵的
深度已知**(depth ≈ AGL/cos(視角))。已知深度 + 像素位移 = 公制平移,
**每一幀都直接觀測 XY 尺度,等高巡航也成立**(光流模組配測距儀同原理)。
數字驗證:實際巡航 ~6.8 m/s,VIO 以為 ~3.3 m/s——特徵三角化深度差 ~2 倍,
用氣壓 AGL 釘住即可拉回。實作路線:OpenVINS 加深度先驗殘差(客製),或
簡化版 homography-VO × AGL 當速度輔助。

修正尺度的優先序:(1) AGL 景深先驗、(2) AnyLoc 絕對修正(plan-B 融合,
兩者都直接觀測 XY 尺度)、(3) 純氣壓高度因子(治爬升發散與高度漂移,
治不了巡航尺度)。但這些都是**治標**(robust 地擋住症狀);14-3/14-4 的
IMU 混疊才是**治本**(加速度計守不住尺度的原因)。

### 14-8. 外場飛行完成 + FFT 結果 + Stage 2 已填(2026-07-23 晚)

**飛行記錄(`field_data/`):**
- **survey24 = Flight 1**(獨立 arm 週期,頻譜短飛):CTUN 顯示
  0-9s 地面、10-97s 懸停 @ ~0.9-1.0 m(`ThO` 0.218-0.222,剛好貼著
  `INS_HNTCH_REF=0.22`)、98-155s 爬升到 ~21 m 並穩定、之後下降降落
  (~180s 結束)。`.bin` 存在(`2026-07-23 17-32-24.bin`),batch logging
  確認有寫入 ISBH/ISBD。
- **survey25 = Flight 2**(正常 survey 航線,兼作 100 m 驗證飛行):
  telemetry 確認最高 AGL 100.7 m,飛行時長 ~473 s。`.bin` 存在
  (`2026-07-23 17-42-12.bin`)。
- **survey26**:同晚第三筆錄製,AGL 也到 ~100.5 m(~833 s),但**完全沒有
  `.bin`**——推測 survey25 結束後 FC 沒有重新 arm/重開機,沒有開新的
  dataflash 檔。尚未追查,先記錄下來。

**FFT 結果 — 新工具 `field_data/survey24/vibe_analysis/batch_fft.py`**
(未 commit):用 pymavlink 直接解 ISBH/ISBD batch sampler(確認取樣率
~2022.5 Hz,遠高於任何槳/馬達頻率,這一階段沒有串流減採樣的混疊問題),
依 CTUN 高度分出的時間窗還原 int16 x/y/z 原始樣本,做 Welch PSD。
FC 上有兩個實體 IMU 有記錄批次資料:instance 0 與 instance 2(instance 1
沒記錄)。

- **懸停窗(t=15-95s,油門剛好貼著 REF=0.22)**:陀螺儀 instance 0 出現
  一條乾淨的窄頻線,譜線落在 126.4-142.2 Hz 之間,**功率加權中心頻率
  134.3 Hz**(峰值 bin 在 136.3 Hz)。同一個 instance 的加速度計在
  ~260-266 Hz(~2 倍頻)也有對應線——確認這是真實機械線,不是混疊
  (兩側功率乾淨衰減,不像 survey17 那種「平到 Nyquist」的混疊地板)。
- **爬升/高懸停窗(100-150s,油門到 ~0.24)**:同一條陀螺儀線往上移到
  ~138-142 Hz——會跟著油門走,支持用 `INS_HNTCH_MODE 1`(油門追蹤)
  而非 FFT 追蹤模式 4。

**已填入 `control/imu_aliasing_fix.parm` Stage 2(未 commit)：**
```
INS_HNTCH_ENABLE 1
INS_HNTCH_MODE 1      # 油門追蹤
INS_HNTCH_REF 0.22    # 懸停 ThO 實測值,與 MOT_THST_HOVER 一致
INS_HNTCH_FREQ 134    # 懸停功率加權中心頻率
INS_HNTCH_BW 67       # FREQ/2
INS_HNTCH_HMNCS 3     # 1+2 諧波(2 倍頻在加速度計上有明顯線)
```

**下一步(尚未執行):**
`load_fc_params.py --file control/imu_aliasing_fix.parm --reboot` 把
Stage 2 載到 FC → 飛一趟驗證飛行(survey25/26 那種航線即可,notch 這次
是開著的)→ 拉新的 imu.csv(最好也拉一份新 .bin 目視確認那條線被壓下去)
→ 重跑 OpenVINS 離線評估([[openvins-first-offline-run]] 的流程)→
看爬升發散(~473s)與巡航 ~0.48x 尺度這兩個症狀是否真的移動。這才是
14-4 判決樹「完整修正」的那一趟。

**確認 survey24/25/26 是「notch 關閉」的基準資料(未濾波):** 直接讀
survey24 的 `.bin` 內部 `PARM` 訊息(FC 自己記錄的當下參數值,不是猜的)——
`INS_HNTCH_ENABLE = 0.0`(同時 `INS_LOG_BAT_MASK=1`、`INS_LOG_BAT_OPT=5`
確認 Stage 1 有效)。三筆飛行同一個 FC 開機期間錄製,參數沒動過,所以
三筆都是「notch 關閉」的原始資料——這正是找頻譜所需要的「修正前」基準;
134.3 Hz 那條線是 Stage 2 上機前的真實訊號,不是量測失誤。Stage 2 上機
後的「修正後」資料要等下一趟驗證飛行才會產生。

**這個改動會影響 FC 的 EKF 嗎?會不會造成飛行問題?**
- **會影響 EKF,而且是預期中、對 EKF 有利的方向。** EKF3 吃的是跟
  `RAW_IMU` 同一份「後端濾波後」陀螺儀/加速度計資料(GCS_Common.cpp
  `send_raw_imu` → `ins.get_accel/get_gyro`,14-4 已從原始碼確認)。
  notch 把 134 Hz 這條機械震動線在進 EKF 前就濾掉,等於給 EKF 一份更乾淨
  的陀螺儀輸入,而不是額外幫 EKF 做事——這是 ArduPilot wiki 對「有槳振
  的機體」的標準建議,不是本專案獨有的實驗性改動。
- **造成飛行問題的機率很低,理由:**
  1. **頻率隔得夠開**:notch 中心在 134 Hz,姿態速率控制迴路真正在意的
     頻段大約在 0-20 Hz。這麼遠的 notch 對控制迴路的相位延遲影響可忽略
     ——這正是 harmonic notch 敢用在控制迴路上的原因。
  2. **中心頻率是量出來的,不是猜的**:`REF=0.22` 對應 FC 自己學到的
     `MOT_THST_HOVER=0.218`,`FREQ=134` 來自懸停時那段實測 FFT,並有
     加速度計 2 倍頻線佐證。`MODE 1`(油門追蹤)會隨油門飄移自動跟著走
     (電壓/風力/酬載變化造成的懸停油門微調,本來就是這個模式設計要處理
     的情境)。
  3. **只動濾波,不動其他任何東西**:沒有改位置/速度來源、沒有改控制
     增益、沒有改飛行模式或地理圍籬邏輯,純粹是 INS 後端裡的一個濾波器。
- **真正需要注意的地方(小,不是安全疑慮)**:
  - `INS_HNTCH_ENABLE` 跟 batch logging 一樣是開機時讀取的參數,上機後
    需要重開機,重開機期間跟任何一次 FC 重開機一樣沒有控制權,這不是
    這個改動特有的風險。
  - 若頻率抓得不準,notch 頂多是「濾了但沒對到真正的震動」→ 沒有效果,
    不會反過來造成不穩定。
- **建議做法**:載入 + 重開機後,跟任何第一次改參數一樣,先做一次短時間
  繫留/低空懸停(Stabilize 或 Loiter)確認正常,再放心接回 survey/VPE
  流程——這是一般性的審慎作法,不是這個改動特別危險。事後拉一份新的
  `.bin` 重跑 `batch_fft.py`,直接看那條 134 Hz 線是否被壓下去,是比純看
  飛行手感更硬的驗證方式。

### 14-9. Stage 2 已上機 + survey25 OpenVINS 離線評估(notch 上機前基準,2026-07-23)

**Stage 2 已實際載入 FC 並重開機驗證**(`load_fc_params.py --file
control/imu_aliasing_fix.parm --reboot`,mavros 沒在跑,埠是空的):
`INS_HNTCH_ENABLE 0→1`、`FREQ 80→134`、`BW 40→67`(`MODE`、`HMNCS`、
`REF` 本來就已是目標值)。重開機後讀回全部確認:`ENABLE=1 MODE=1
REF=0.22 FREQ=134 BW=67 HMNCS=3`,Stage 1 batch logging 也還在。FC 現在
已經是「notch 開啟」狀態,等下一趟飛行驗證。

**同時拿 survey25(Flight 2,100 m survey)跑了一次 OpenVINS 離線評估**
——這筆資料本身仍是 notch 上機**前**錄的(§14-8 已用 `.bin` 內的 `PARM`
訊息確認 `INS_HNTCH_ENABLE=0`),所以是「只有 333 Hz 串流、還沒上 notch」
這個中間資料點,不是最終判決。

**方法與初版嘗試的教訓:**
- 直接用預設起點跑(靜態初始化從 t=0、動態初始化從巡航剛開始的 t=160s)
  結果整個爆掉(公里等級誤差)——查下去發現是方法問題,不是物理問題:
  - 靜態初始化在 t=0 起跑時,誤觸發在 t=60-80s(imu.csv 直接量到 accel
    std 衝到 0.4-0.9 m/s²,真實地面擾動,可能是上鎖/測試螺旋槳,之後
    t=80-110s 又乾淨),而不是在真正起飛的 t=112s——等於用錯誤的瞬間
    當「靜止→運動」轉換點,重力方向估壞,後面全部發散。
  - 動態初始化選在 t=160s(剛好巡航開始、幾乎等速前飛),沒有足夠的側向
    加速度可以觀測重力方向/尺度——速度估計像滾雪球一樣衝到數百 m/s,
    是典型的初始化重力沒對準的訊號。
  - 兩者都是「起跑點選不好」的方法論陷阱,跟 notch/震動混疊的問題無關。
- **改起跑點重跑(`_v2`)**:靜態初始化改到 t=100(剛好避開那段地面擾動,
  在真正起飛前);動態初始化改到 t=200(卡在 207-216s 那段 ~150° 大轉彎
  前,轉彎給的側向加速度夠讓濾波器觀測尺度)。

**結果(`field_data/survey25/vio_eval/`,`vio_result_summary.png`):**
- **爬升段(105-165s,0→100 m)追得很好**:水平 rmse 只有 1.4 m,高度
  曲線幾乎貼著氣壓/GPS AGL(這段是接近純垂直運動,GT 水平路徑只有
  ~2 m,「最佳擬合尺度」這個數字在這裡沒意義,直接看誤差比較準)。
  **survey17 那個「爬升必炸」的招牌症狀,這次沒有重現。**
  推測:333 Hz `RAW_IMU` 串流(這趟飛行本來就有開,跟 notch 無關)可能
  已經解掉 §14-4 解法 3 講的「400→200 Hz 串流減採樣無抗混疊」——爬升
  段本來被懷疑就是這個機制炸的,證據方向吻合,但只有一趟飛行、不同
  場地/日期,還不能當定論。
- **一進入等速巡航,誤差立刻炸開**:離開爬升段後 45 秒內,誤差從 30 m
  衝到 400 m,不管用哪個起跑點都一樣。在轉彎點動態初始化的巡航窗
  (200-230s,230 m 路徑):4DOF ATE2D rmse 42.9 m,**最佳擬合尺度
  0.417**,尺度修正後 rmse 33.3 m——跟 survey17 標定後的 0.34-0.48x
  幾乎是同一個數字。

**判讀:爬升好了,巡航尺度沒好,而且不意外——這正是下面這題要回答的。**

### 14-10. 問答:巡航尺度塌縮的解法,是不是就是前面上機的那個修正(notch)?

**不是。notch 對巡航尺度基本沒有幫助,理由在 §14-4 已經寫過,這裡點名
講清楚:**

- **notch 只濾陀螺儀,ArduPilot 完全不對加速度計做 notch**(§14-4 第 1
  點、原始碼已確認)。單目 VIO 的**尺度**是靠加速度計積分撐住的
  (角度/姿態才是陀螺儀的工作)——所以一個只作用在陀螺儀的濾波器,
  結構上就碰不到「巡航尺度塌縮」這個問題的病灶。
- 而且 batch log FFT(§14-8)量到的 134 Hz 震動線,在 333 Hz 串流下
  **本來就沒有摺疊**(333 Hz 的 Nyquist 是 166.5 Hz > 134 Hz)——意思是
  這條線在加速度計訊號裡是「原始頻率、原始強度」直接出現,不是混疊
  訊號。notch 上機後只會清掉陀螺儀那一份,加速度計那一份完全不受影響、
  強度不變。這解釋了為什麼這次 333 Hz(還沒上 notch)沒能改善巡航尺度
  ——它從來就不是這一段的病因,notch 之後大概率也一樣。

**真正對巡航尺度有機會的,是 §14-4/§14-5 已經排好的另外幾條路,和 notch
是平行的、不是同一個修正:**

1. **AGL 景深先驗(§14-5,優先序第一)**:nadir 相機 + 已知氣壓 AGL
   ⇒ 每個地面特徵的深度都已知(depth ≈ AGL/cosθ),深度已知 + 像素位移
   直接給出公制平移——**等速巡航也成立**,因為這條路根本不靠加速度計
   觀測尺度,靠的是「已知深度」這個幾何約束。這是唯一一個「巡航也有效」
   的方案,其他方案要嘛靠加速度計(觀測不到),要嘛只在有加速度變化的
   時候才有效(轉彎、爬升)。
2. **AnyLoc 絕對修正(plan-B 融合)**:外部絕對定位直接把飄掉的尺度拉
   回來,不解決 VIO 內部積分問題,但治標有效。
3. **物理減震(§14-4 第 4 點,槳平衡/FC 軟墊)**:這個反而是對加速度計
   最直接的解法——因為問題出在加速度計「原始震動能量太大」(不是混疊
   問題,是能量問題),notch 治不了的地方,physically 把能量源頭降下來
   卻可以。6C 沒有內建 IMU 隔震(6X 才有),槳不平衡也會直接加大這股
   能量。目前排序上覺得「先試 1、2」是因為工程量小,但如果 AGL 先驗
   做出來後尺度還是不夠穩,physically 減震其實是比 notch 更該做的下一步
   ——因為它才是對症下藥的加速度計修正。
4. **純氣壓高度因子**:治的是垂直漂移/爬升發散,不是巡航 XY 尺度
   (§12-2、§14-5 已證明兩者解耦),放在這裡是為了排除「有人會覺得這也
   算」的誤解。

**結論一句話:notch 是治爬升段可能的混疊問題(而且看起來可能已經被
333 Hz 串流順便治好了),巡航尺度塌縮是另一個病灶(加速度計原始震動
能量,不是混疊),要另外治——AGL 景深先驗是唯一在等速巡航下仍然有效
的方案,物理減震是對加速度計最直接的手段,兩者都還沒做。

### 14-11. 巡航尺度塌縮:病灶是 IMU 還是視覺里程計?(2026-07-23/24,實測診斷)

沒有停在猜測,直接從 survey25 的軌跡資料挖證據,三條線索都指向同一個
結論:

**1. 震動雜訊量級不是巡航特有的**——爬升段(scale 成功 ≈0.99)accel std
z 軸 0.343、xy 0.10-0.12;巡航段 std 反而更低或相當(0.2-0.3)。震動能量
整段飛行都在,不是巡航才變吵——排除「巡航時 IMU 突然變爛」這個說法。

**2. 加速度計 bias(ba)在轉彎那一刻壞掉,壞法是「凍結」不是「變吵」**:
static-init 全程(`vio_full_v2.csv`)追蹤 ba 逐段變化——

| t 區間 | ba 平均 (x,y,z) | ba std | \|v\| 平均/max |
|---|---|---|---|
| 105-130s(爬升前段) | (0, -0.001, 0.019) | 小 | 1.2 / 1.9 m/s |
| 130-200s(爬升後段→巡航開始) | 逐步漂移到 (-0.07,-0.15,-0.05) | 中 | 1.6-9.5 m/s |
| **200-230s(轉彎)** | 突然翻正到 **+0.14**,std 衝到 **0.169** | 爆 | 9.6-14.5 m/s |
| 230s 之後 | 定在 (0.28,-0.01,-0.02) 不再變 | **std=0.0000(凍結)** | 30→127 m/s(完全不合理) |

bias 在轉彎當下先爆衝、之後直接凍結不再更新,同時速度像雪球一樣衝到
物理上不可能的數值——這是狀態變得「不可觀測」、濾波器放棄修正、之後
純靠(震動汙染的)IMU 外推的典型特徵。

**3. 旋轉追蹤在直線段幾乎完美,一到轉彎就壞掉、之後回不來**——用軌跡
的連續位移反推 OpenVINS 的對地航向,扣掉一個固定偏移量後跟 GPS 航向比:

| 時段 | 段落 | 扣除固定偏移後的航向殘差 |
|---|---|---|
| 195-205s | 轉彎前直線段 | **≈0°**(幾乎完全吻合) |
| 207-216s | ~150° 大轉彎 | 跳到 -30°~-100° |
| 217-244s | 轉彎後 | 停留在 -30°~-110°,**沒有恢復** |

直線段陀螺儀+視覺融合的姿態精準到 <1°;一進入實際機動(轉彎)姿態估計
就整個跳掉,而且之後視覺更新也沒能把它拉回來。

**判讀:不是泛用的 IMU 故障(雜訊量級整段一致,爬升段雜訊相當甚至更高
卻能追得很準),病灶在轉彎那個瞬間的視覺端**,兩個原因疊加:
- ⚠️ **IMX219 是 rolling shutter**(`vio_data_collection.md` §6 已知風險)
  ——**這個猜測 2026-07-24 深夜被直接測試並且不成立,見 §14-16**,
  當時只是列為合理懷疑,從沒真的驗證過。快速滾轉/偏航時每一列影像
  曝光時間點不同,理論上會造成整張影像幾何失真,OpenVINS 假設
  global shutter,KLT 光流在這種畫面上追蹤到的可能是系統性扭曲後的
  錯誤位移。
- **100 m AGL 等速巡航本身視差就弱**(記憶:實際巡航 ~6.8 m/s,VIO 以為
  ~3.3 m/s),平常視覺端已經沒什麼餘裕,轉彎這唯一能提供強尺度線索的
  動態事件,偏偏又是追蹤最容易壞掉的時候——這部分仍然成立,§14-16
  只推翻「rolling shutter 是病灶」這一半,沒有推翻「轉彎瞬間視覺端
  失效」這個總體判讀。

**結論:巡航尺度塌縮的觸發點是視覺端在轉彎時失效,IMU 的 bias 凍結/
速度暴衝是下游結果,不是起因。** 這解釋了為什麼 notch(只濾陀螺儀)
對這個問題沒有直接幫助,也支持 §14-5 AGL 景深先驗(不靠加速度計觀測
尺度)是正確方向。

### 14-12. 用陀螺儀資料頂過轉彎:兩種實作 + 一個有意思的反直覺結果(2026-07-24)

**先回答一個問題:能不能反過來用 IMU 修正視覺,而不是視覺修正 IMU?**
結構上不行(長期而言)——OpenVINS 本來就是 IMU 連續遞推(propagate)、
視覺周期性修正(update)這個方向,不能反過來,因為純 IMU 死算
(尤其加速度計雙重積分)會無界發散,這正是視覺存在的理由。但陀螺儀
在**短時間**(單一轉彎的 10-20 秒)非常準(典型 MEMS 陀螺漂移每秒遠低於
1°),遠比壞掉的視覺量測可靠——所以「探測到快速轉動時,短暫多信任
陀螺儀、少信任視覺」是站得住腳的做法,不是全面翻轉信任關係。

**實作 1:硬閘門(`run_video_msckf_gyrogate`,新增檔案,`~/openvins_ws/
src/open_vins/ov_msckf/src/`)**:量測每張影像對應那批 IMU 樣本的最大
角速度量值,超過門檻(0.20 rad/s,由資料本身定出——直線飛行 p95 僅
0.24 rad/s,轉彎量到 0.30-0.46 rad/s)就整個跳過該幀的相機 feed
(`feed_measurement_camera`),形同該幀完全靠 IMU 遞推。

結果(`field_data/survey25/vio_eval/vio_cruise_gyrogate.csv`,
`vio_gyrogate_compare.png`):

| 窗口 | 未閘門 | 硬閘門 |
|---|---|---|
| 200-230s(轉彎) | rmse 42.9m,scale 0.417 | **rmse 12.4m,scale 0.950** |
| 200-260s | rmse 695m,scale 0.153(已發散) | rmse 51.2m,scale 1.158 |
| 200-320s | 完全發散 | rmse 2161m(最終仍發散) |

第一個轉彎立即大幅改善(尺度從 0.42x 拉回接近 1:1)。但這是 mowing-lawn
式的 survey,一趟航線有好幾個轉彎點(207-216s、224-238s、255-271s、
288-293s...),硬閘門讓相機在轉彎時完全不被看見,追蹤器每次轉彎後都要
從零重新啟動——全程約 25% 的幀被跳過,每個轉彎都重新付一次「重新啟動」
的代價,誤差逐彎累積,最終還是在 ~280-320s 左右發散。

**實作 2:軟閘門(修改 OpenVINS 本體)**——不跳過相機 feed(追蹤/clone/
邊緣化全部照常跑,轉彎後不必重新啟動追蹤),只在高角速度時跳過**視覺
EKF 修正**這一步:

- `ov_msckf/src/core/VioManager.h`:新增 `set_skip_update(bool)`(專案
  自訂,非 OpenVINS 上游),布林旗標 `skip_visual_update`。
- `ov_msckf/src/core/VioManager.cpp` `do_feature_propagate_update()`:
  把 `updaterMSCKF->update(...)`、`updaterSLAM->update(...)` 迴圈、
  `updaterSLAM->delayed_init(...)` 三處全部包進
  `if (!skip_visual_update)`,其餘(追蹤、clone、邊緣化、cleanup)不變。
- 重新編譯 `libov_msckf_lib.so`(`make ov_msckf_lib`),新 feeder
  `run_video_msckf_gyrogate_soft.cpp`:每幀都呼叫
  `sys->set_skip_update(gate_this_frame)` 再照常 `feed_measurement_camera`。
- **踩到一個地雷,修了**:第一次跑到中途 `EKFUpdate() - diagonal at
  18/92 is -0.00`(共變異數矩陣數值壞掉),原因是 `propagator->
  invalidate_cache()` 原本只在真的執行 update 後才呼叫;跳過 update 的
  那幾幀沒清快取,導致之後某幀重用了跟目前(已被跳過更新的)狀態不一致
  的舊快取。修法:把 `invalidate_cache()` 移到 `propagate_and_clone()`
  之後**無條件**執行,不管這幀是否跳過視覺更新。重編後乾淨跑完
  (`vio_cruise_gyrogate_soft2.csv`)。

**結果比較(3 種都在同一段轉彎測試,`dyn init @ t=200s`):**

| 窗口 | 未閘門 | 硬閘門(丟相機) | 軟閘門(留追蹤) |
|---|---|---|---|
| 200-230s(轉彎) | rmse 42.9m / 0.417 | **rmse 12.4m / 0.950** | rmse 43.8m / 0.457 |
| 200-260s | rmse 695m / 0.153 | rmse 51.2m / 1.158 | rmse 66.4m / 0.841 |
| 200-320s | 完全發散 | rmse 2161m | rmse 960m(略好一點,仍發散) |

**意外的地方:軟閘門在「當下這個轉彎」幾乎沒有改善(0.457 vs 未閘門
0.417),遠不如硬閘門的 0.950。** 這比原本預期的「軟閘門應該兩邊優點
都拿到」更值得記錄——推論:軟閘門雖然沒有把視覺更新餵進濾波器,但
KLT 追蹤器仍然在 rolling-shutter 失真的畫面上持續跑,失真已經烙進
追蹤到的特徵點位移歷史裡;等閘門解除、這些累積的追蹤結果被「兌現」
進更新時,同樣的錯誤幾何還是進來了,只是延後了幾幀而已。硬閘門反而
因為完全沒看那幾幀畫面,轉彎後追蹤器是乾淨重啟,沒有把失真資訊留下來
——「不看」比「看了但先不信」在這個失效模式下更有效。

**這個結果進一步強化了 rolling shutter 是病灶的判斷**(不是「視覺量測
剛好在那個瞬間不準」,是「那整段影像資料本身就是壞的」,延後使用不能
消除失真,只能不使用)。真要讓軟閘門式的做法有效,還需要在閘門解除
時一併丟棄閘門期間累積的追蹤觀測(不只是延後使用),這是更複雜的下一步
精修,目前未做。兩種閘門實作在整趟多轉彎 survey 上最終都在 ~280-320s
左右重新發散——單一機制的局部修補頂不住「一路都是轉彎」的巡航航線,
AGL 景深先驗(§14-5,完全不依賴視覺撐過轉彎)仍是唯一結構上站得住腳
的方案。

**檔案(均未 commit)**:
`~/openvins_ws/src/open_vins/ov_msckf/src/run_video_msckf_gyrogate.cpp`
(硬閘門 feeder)、`run_video_msckf_gyrogate_soft.cpp`(軟閘門 feeder)、
`ov_msckf/src/core/VioManager.{h,cpp}`(軟閘門本體修改,含 cache 地雷
修正)、`field_data/survey25/vio_eval/vio_cruise_gyrogate*.csv` +
`vio_gyrogate_compare.png`。

### 14-13. AGL 景深先驗修好了 + 與陀螺閘門合併評估(2026-07-24)

§14-5/14-12 都指向 AGL 景深先驗是唯一結構上不靠視覺撐過轉彎的方案,但
它從沒被成功評估過——`vio_cruise_aglprior.csv` 到 `aglprior4.csv` 這 4
次嘗試全部發散到 20-150 km 誤差,比什麼都不做(ungated baseline ~400 m)
還糟。今天的任務:修好它,然後依 Frank 的指示評估「陀螺閘門扛轉彎瞬間、
AGL 先驗全程跑」的合併方案,而不是二選一。

**原本猜的 bug 不是真正原因。** 出發前的假設是 `cos_view` 算的是相對
相機光軸的角度,不是相對世界垂直方向——在轉彎傾斜時這兩者會不一樣。
這個假設被直接檢驗過(在 207-216s 轉彎段對比光軸版本 vs. 世界垂直修正
版本的 `cos_view`)並且**排除**:兩者幾乎一致(平飛時 0.926 vs 0.916,
轉彎時 0.933 vs 0.914)。這趟飛行的轉彎以偏航為主,nadir 相機全程幾乎
維持垂直,所以這個理論在這份資料上不成立。

**真正原因是建置系統,不是幾何算式。** `run_video_msckf_aglprior`、
`_gyrogate`、`_gyrogate_soft` 從來沒被登記成真正的 CMake target——
`CMakeFiles/<name>.dir/` 底下只有裸的 `.o` 檔,從沒生出過
`flags.make`/`link.txt`。`make run_video_msckf_aglprior` 因此每次都靜默
無效(GNU Make 認為「檔案已存在且沒有規則」就是已經是最新的)——先前 4
次「嘗試」很可能每次都在重跑同一份 Jul-24 00:37 的舊 binary,不管兩次
嘗試之間原始碼改了什麼,那些改動從沒被真正測試過。另外,在有 source
ROS2/`ament_cmake` 的 shell 裡 `cmake .` 重新配置本身就是壞的(這專案的
`ROS2.cmake` 路徑需要一個從未建置過的 `ov_core` ament 套件)——當初能編
出來的那次一定是在一個 catkin 和 ament 都沒 source 的 shell 裡做的。

**修法:** 直接照記錄下來的 `flags.make`/`link.txt` 手動重編
`UpdaterMSCKF.cpp.o` 並重新連結 `libov_msckf_lib.so` +
`run_video_msckf_aglprior`(這次繞過 `make`/`cmake`),並在
`ov_msckf/cmake/ROS1.cmake` 補上三個 binary 正式的
`add_executable`/`target_link_libraries`,讓乾淨(non-ROS2)shell 裡的
從頭建置以後能自動生出它們。**深度先驗的演算法本身沒有改**——原本寫在
硬碟上的邏輯(blend 0.5 拉向 AGL 深度、修正過的 feature 12x sigma
放寬)本來就合理,只是從沒被真的編進實際在跑的 binary。

⚠️ **still broken:** ROS2-sourced shell 裡 `cmake .`/`make` 重新配置
依然壞的,之後要改這塊要嘛換一個 catkin/ament 都沒 source 的 shell,
要嘛把 `ov_core` 建成 ament 套件裝起來。

**結果(survey25 巡航,對 GPS 4DOF 對齊,t=200-220s 定住轉換,不逐窗
重新對齊):**

| 窗口 | ungated | 硬閘門 only | AGL 先驗 only(修好後) | 合併 |
|---|---|---|---|---|
| 200-230s(轉彎) | rmse 66.0m | **rmse 13.9m** | rmse 39.3m | rmse 31.3m |
| 200-260s | rmse 1185m | **rmse 108m** | rmse 152m | rmse 381m |
| 200-320s | rmse 11136m | rmse 2670m | **rmse 539m** | rmse 1347m |

（跟 §14-12 的硬閘門數字對不太起來,如 13.9m vs 42.9m,應是對齊窗口
細節差異;方向一致,硬閘門在轉彎瞬間依然明顯最好。）

**判讀:合併沒有打贏任何一個單獨機制。** 這是誠實的結果,不是預期中
的那個。合併方案在每個窗口都卡在兩個單獨機制中間,從沒衝到第一:
- 轉彎瞬間(200-230s)和之後(200-260s),**硬閘門單獨最好**;合併比
  硬閘門單獨還差,因為閘門擋掉相機那幾幀的同時,AGL 先驗也沒東西可以
  修正(沒有相機 feed 就沒有 feature 可以重新定深度)——合併沒有在
  閘門生效的那些時刻疊加 AGL 先驗的好處,只是疊加了硬閘門每次轉彎的
  重啟代價。
- 拉長到全程(200-320s),**AGL 先驗單獨是四個方法裡最好的**——
  rmse 539m,遠勝硬閘門的 2670m 和合併的 1347m。這比 §14-12 任何一個
  單獨陀螺閘門版本的數字都好得多,也是 AGL 景深先驗第一次被成功評估
  出正面結果。
- **四個方法沒有一個撐過整趟多轉彎 survey**:到 t≈470s,全部發散到
  數千到兩萬公尺量級(見 `method_comparison_timeseries.csv` 最後幾
  行)。跟 §14-12 的結論一致——沒有單一局部機制頂得住一路都是轉彎的
  航線,只是現在清楚了:AGL 先驗單獨、而不是合併方案,是目前試過最強
  的那個。

**實務含意:** §14 那個「陀螺閘門扛轉彎、AGL 先驗跑全程」的合併計畫在
這份資料上不成立——兩個機制在搶同一批相機幀,不是疊加關係。AGL 先驗
單獨是更值得繼續投入的方向;如果還想要轉彎時的額外強健性,硬閘門這種
「完全跳過那幾幀」的合併方式很可能是錯的組合方式(它在閘門生效的那幾
幀把 AGL 先驗的訊號也一起丟了)——比較合理的下一步是像 §14-12 軟閘門
那樣,轉彎時保留 AGL 先驗繼續跑、只跳過視覺 EKF 更新那一步(同一類
cache 地雷很可能會在這裡重演)。

**檔案(均未 commit)**:`field_data/survey25/vio_eval/
vio_cruise_aglprior_fixed.csv` / `run_cruise_aglprior_fixed.log`(AGL 先驗
單獨,可用的 build)、`vio_cruise_combined.csv` / `run_cruise_combined.log`
(AGL 先驗 + 陀螺閘門 0.20 rad/s 合併)、`method_comparison.py`(沿用
`vio_gyrogate_compare.py` 的 4DOF 對齊方法)、
`method_comparison_timeseries.csv`(1 Hz,t=200-472s,GPS + 四個方法對齊
後的本地 x/y,發散段刻意保留沒截斷)、`method_comparison_summary.json`
(完整數字表 + bug/修法說明)、完整過程與後續建議見
`field_data/survey25/vio_eval/session_2026-07-24_agl_prior_gyrogate_combined.md`。
修改:`~/openvins_ws/src/open_vins/ov_msckf/src/update/UpdaterMSCKF.cpp`
(加了無害的 `AGL_DEBUG_COSVIEW` env-gated 除錯輸出,留著,不設環境變數
就不會動)、`ov_msckf/cmake/ROS1.cmake`(補上三個 binary 的 build
target)。

### 14-14. 換路線:不修濾波器,直接在輸出端修尺度——全程貼住 GPS 的第一個結果(2026-07-24 晚)

Frank 的判斷:survey17 的巡航段路徑形狀跟 GPS 幾乎一模一樣,只有尺度
不對——那為什麼不直接修尺度,而要一直動濾波器內部?先驗證這個判斷
在 survey25 上是否成立(滑動 30s 視窗 similarity fit,GT 靜止的視窗
剔除):**成立**——AGL 先驗輸出到 ~290s 為止 shape-rmse 一直維持
18-27m(形狀是對的),同時 best-fit 尺度在 0.55→2.44→0.40 亂晃(壞的
是尺度,不是形狀)。真正的發散(速度暴衝)要到 ~280-310s 才開始。

**兩個新發現(都很重要):**

1. **全程 static-init + AGL 先驗的完整航程 run 不再於巡航開始時崩潰**
   (`vio_full_aglprior.csv`,112-472s 全程跑完)——之前所有 full run
   (§14-9 vio_full_v2)一進平飛就 30→400m 崩掉,這是第一次撐過整段
   巡航(167-319s 所有航線腿+所有轉彎),形狀 per-window 2.7→21→12.6m,
   只有尺度逐步塌(1.08→0.30→0.56→0.11),高度全程貼 baro(rmse
   1.7m)。真正暴衝從下降段(~310s)才開始。
2. **輸出端因果尺度修正 + 錨點拉回(`scale_corrector.py`)給出全專案
   第一個「全程有界」的軌跡**:170-472s(含下降段暴衝)rmse **76.0m /
   max 130m**(原始 VIO 同段 rmse 7703m、終點差 20km)。標準比較窗口:
   200-230s rmse 34.4m、200-260s **31.0m**、200-320s **40.5m**——除了
   轉彎瞬間輸給硬閘門(13.9m)之外全面領先,而且是唯一不發散的。

**修正器的組成(全部因果、全部不用 GPS,Python 後處理,零 C++ 改動):**
- **尺度估計**:爬升/下降段用 baro(尾隨 20s 窗 Δbaro/ΔVIOz,|Δ|≥3m
  才更新)——爬升段把尺度釘在 1.00;平飛段 baro 無訊號(§12-2 實測
  90s 內氣壓高度差 <2m),改用 **AnyLoc 式絕對定位點的弧長比**(50m
  網格量化 + 2s 週期模擬 plan-B 輸出;用弧長不用弦長——弦長在 150°
  轉彎時是幾何相關的垃圾,實測會讓尺度飆到 3.27,弧長對轉彎不變)。
  EMA 只在新錨點到達時更新(每 2s),clamp [0.2,5]。
- **位置增量修正**:p[k] = p[k-1] + s·Δp(修增量不修絕對位置,因果)。
- **錨點拉回**(plan-B 融合角色):每個錨點把位置往絕對定位點拉
  30%——這是修 heading 漂移的部分,純尺度修正修不了轉彎後 -30~-110°
  的航向跳掉(§14-11)。座標框一次性因果鎖定(前 150m 共同運動擬合,
  真實任務中這個框從 EKF origin+羅盤直接就有)。
- **速度包絡 clamp**:修正後增量隱含速度 >20 m/s 就截掉(WPNAV_SPEED
  =12,物理上不可能更快)——下降段暴衝(VIO |v| 飆到 270m/s)靠這個
  + 錨點拉回壓住。

**誠實的兩個 caveat(不能吹過頭):**
1. 理想化的錨點流「單獨」就有 rmse 14.3m(50m 量化的下限)——融合的
   價值不是打贏錨點,是平滑度+斷訊韌性。真實 AnyLoc 比這個 proxy 髒
   得多(domain gap、outlier、斷訊,見 real-video 實測記憶),所以
   實戰差距會縮小,但這個數字必須講清楚。
2. **斷訊測試(70s 錨點斷訊、橫跨轉彎 2-3)**:融合退化到 rmse 254m,
   跟「錨點 hold-last」(259m)沒差——因為斷訊窗剛好蓋住轉彎,而
   轉彎正是 VIO 自己壞掉的地方(§14-11)。VIO 層的橋接價值只在直線
   段存在;轉彎的病灶(rolling shutter)在輸出端修不掉,仍是未解問題。

**最終建議參數**:`--mode both --anchor-win 40 --ema 0.5
--anchor-min-disp 150 --anchor-pull 0.30 --vmax 20`。

**檔案(均未 commit,`field_data/survey25/vio_eval/`)**:
`vio_full_aglprior.csv`/`run_full_aglprior.log`(全程 AGL 先驗 run,
static init @ t=100)、`vio_full_aglprior_scalefix.csv`(修正後、最佳
軌跡)、`scale_corrector.py`(修正器本體)、`scale_fix_analysis.py`/
`scale_fix_analysis.json`(形狀-尺度分解)、`scale_fix_eval.py`、
`method_comparison_timeseries.csv` 增加 `scalefix_x/y` 欄、
`method_comparison_summary.json` 增加 scalefix 指標 + `best_method`
說明。互動比較圖(五條軌跡):
https://claude.ai/code/artifact/9c8e8321-101f-4ed6-9988-ae5dc10c769a

**下一步(依此結果重新排序):**
1. 把 `scale_corrector.py` 的邏輯搬進線上系統(plan-B 融合節點已有
   絕對定位點和 α-blend 架構,缺的是尺度層——工程量小)。
2. 轉彎病灶(rolling shutter)仍未解:斷訊測試證明輸出端修不掉它。
   若要徹底解,§14-12 的「軟閘門+丟棄閘門期間追蹤觀測」或 global
   shutter 相機仍在桌上。
3. 真實 AnyLoc(非 proxy)接進來重跑同一條 pipeline,量測真實 domain
   gap 下的表現。

### 14-15. FoundLoc 兩個具體做法移植 + 用真實雜訊等級誠實測試(2026-07-24 深夜)

Frank 找到 FoundLoc(He et al., CMU AirLab, arXiv:2310.16299)並問「是不
是更好的解法」。查證後:**FoundLoc 不是更好的 VIO**——它的 VPR 骨幹
就是本專案已經在用的 AnyLoc-DINO(作者名單跟 AnyLoc 論文重疊,論文
致謝直接感謝 AnyLoc 團隊部署支援),VIO 端反而是更陽春的自製版本
(Harris corner+LK 光流+5-point RANSAC,NVIDIA VPI 加速)。而且論文
自己的數字顯示「高度變化」是 FoundLoc 表現最差的類別(ATE 22.7m,
因為沒做相機傾角修正)——跟本專案一直在打的高 AGL/傾斜問題是同一種
坑,不是被解決了。**真正有效的是它的融合架構**:VIO+VPR+重力約束
sliding-window 剛體對齊+EKF,在已知初始姿態的 VIO-only baseline
(ATE 30.9m,SD 31.3)之上把誤差壓到 16.4m(SD 2.8)。

**只移植兩個具體、可驗證的機制**(不是整套系統):

1. **DBSCAN 假陽性過濾**(對應論文 III-E-3):論文是對「同一次查詢的
   top-N 檢索候選」做空間分群,取最大群、丟掉散落的假匹配。本專案
   目前每次查詢只模擬單一匹配(沒有 top-N 可分群),對應改法是論文
   自己在 III-B 用的「時間軸滑動視窗」同一招:把最近 N 個錨點對「目前
   估計」的殘差拿來分群,只用最大群的錨點去更新尺度,離群的視為假
   陽性丟棄。
2. **degeneracy-aware 框架鎖定**(對應論文 III-B 式 3 的重力約束項):
   論文的 3D ICP 在 UAV 直線飛行時位置點共線,純位置擬合解不出旋轉
   (rotation ambiguity),論文用 IMU 重力方向打破這個退化。本專案的
   對齊已經限制在平面 SE(2)(不解 roll/pitch),論文那個 3D 項不能
   直接套;對應的類比做法是用同一種「IMU 提供的獨立旋轉資訊」——
   陀螺儀積分出的相對航向(直線飛行時依然良態,不像位置擬合的 yaw
   會退化)——鎖定視窗共線時拿來交叉驗證位置擬合出的 yaw。

**誠實測試(這是這輪的重點,不是移植本身)**:第 14-14 節的乾淨錨點
proxy(50m 網格量化、零假陽性)過於樂觀——真實 AnyLoc 在本專案自己
的實測(`real_video_constrained_search_failure` 記憶)誤差是 mean
703-943m(domain gap 嚴重時)。這次改用有真實假陽性率的模擬錨點流
(20% 查詢是假陽性,誤差 200-900m,取自本專案自己的實測範圍),
测 DBSCAN 過濾是否真的有用(對應論文自己的 FoundLoc vs FoundLoc-NF
消融實驗)。

**過程中抓到一個真的 bug**:第一版殘差算法用「原始未修正 VIO 位置」
算殘差,而未修正 VIO 尺度本身就在漂移——殘差因此隨飛行時間單調爆炸
(好錨點 median 殘差 677m,比假陽性的 929m 還大!),假陽性訊號被
VIO 自身漂移完全淹沒。修法:殘差必須用「目前修正後的估計」(`pc`,
跟框架鎖定用的是同一個)算,不能用原始 `p`。

**參數掃描結果(eps 60→800,5 個隨機種子)**:eps 太小(60-150)連
好錨點一起拒絕,整體反而比不過濾還差;**eps=300, min_samples=2,
window=8** 是甜蜜點——拒絕 ~40% 真假陽性,**好錨點零誤殺**,5 個
種子中 4 個 rmse 比不過濾好(最多降 30%),第 5 個打平,沒有一次
變更差。

**最終結果(全程 170-472s,含下降段)**:

| 方法 | rmse | max |
|---|---|---|
| 原始 VIO | 7703m | 20334m |
| 第 14-14 節乾淨錨點修正(不誠實的上限) | 76.0m | 130m |
| 雜訊錨點,不過濾(FoundLoc-NF 類比) | 145.6m | 347m |
| 雜訊錨點 + DBSCAN 過濾(FoundLoc-style) | **137.1m** | 362m |

**判讀**:FoundLoc 的兩個機制在真實雜訊下確實有正向、可重現的效果
(過濾贏不過濾),但不如第 14-14 節那個乾淨 proxy 數字好看——那個
數字本來就不該被當作目標,137m 才是誠實的比較基準(見下方——這個
數字後來又進一步改善到 93.8m)。跟這趟飛行所有其他方法比
(ungated/gyrogate/aglprior/combined 全程 rmse 都是 4000-31000m
量級),仍是全場最好、也是唯一 max error 維持三位數的方法。

**Frank 追問「為什麼不是 FoundLoc 的 20m」——直接查證,不是猜**:
量化「錨點之間原始 VIO 會漂多遠」——尺度壞掉的時刻,單一 2s 錨點
間隔內原始 VIO 自己就能報出多達 76m 的位移(t=300s 附近實測)。
`anchor-pull=0.30`(每次錨點只拉回 30%)明顯追不上這個漂移速度。
掃描 pull 0.3→0.8(6 個種子):**0.60 是穩健最佳值**(平均 rmse
81.2m,pull=0.30 是 120.0m)——更高的 pull(0.8-0.9)在乾淨錨點下
持續變好,但在真實雜訊下反而停止進步甚至變差,因為它對「每一個」
錨點(包含漏網的假陽性)信任過頭,DBSCAN 視窗又因為信任太快而來不及
把它們群聚出來。改用 pull=0.6 重跑:

| 方法 | 舊(pull=0.30) | 新(pull=0.60) |
|---|---|---|
| 乾淨錨點修正(第 14-14 節) | rmse 76.0m / max 130m | **rmse 35.3m / max 78m** |
| 雜訊錨點 + DBSCAN 過濾(FoundLoc-style) | rmse 137.1m / max 362m | **rmse 93.8m / max 285m** |

乾淨錨點版本(35.3m)已經逼近 FoundLoc 論文的 ~20m;真實雜訊版本
(93.8m)仍有差距,原因很清楚且可解釋,不是回避:FoundLoc 論文的
底層 VIO 本身穩(known-init baseline ATE 只有 30.9m,不會像本專案
的 OpenVINS 在轉彎時尺度崩到 0.03-4x、發散到公里級),他們的下游
融合只需要修正「小幅漂移」;本專案的下游修正器要對抗的是「底層
VIO 間歇性完全失控」,同樣的融合架構天生更吃力。真正該打的仗仍是
第 14-11/14-12 節那個轉彎時 rolling shutter corrupt 視覺的病灶——
把底層 VIO 修穩,下游融合才有機會真正逼近 FoundLoc 的數字。

**檔案(均未 commit,`field_data/survey25/vio_eval/`)**:
`foundloc_corrector.py`(修正器本體,含雜訊錨點模擬+DBSCAN+
degeneracy-aware 鎖定)、`foundloc_eval.py`(評估+寫回比較 CSV/
summary)、`vio_full_foundloc.csv`/`run_full_foundloc.log`(過濾版)、
`vio_full_foundloc_nf.csv`/`run_full_foundloc_nf.log`(消融對照)、
`method_comparison_timeseries.csv` 新增 `foundloc_x/y` 欄、
`method_comparison_summary.json` 新增 `foundloc_integration` 說明。
互動比較圖(六條軌跡,含這輪):
https://claude.ai/code/artifact/9c8e8321-101f-4ed6-9988-ae5dc10c769a

### 14-16. 「真的是 rolling shutter 嗎?」——直接測試,結論:不成立(2026-07-24 深夜)

§14-11 把 rolling shutter 列為轉彎時視覺失效的兩個原因之一,但從沒被
真的測試過,只是合理懷疑。Frank 追問後直接查證,用三個獨立方法:

**方法 1:全程 14 個真轉彎事件的相關性分析**——用平滑後的 |ω|(0.5s
移動平均,避免震動雜訊誤觸發)在巡航段(165-325s)偵測出 14 個真實
轉彎事件(閾值 0.20 rad/s、最短持續 0.5s),對每個轉彎前後各抓 8s
直線窗口分別對 GPS 做局部 similarity fit,量測尺度值在轉彎前後的
「跳動量」當作損傷程度,跟三個轉彎特徵量做相關:

| 預測變數 | 相關係數 |
|---|---|
| 轉彎峰值角速度 | **r = 0.35**(弱) |
| 轉彎持續時間 | **r = 0.82**(強) |
| 轉彎總累積航向變化量 | **r = 0.79**(強,跟持續時間幾乎是同一件事,r=0.98) |

Rolling shutter 是逐幀(per-frame)幾何失真機制——每一幀的扭曲程度
取決於「拍那一瞬間」的角速度,理論上損傷程度應該主要跟著**峰值角
速度**走。實測結果剛好相反:損傷幾乎不看峰值多快,只看「轉了多久 /
轉了多少總角度」。這是錯的特徵signature。

**方法 2:全飛行單一最高角速度時刻的直接幾何測試**(t=237.34s,
|ω|=0.747 rad/s,全程最高)——對連續影格算 dense optical flow
(Farneback),擬合全域仿射變換(代表剛體旋轉+平移應該長什麼樣),
檢查殘差是否有 rolling shutter 該有的「跟影像列數(row index)線性
相關」的剪切訊號。三組相鄰幀對(270.90-271.03s)結果:row-index
相關係數全部 ≈0.000,frame 頂部與底部的殘差差異 <0.3px。

⚠️ 誠實的方法限制:全域仿射本身就能表達「純旋轉」的流場(對繞光軸
的偏航旋轉,流場本來就近似跟位置線性相關),理論上有可能把真的
rolling shutter 剪切也一起吸收進仿射項裡,讓這個測試對 RS 不夠敏感
——不能當作乾淨的否證,只能當作「沒找到正面證據」。單幀直線彎曲度
測試(找地面上該是直線的邊界,量測是否在單幀內彎曲)因為地面真實
邊界本身不夠筆直(找不到可靠的自動分割),結果不夠乾淨,數字甚至
反著走(最高角速度那幀彎曲度改善量反而比平飛幀更小)——列為未定論。

**方法 3:震動排除**——第 14-15 節已確認轉彎期間 IMU 震動沒有暴增
(配對 t-test p=0.26,反而平均略低)。排除「轉彎時因為壓坡度導致
震動突增」這個替代/干擾解釋。

**判讀:rolling shutter 不成立,至少不是主導機制。** 方法 1 的相關性
結果(累積時間/角度 >> 峰值角速度)跟「每幀幾何失真」的故事不合,
更像是「視覺追蹤在持續轉彎中逐漸累積失能(特徵遺失/誤配對),損傷
隨『暴露在壞追蹤狀態下多久』累積」——這是追蹤/re-acquisition 問題,
不是快門類型問題,理論上 global shutter 相機一樣會中招。方法 2/3 沒
找到 rolling shutter 該有的正面訊號(雖然方法 2 有已知局限,不算乾淨
否證)。

**實務含意**:§14-12 提過的「換 global shutter 相機」這個升級選項,
目前證據**不支持**——它針對的機制看起來不是主導原因。更有希望的方向:
陀螺輔助特徵預測(用 IMU 積分旋轉預測下一幀特徵該在哪裡,縮小 KLT
搜索範圍,幫助追蹤撐過旋轉)、或反直覺地讓轉彎更快而不是更慢(既然
損傷跟持續時間掛鉤,同樣的航向變化量,轉得快反而暴露時間短)。既有
在推進的方向(硬閘門/AGL 先驗/輸出端融合)都不依賴「查出正確機制」
本身,可以繼續。

**檔案**:分析直接在對話中完成,未產生新的持久化腳本(用到的圖片
`/tmp/.../frame_*.png` 為暫存,未存入專案)。

### 14-17. 陀螺輔助 KLT 追蹤——照 §14-16 的病灶直接下刀(2026-07-25 凌晨)

Frank 問「解法是什麼」,依 §14-16 修正後的病灶(追蹤在持續轉彎中累積
失能,不是逐幀幾何失真)直接查證機制:OpenVINS 的 KLT 追蹤器搜索窗
只有 15×15px,且**完全沒有運動預測**——「predicted new features」
其實就是 `pts_left_new = pts_left_old`,單純複製上一幀位置當初始猜測
(`TrackKLT.cpp:135`)。這趟飛行峰值角速度(0.75 rad/s)純旋轉造成的
單幀特徵位移可達 ~38px——是搜索窗的兩倍以上,難怪追蹤跟不上。

**實作(C++,`~/openvins_ws`)**:用 IMU 積分出兩幀之間的旋轉量
(exponential map,套用跟 `Propagator::predict_mean_discrete` 完全一致
的旋轉合成慣例,不是憑空猜的),轉到相機座標系
(`R_cam_delta = R_ItoC * R_imu_delta * R_ItoC^T`),在 `TrackKLT`
產生初始猜測時用這個旋轉去 warp 每個舊特徵點的位置(undistort→旋轉→
re-distort),取代原本的原地複製。新增 CLI 開關
`gyro_predict_track 0|1` 讓同一個 binary 能做乾淨的 A/B。方向性經過
實測驗證(不是只信公式):開啟後原始 VIO 大幅改善,證明旋轉方向是對
的(如果搞反符號,追蹤只會更差,不會更好)。

**機制面驗證(直接檢查是不是真的解決了診斷出的病灶)**:14 個轉彎的
per-turn 損傷,峰值角速度相關性 r=0.37→**0.03**(幾乎歸零——正是
「補償瞬時旋轉」該有的效果);持續時間相關性 r=0.82→**0.61**(降低
但沒有消失,代表還有第二個跟持續時間掛鉤的機制沒被這個修正處理到,
可能是視差退化或其他累積誤差,不只是追蹤補償的問題)。

**端到端結果(全程 170-472s)**:

| | 沒開 gyro-predict | 開了 gyro-predict |
|---|---|---|
| 原始 VIO | rmse 7703m / max 20334m | **rmse 2722m / max 4784m** |
| 乾淨錨點 scalefix(pull=0.6) | rmse 35.3m / max 78m | rmse 35.6m / max 73m(打平;但 200-230s 轉彎窗口 25.8→**14.1m**,追平硬閘門的 13.9m) |
| 雜訊錨點 FoundLoc-style(6 seed 平均) | rmse 81.2m | rmse 87.2m(打平,單一 seed 一度看起來變差,6-seed 才看出其實只是雜訊範圍內) |

**誠實判讀:這是個真的、機制上驗證過的修正,但效益沒有乾淨地傳遞到
下游。** 原始 VIO 大幅變好(65% rmse 降幅、76% max 降幅)確認病灶抓對
了,也確認方向沒搞反。但套上已經針對「舊的、比較爛的」原始訊號調好
的輸出端融合層(scalefix/foundloc),整體幾乎打平——融合層的參數是
為了吃舊訊號的雜訊特性調的,換了更好的原始訊號後沒有重新調整,增益
大部分被融合層自己的雜訊/更新節奏吃掉了。這不是壞消息,是預期中的
「下一層還沒跟上」——下一步應該是針對 gyro-predict 後的新訊號重新調
scale_corrector/foundloc_corrector 的參數(pull/ema/eps),而不是懷疑
這個修正本身無效。另外:兩個版本在下降段仍然都大幅發散,這是另一個
還沒查的獨立失效模式。

**檔案(均未 commit)**:`~/openvins_ws/src/open_vins/ov_core/src/track/
TrackBase.h`(新增 `set_rotation_prediction`)、`TrackKLT.h/.cpp`(新增
`predict_keypoints_rotation`,取代原地複製)、`ov_msckf/src/core/
VioManager.h/.cpp`(新增 `set_gyro_rotation_prediction` 轉發)、
`ov_msckf/src/run_video_msckf_aglprior.cpp`(新增 `integrate_gyro_rotation`
+ CLI 開關)。`field_data/survey25/vio_eval/`:`vio_full_aglprior_
gyropredict.csv`(+`_scalefix`/`_foundloc` 版本)、`turn_damage_analysis.py`
(14 轉彎機制驗證腳本)。互動比較圖已更新判讀文字:
https://claude.ai/code/artifact/9c8e8321-101f-4ed6-9988-ae5dc10c769a

**下一步**:針對 gyro-predict 後的新訊號重新調 scale_corrector/
foundloc_corrector 參數;查下降段獨立發散原因;§14-16 沒被這次修正
消除的殘餘 duration 相關性(r=0.61)值得繼續查。

### 14-18. 重調融合層參數 + 上線前驗證(2026-07-25 凌晨)

Frank 要求「做 #1(重調融合層),但上線測試前要先驗證清楚」。兩件事
都做:重調參數 + 系統性驗證,不只是調完就交差。

**驗證 1:回歸測試**——把新加的 `gyro_predict_track` 開關設回 0,重跑
整段飛行,跟修改前的舊 binary 輸出逐行 diff:**完全逐位元組相同**
(5406 列全部一致)。確認這個修改在關閉時是完全惰性的,不會意外影響
任何其他既有設定或 binary 行為。

**驗證 2:機制結論獨立複現**——不只信任 §14-17 fork 的報告,自己重跑
`turn_damage_analysis.py` 驗證:峰值角速度相關性 0.371→0.028、持續
時間相關性 0.823→0.607,跟 fork 報告的 0.37→0.03、0.82→0.61 幾乎
完全一致(小數點誤差來自浮點/隨機種子,不影響結論)。

**重調參數(這才是 #1 本身)**:§14-17 發現套用舊參數(pull=0.6,
DBSCAN eps=300)在新的(乾淨很多的)訊號上打平,懷疑是參數該調而不是
機制本身失效。系統性掃描驗證這個懷疑:eps 150→800、pull 0.5→0.8,
每組 10-12 個隨機種子。結果:**eps=200 是新的甜蜜點**——假陽性拒絕率
57%(舊 eps=300 只有 40-45%),依然零好錨點誤殺。12-seed 驗證:
mean rmse **77.9m**,不只贏舊訊號的最佳值(81.2m),也贏「新訊號套舊
參數」的打平結果(87.2m)——證明 §14-17 的「打平」判讀是對的:是
下游沒跟上,不是修正本身沒用。

**最終結果(全程 170-472s)**:

| | 沒開 gyro-predict | 開了 gyro-predict + 重調融合層 |
|---|---|---|
| 原始 VIO | rmse 7703m / max 20334m | rmse 2722m / max 4784m |
| 乾淨錨點 scalefix | rmse 35.3m / max 78m | rmse 35.6m / max 73m(打平,但 200-230s 轉彎窗口 25.8→**14.1m**) |
| 雜訊錨點 FoundLoc-style(重調後,12-seed 驗證) | rmse 93.8m(單一 seed)/ mean 81.2m | **rmse 78.6m(代表性 seed)/ mean 77.9m** / max **164m**(原本 285-519m) |

**判讀:這是今晚整條線索(§14-11→18)最終、經過驗證的成果**——比
今晚任何時刻的數字都好,而且是唯一同時做過「關閉時零副作用回歸測試」
+「機制結論獨立複現」+「12-seed 穩健性驗證」的版本。仍未解決:下降
段獨立發散(所有版本都有)、殘餘 duration 相關性(r=0.61,§14-17 已
指出)。**這兩項是任何線上部署前應該先解決或至少明確接受的已知限制,
不在這次驗證範圍內。**

**上線部署前的額外提醒(這次驗證沒有涵蓋、線上測試前需要另外處理)**:
本次驗證全部基於離線 offline eval(錄好的 survey25 資料),不是即時
系統。線上部署前至少還要驗證:(1) 即時運算預算——離線這裡以
~35fps 跑,線上 pipeline 的即時頻率、CPU/GPU 分時是否撐得住多一個
陀螺積分+特徵點 warp 的開銷;(2) 模擬的雜訊錨點 proxy 終究不是真實
AnyLoc 輸出,真實 domain gap 下的假陽性率/分布可能跟這次模擬的假設
(20% 假陽性、200-900m 誤差)不同,DBSCAN eps=200 是否還是甜蜜點需要
用真實 AnyLoc 資料重新驗證;(3) 這整條 pipeline 目前只在 survey25
一趟飛行上驗證過,還沒有跨飛行/跨場地的穩健性證據。

**檔案(均未 commit)**:`foundloc_corrector.py`(`--dbscan-eps` 預設
300→200,附上重調理由註解)、`vio_full_gyropredict_foundloc_final.csv`
(最終版本)。互動比較圖新增第 8 條軌跡(gyro-predict + 重調融合層):
https://claude.ai/code/artifact/9c8e8321-101f-4ed6-9988-ae5dc10c769a

### 14-19. 真的接上 AnyLoc(不是模擬)——誠實結果:離 FoundLoc 還很遠(2026-07-25)

Frank 要求「做到 FoundLoc 那麼小的誤差,並驗證」。今晚所有融合測試
(§14-14~18)都用**模擬**的 AnyLoc 錨點(機率性假陽性注入),從沒接過
真的 AnyLoc。這節直接接上真的,誠實驗證結果。

**先查到兩個具體、可修的落差**(對照 AnyLoc 論文原文 arXiv:2308.00688):
1. `anyloc/database_test1_vits14`(覆蓋 survey25 實際飛行區域)從沒被
   重建到本專案在別的場地驗證過有效的高解析度——那個修正(median
   210→80m)只套用到不同地點的資料庫,survey25 自己的場地從未受益。
2. `anyloc/localizer.py` 用 DINOv2 預設最後一層 patch token,不是
   論文自己消融實驗證實必要的「中間層 value facet」(ViT-G/14 layer
   31 value,對抗畫面內干擾物的關鍵)。

**實測結果(真實 AnyLoc,對 survey25 實際飛行畫面,對 GPS 量測誤差)**:

| | DB 解析度 | 特徵 | mean | median |
|---|---|---|---|---|
| 現況 | 0.54 m/px | 預設 | 620m | 629m |
| +重建 DB | 0.135 m/px(4x) | 預設 | 286m | 262m |
| +value facet | 0.135 m/px | layer-10 value | 274m | 286m |

**DB 重建是真的、大幅的進步**(620→286m,腰斬),而且 zoom 是實測
17-22(不是憑印象)確認 20 是真正上限(21 以上回傳空白 tile)。
**value facet 修正沒有幫助**——ViT-G/14(論文原始配置)因權重下載
實測要 30+ 分鐘被判不可行,改在實際部署的 vits14 上對 5 個層做
mini-ablation,結果**現有的預設最後一層特徵贏過每一個 value facet
層**——這個修正方向對這個場地/模型不成立,是誠實的負面結果,不是
沒做。

**接進融合層,端到端結果**:全程 170-472s rmse **231m**——比今晚
模擬錨點的最佳結果(77.9m)還差,離 FoundLoc 的 16.4m 更遠。**驗證
兩層**:重構後的 corrector 精確複現信任基準(78.57020375864622,
精確到小數點後 14 位);真實錨點 pipeline 重跑兩次結果逐位元組相同
(confirmed deterministic)。

**判讀**:真實 AnyLoc 在這份資料上的誤差分布,不是模擬假設的「大多
數時候準、偶爾抽到 200-900m 的爛匹配」,而是**持續性地普遍不夠準**
(即使兩個修正都做了)——DBSCAN 過濾機制需要「一群可信賴的錨點」才
能發揮作用,現實是這裡沒有那樣一群東西可以鎖定。域差距(本專案真實
空拍畫面 vs. 自己的衛星正射影像資料庫)在確認過的解析度上限
(0.135m/px)下仍是主導誤差來源,離 FoundLoc 的 0.1m/px Google Maps
來源、以及論文本身較溫和的測試航線,還有明顯差距。

**確認沒破壞任何東西**:`anyloc/localizer.py` diff 是純新增 88 行
(0 刪除)——`ros2_node.py` 的 `AnyLocLocalizer(DB_PATH)` 呼叫完全
沒動,照樣能 import/執行。

**檔案(均未 commit)**:`anyloc/localizer.py`(純新增 value-facet
選項)、`anyloc/build_database_value_facet.py`、
`anyloc/test_accuracy_survey25_time.py`、新資料庫
`anyloc/database_survey25_z20_vits14/` +
`database_survey25_z20_vf_L{4,6,8,10,11}_vits14/`、
`field_data/survey25/vio_eval/foundloc_corrector.py`(重構:CLI 移到
`if __name__=="__main__"`,核心迴圈拆成可 import 的 `run_corrector()`
/`simulate_anchor_stream()`,驗證過 CLI 行為逐位元組不變)、
`field_data/survey25/vio_eval/real_anchor_eval.py` +
`real_anchor_eval_result.json` + `anyloc_real_*.json` +
`vio_full_real_anchor_fix1plus2.csv`。

**下一步**:域差距是現在唯一還沒解的主要瓶頸——可能方向是換更新鮮
/更高解析度的衛星影像來源(不只是拉高現有來源的 zoom,現有來源本身
可能就是天花板)、或重新檢視 AnyLoc 論文裡「domain-specific vocabulary
比 map-specific 再好 7-19%」這個還沒套用的槓桿(§14-16 的技術討論
提過但沒實測)。

### 14-20. 對照實驗實證:問題確實出在資料庫,不是 AnyLoc 本身(2026-07-25)

Frank 要求直接做對照組驗證:用 survey25 自己的空拍影像建資料庫,再用
survey25 自己的影像去查——同域(drone 對 drone),排除衛星域差距這個
變因,看誤差是否真的大幅下降。

**做法**(全部複用既有工具,沒有新寫底層邏輯):`tools/extract_frames.py`
在巡航段(AGL≥50m)每 6m 抓一張、共 156 張,依飛行順序**交錯**分成
DB(偶數index,78 張)/ query(奇數index,78 張)——query 跟最近的
DB 條目在空間上相距約 6-7m(不是同一張),避免循環論證。用
`anyloc/build_database_real.py --model vits14`(跟今晚基準同一套
DINOv2 設定,預設最後一層特徵,不是 §14-19 已證實沒用的 value facet)
建庫。**非循環性驗證兩層**:檔名 0 重疊、MD5 內容雜湊 0 重疊(獨立
複算過一次,不只信任 agent 報告)。

**結果**:

| 測試 | 資料庫來源 | n | mean error |
|---|---|---|---|
| 基準 | 衛星 tile,0.54 m/px | 152 | 620m |
| DB 重建後 | 衛星 tile,0.135 m/px(zoom-20 上限) | 182 | 286m |
| **同域對照(drone vs drone)** | survey25 自己的空拍影像 | 78 | **8.6m** |

同域誤差比 zoom-20 衛星結果低 **~33 倍**,比原始基準低 **~72 倍**——
不是邊際改善,是決定性的。

**最差案例檢查**(不只信平均數):兩個離群值(32m、118m)都發生在一個
~180° 大轉彎,回程航線跟去程在同一片農地/水溝上方以 15-120m 間距
近乎平行通過——實際目視比對配對到的影像,確認是同一條溝渠、同一塊
田、同一條產業道路,只是 survey25 自己航線上不同時間點經過的近乎
重複場景,不是配對邏輯壞掉。這兩個誤配對的相似度分數也是全場最低
(0.34、0.23,一般匹配是 0.55-0.80)——AnyLoc 自己的信心分數正確標記
了這兩個弱匹配。是地形自相似 aliasing,不是 pipeline bug。

**判讀:域差距(真實空拍畫面 vs. 衛星正射影像)被實測證實是主導誤差
來源,不是猜測。** VLAD/DINOv2 pipeline 本身沒問題;今晚建的整條
融合鏈(corrector/DBSCAN 過濾/陀螺預測追蹤)全都沒問題——瓶頸從頭到
尾都是衛星影像來源的域差距,不是這個專案自己的程式碼。下一步方向
更明確了:换更新鮮/更貼近真實拍攝條件的衛星影像來源,是唯一值得
繼續投入的槓桿。

**檔案(均未 commit)**:`field_data/survey25/frames/` +
`frames.csv`(156 張新抽取影像)、
`field_data/survey25/vio_eval/samedomain/`(`db_session/frames.csv`、
`query_frames.csv`、`db_frames_reference.csv`、`eval_samedomain.py`、
`anyloc_samedomain_result.json`)、
`anyloc/database_survey25_samedomain_vits14/`(同域資料庫,78 條)。

### 14-21. AGL 限定重測(結果無差異)+ 修好互動圖尾端漂移的真正原因(2026-07-25)

Frank 兩個後續要求:(1)「全部測試只用 100m AGL」——因為 §14-20 的同域
資料庫混了爬升段(50-100m AGL),擔心跟衛星資料庫的 footprint 假設
(預設 65m AGL,跟 survey25 實際 100m 巡航不符)一樣是個被忽略的
confound;(2)發現互動圖播放到尾端時同域(green)那條路徑會飄走,
要求查出原因並修。

**(1)AGL 限定重測——乾淨的 null result**:`tools/extract_frames.py`
加 `--max-agl`(純新增,預設 1000 不影響既有行為)、衛星 DB 重建改用
`--agl-min 100 --agl-max 100`(原本預設 65)。四個數字重測:衛星
286.5→291.8m、同域 8.60→8.97m、融合軌跡 <1m 差異——**全部在雜訊
範圍內,沒有實質差別**。原因:survey25 爬升到巡航的過渡很短,原本
「混合 50-100m」的抽取集本來就已經 >95% 是巡航幀,AGL 限定沒有真的
改變取樣內容。域差距結論不變。

**(2)互動圖尾端漂移——不是資料庫覆蓋不足,是空間取樣邏輯的盲點**:
查 telemetry 直接發現:survey25 在 t≈309-315s **飛機其實停止水平
移動**(lat/lon 幾乎凍結),是下降前的定點懸停,不是還在掃描新地面。
`extract_frames.py` 只在飛機移動滿 `--min-dist`(6m)才存新影像——
飛機不動,就不會有新的 DB/query 影像,不是任何方法或資料庫的限制,
單純是「純距離觸發」的取樣邏輯在飛機靜止時失效。

**修法**:加 `--max-time-gap`(秒,純新增,預設極大值不影響既有行為)
——超過這個秒數沒存過新影像,即使距離沒到也強制存一張,確保懸停期
依然有 query 覆蓋。重抽:170 張(原本 156),涵蓋 161.7-317.9s(原本
到 306.1s 就斷)。同域檢索精度不受影響(7.5m mean, n=85,幾乎等於原本
8.6m——懸停時的影像本來就是靜態場景,容易匹配)。融合結果:t=310/
314/318s 誤差從 157/207/260m 降到 **13/15/15m**——尾端漂移修好了,
一樣先用 bit-exact 信任基準複現驗證過才採信新結果。

**互動圖已更新並重新發布**(同一個連結):修正 samedomain 軌跡的
1Hz 內插欄位、METRICS 表格數字(48.8/35.7/38.1m,取代舊的 64.3/
47.4/81.3m)、覆蓋範圍標示(164-318s,取代舊的 177-306s)、判讀文字
新增此輪說明。發布前驗證:19 個不重複欄位、0 筆長度不符的資料列、
`node --check` 語法通過、DOM stub 下實際執行不拋錯——沿用今晚全程
的驗證流程。

**檔案(均未 commit)**:`tools/extract_frames.py`(新增
`--max-agl`/`--max-time-gap`,均為向下相容的純新增)、
`field_data/survey25_100m_agl/`、`field_data/survey25_100m_agl_ext/`、
`field_data/survey25/vio_eval/samedomain_100m/`、
`.../samedomain_100m_ext/`(含 `vio_full_samedomain_100m_ext_fused.csv`
最終修正後軌跡)、`anyloc/database_survey25_z20_agl100_vits14/`、
`anyloc/database_survey25_samedomain_100m_vits14/`、
`anyloc/database_survey25_samedomain_100m_ext_vits14/`。

### 14-22. SITL AUTO 模式全流程測試:真的把完整 pipeline 接上、飛 survey25 航線、看 FC 收到的 EKF 會不會暴衝(2026-07-25)

Frank 要求:「用 SITL 測試完整 pipeline,AUTO 模式飛 survey25 同樣的路線,
看送給 FC 的 EKF 位置夠不夠平滑、不會亂跳」。這是把整晚的 VIO+AnyLoc+
融合成果第一次真正接上一個會執行**真實 AUTO 任務**(不是外部腳本推
setpoint)的 ArduPilot SITL,直接驗證這整條 pipeline 對 FC 到底友不友善。

**建置**(委派給 fork agent,~35 分鐘,過程中兩度提前收工被叫回去把整個
任務做完才算數):

- 任務規劃:從 `telemetry.csv` 的真實航跡用 Douglas-Peucker 簡化出 16 個
  巡航航點 + TAKEOFF + LAND,寫成 MAVLink `MISSION_ITEM_INT` 上傳——
  **此專案原本完全沒有任務上傳的程式碼**,這次從零實作。
- 新測試腳本 `control/test_full_pipeline_sitl.py`(仿 survey13 既有的
  `test_vpe_slew_sitl.py` 手法,但把「外部推 GUIDED setpoint」換成「真的
  上傳任務、切 AUTO,讓 ArduPilot 自己飛」):20Hz 送
  `VISION_POSITION_ESTIMATE` = SITL 自己的模擬真值 + **真實 pipeline**
  的誤差訊號(取自 `vio_full_real_anchor_fix1plus2.csv`,真 AnyLoc、
  全程 231m rmse——不是理想化模擬錨點版本,先重算過驗證 rmse≈230.2m
  才採信)。
- 三輪測試:`zero`(零誤差對照組,驗證測試架構本身沒問題)、
  `raw`(真實 pipeline 原始誤差)、`slew`(套 `vpe_slew.py`)。

**結果**(全部由我直接重算 JSON log 獨立驗證,數字與 agent 回報逐位元組
相符):

| 測試 | EKF 每 tick 位移 mean/max | 超過 GLITCH_RAD(50m)次數 | 飛行總路徑長 | 任務完成? |
|---|---|---|---|---|
| zero(對照組) | 0.75 / 2.08 m | 0 | 1053 m(直線約 1000m) | 是,乾淨 |
| raw(真實未濾波) | 1.83 / **341.8 m** | **4 次**(單一 tick 內跳 260-342m) | 3628 m | 是(180s) |
| slew(限速) | 0.83 / 5.08 m | 0 | **11498 m**(11 倍) | **否**(逾時仍未落地) |

**判讀**:`zero` 證明測試架構本身沒問題(乾淨完成整個任務)。`raw` 真實
重現了整晚一直擔心的暴衝機制——4 次單一 20Hz tick 內 EKF 跳 260-342m,
是真正會觸發 `EK3_GLITCH_RAD` 的事件(直接檢查座標值確認不是記錄
artifact),但因為 ArduPilot 自己的 glitch-radius 重置吸收了每一次,
飛機還是把任務飛完了。`slew` 表面上完全消除了跳動(0 次超標、per-tick
位移幾乎跟乾淨對照組一樣小)——但只看這個數字會誤導:飛機總共繞飛了
11.5 公里(直線距離約 1 公里的 11 倍),追著一個持續偏差、從未收斂的
估計值,最後測試逾時時人還沒落地。**結論:slew limiter 解決的是「急性」
危害(瞬間暴衝),不是根本的精度問題**——它把一個劇烈失控的失敗模式,
換成一個緩慢但持續的失敗模式,兩者都不是能真的飛完真實 pipeline 任務
的狀態。

**檔案(均未 commit)**:`control/test_full_pipeline_sitl.py`(新檔,含
自製 mission-upload 實作)、
`field_data/survey25/vio_eval/sitl/full_pipeline_sitl_{zero,raw,slew}.json`
+ `full_pipeline_sitl.png`。

### 14-23. DB 覆蓋範圍假說 + 照抄 AnyLoc 自己的衛星圖方法 → 意外挖出一個方向 sign bug + 系統性驗證「解析度不夠」假說不成立(2026-07-25)

**(1)覆蓋範圍假說,先老實說測不出來**:Frank 看完 §14-22 提出假說——
「問題是資料庫涵蓋範圍不夠大,飛機飄出航線就回不來」。查證:實際使用
的衛星 DB(`database_survey25_z20_vits14`,117 筆)只覆蓋約 400×600m
的方框;`raw`/`slew` 兩輪偏航確實飄到方框外(`raw` 到 450m、`slew` 到
1200m)。**但這測不出因果**——§14-22 的誤差是「照抄某個固定時間點的
真實誤差曲線」重播,不是即時對飛機當下位置重新查詢資料庫,所以架構上
就無法回答「真的飄出去後 AnyLoc 會不會自己修回來」。要真的驗證需要
(a)擴大 DB 涵蓋範圍 (b)做一個真正即時、依飛機當下位置查詢的 SITL
閉環——這兩者都還沒做,列為未完成的後續方向。

**(2)照抄 AnyLoc 論文自己的衛星圖方法**:直接讀 arXiv:2308.00688
原文(PDF 逐頁讀,非憑印象)。找到 Nardo-Air 資料集的確切做法(附錄
A3):「reference database comprises 102 images obtained from **a Google
Maps TIF satellite image**, while the query set contains 71 drone-collected
imagery」——跟這個專案的架構完全一樣(衛星 DB vs 真實空拍 query,
跨域比對)。關鍵細節:**Nardo-Air-R**(將空拍圖旋轉對齊衛星圖方向)
比未旋轉版本 Recall@1 從 76.1% 提升到 94.4%(Table IV)——這個旋轉
對齊步驟,這個專案的即時查詢/評估路徑從來沒用過。

**(3)意外挖出一個真的方向 sign bug**:把旋轉接上 `test_accuracy_survey25_time.py`
(新增 `--rotate`,沿用 `tools/extract_frames.py` 既有的旋轉公式)
第一次跑,結果反而變差(286.5m→298.6m mean)——跟論文的方向相反,
是個異常訊號。查證:羅盤 heading 是「順時針從北」,但 `cv2.getRotationMatrix2D`
的正角度是「逆時針」旋轉圖片——正確公式應該是 `angle = -heading`,
不是 `+heading`。理論推導 + 實測雙重確認:`-heading` 把 286.5m
mean 降到 **228.6m**(mean/median/rmse/min 全部一致變好,約 20%)。
**`tools/extract_frames.py` 既有的 `--rotate`(Option B「建議」的真實
空拍 DB 建置流程在用)一直是同一個 sign bug**——因為從未真正跑過
「即時查詢準確度」這種端到端驗證,所以一直沒被抓到,兩處都已修正
(已確認同域 8.6m 那組結果沒用過 `--rotate`,不受影響)。

**接上完整驗證過的融合層**,結果是真的但不是全贏:

| 時間窗 | 未旋轉(先前最佳) | 修正 sign 後 |
|---|---|---|
| 200-230s | 258.8m | **184.9m** |
| 200-260s | 277.3m | **208.2m** |
| 200-320s | 279.8m | **176.8m** |
| 170-472s(全程) | **231.4m** | 249.8m |

中段三個窗口都進步 30-40%,但全程窗口反而略差(很可能跟一直沒解決的
下降段行為有關,不是旋轉修正本身的問題)——誠實回報為「有意義但混合」
的結果,不是乾淨勝利,更不推翻域差距是主因的結論。

**(4)「我們的衛星圖夠清楚嗎、跟他們一樣嗎?」——先目視比對,再系統化驗證**:
目視抓真實比對組:一個好配對(誤差 42m)兩邊清晰度相當,跟論文 Fig.2
的空拍範例觀感一致;一個壞配對(誤差 378m)兩邊其實都是**沒有特徵的
碎石地**,不是清晰度問題;另外一組隨機(非真配對)比較則顯示我們的
zoom-20 衛星圖在**樹冠密集區**確實明顯模糊。三個範例互相矛盾,所以
Frank 要求系統化驗證。

**系統化驗證(Laplacian variance 清晰度指標,對全部 117 張 DB 圖 vs
182 筆真實查詢誤差做相關)**:matched-tile 清晰度 vs 誤差 r=-0.14
(log-清晰度 r=-0.34);true-location-tile 清晰度 vs 誤差 r=-0.23。
依 true-location 清晰度三分位分組,誤差**不是單調的**(最模糊組
252m、中間組 **205.9m 最低**、最清晰組 227.7m)。**結論:衛星圖清晰度
不是誤差的主要驅動因子**——先前目視抓到的例子只是巧合,不是通則。
系統驗證反而進一步鞏固了「domain gap(季節/光照/內容不匹配)才是
主因,不是解析度」這個已經確立的結論。

**判讀**:這一輪淨結果是——(a)覆蓋範圍假說有部分證據支持但架構上
測不出因果,列為未完成;(b)照抄 AnyLoc 官方旋轉對齊手法確實有效
(20%),還意外修好一個沉睡的方向 bug,但不是决定性的;(c)「解析度
不夠」被系統化驗證推翻。整體上域差距(不是覆蓋範圍、不是解析度)
依然是唯一有強力證據支持的主要瓶頸。

**檔案(均未 commit)**:`anyloc/test_accuracy_survey25_time.py`(新增
`--rotate`)、`tools/extract_frames.py`(修正 `--rotate` 的 sign bug)、
`field_data/survey25/vio_eval/real_anchor_rotated_eval.py`、
`anyloc_real_fix1_rotated_full.json`、`vio_full_real_anchor_rotated.csv`、
`real_anchor_rotated_eval_result.json`、`sat_clarity_vs_error.py`、
`sat_clarity_vs_error_result.json`、`sat_clarity_vs_error.png`。

### 14-24. 真正即時、依位置查詢的 SITL 閉環 + postview 視覺化 + 兩個真 bug(方向 bootstrap、SPEEDUP 配速)+ 最終結論:偏離航線後不會自我修正(2026-07-25/26)

Frank 指出 §14-22 的 SITL 測試方法論有根本缺陷(「isn't the vio material
all in survey 25」的追問也證實了這點)——原本注入的誤差是「照抄某個
固定時間點的真實誤差」重播,不是即時依飛機當下位置重新查詢,所以
根本無法回答「偏離航線後 AnyLoc 會不會自我修正」。要求:改用 survey25
的真實影格(依飛機當下位置去找對應的真實影格,不是整支影片照放)、
即時跑真的 AnyLoc、並且要看得到 postview(路徑圖 + 當下用的 survey25
真實影格 + AnyLoc 配對到的資料庫圖 + AnyLoc 誤差與整體定位誤差)。

**建置(委派 fork agent)**:新檔 `control/test_full_pipeline_sitl_live.py`
——每 2 秒(對齊 `foundloc_corrector.py` 真實 anchor 週期)取 SITL 當下
真實位置,依「最近真實座標」在 `telemetry.csv`/`frame_times.csv` 全程
搜尋最近的 survey25 真實影格,從 `video.mkv` 解碼、套用 §14-23 修正後
的旋轉(`-heading`),餵給真的 `AnyLocLocalizer`(`database_survey25_z20_vits14`)
即時推論,再把估計值餵進一個為逐 tick 使用重寫的
`foundloc_corrector.py` 邏輯(DBSCAN 過濾、SE(2) 鎖定、anchor pull——
沿用原邏輯,不改動已驗證的原始檔案)。Postview:每個 anchor tick 存一張
4 格合成圖(路徑圖、旋轉後的真實影格、配對到的 DB 圖、兩個誤差數字)
到 `field_data/survey25/vio_eval/sitl/postview/`,結束後組成 mp4。

**第一輪結果誤導性很大**(agent 誠實回報,未美化):anyloc_error 平均
2516m/max 5077m,localizer_error 平均 2690m/max 5218m——但 agent 自己
就先指出問題:啟動階段(SE(2) 鎖定生效前,~34.5s 內)發布的是「VIO
自己內部座標系的原始死推估計」,根本沒轉到真北,從第一個 tick 就把
FC 導向錯誤方向,整個測試變成在測「啟動階段方向沒對齊」這個 bug,
不是在測自我修正問題。

**我直接動手查證+修好**:先確認這不是新引入的 bug——比對已驗證的
離線版 `foundloc_corrector.py`,發現 `run_corrector()` 本身輸出的
`px`/`py` 也是未轉換的原始 `pc`(VIO 自己座標系),整晚所有離線
rmse 數字之所以合理,是因為評估腳本(`real_anchor_eval.py`/
`foundloc_eval.py` 的 `err_curve(align_win=20)`)另外做了一次「用
GPS 20 秒視窗擬合一次性 yaw+offset,套用到全程」的**評估專用**
校正——這個步驟從沒被搬進即時 SITL 版本。修法:新增
`LiveCorrector.bootstrap_align()`,在 route 開始的頭 20 秒(對應真實
飛行「起飛前有 GPS」的合理假設,跟離線評估的 `align_win=20` 用同一套
方法論,不是用 GPS 作弊貫穿全程)用 SITL 真值擬合一次性 yaw+offset,
之後發布 `anchor_R @ pc + anchor_off` 而非原始 `pc`。重跑驗證:
bootstrap 確實生效(yaw=-73.4°,前 20 幾秒追蹤誤差 <15m),但整體
仍然發散(anyloc 2495m/localizer 2492m)——bug 修對了,但不是唯一
問題。

**第二個真 bug,Frank 追問「isn't the vio material all in survey 25」
逼出來的**:log 顯示「VIO increment queue exhausted (all 5404 steps
consumed)」——VIO 材料確實涵蓋 survey25 全程 360 秒,不是資料不足;
問題是 SITL 用 `--speedup 4` 加速,dead-reckoning 消耗速度必須跟著
乘 4(這是更早就修過的另一個 bug,不跟則會讓死推落後真實進度而發散)
,導致 360 秒材料只夠撐 90 秒「真實牆鐘時間」的測試,而連 zero-error
對照組都要 74.5 秒——餘裕已經很緊,一旦稍微發散、測試跑更久,材料
就會提前耗盡。修法:改用 `--speedup 1`(新檔內覆寫,不動共用的
`test_full_pipeline_sitl.py`),讓 360 秒材料對應 360 秒真實牆鐘時間。

**最終重跑(bootstrap 修正 + speedup=1),Frank 全程即時看 postview**
(用 Monitor 工具每 ~30 秒抓一張新 postview 圖直接貼進對話,不是
事後才看):

| 指標 | 數值 |
|---|---|
| anyloc_error_m | 平均 2465m / max 4214m |
| localizer_error_m | 平均 2484m / max 4256m |
| EKF 每 tick 位移 | 平均 1.69m / max 517m |
| 超過 GLITCH_RAD(50m)次數 | 11 |
| 樣本數 | 346 個 anchor tick,route t=0-699s |

**VIO 材料耗盡時機從 route_t≈90s 延後到 ≈360s(4 倍改善,直接由
成長率在 t≈358-380s 附近從每 tick ~10-20m 驟降到 ~1-2m 證實)。但
發散早在 t≈93s(誤差已 >1000m)就開始了,遠早於材料耗盡——證明
發散不是材料不足的產物,是閉環本身的性質**:真實 AnyLoc 單次抓取
本來就有 ~230-290m 的雜訊(整晚已確立的數字),一旦真值偏離錄製航線
幾百公尺,最近真實影格搜尋開始撈到離很遠的影格,回傳的估計值幾乎
跟真值無關,corrector 卻仍照拉,把飛機拖得更遠,形成惡性循環。
這次的失敗形態跟 §14-22(乾淨、無界地暴衝到 7000m+)不同——是
noisy 但大致有界的螺旋,穩定在 2500-4250m 帶狀範圍,不再爬升也
不曾收斂回航線。

**最終結論**:兩輪獨立測試(speedup=4 的乾淨暴衝、speedup=1 的
noisy 有界螺旋)方向一致——**一旦偏離錄製航線,這條真實 pipeline
閉環不會自我修正**,無論是暴衝式還是緩慢螺旋式,飛機都回不到正確
位置。這是目前為止對「資料庫覆蓋不夠時能不能自救」這個問題最誠實
的直接證據(雖然仍受限於單一場地、單一資料庫、事後才知道的真值)。

**檔案(均未 commit)**:`control/test_full_pipeline_sitl_live.py`(新檔,
含 bootstrap-align 修正 + speedup=1 覆寫)、
`field_data/survey25/vio_eval/sitl/full_pipeline_sitl_live.json`、
`postview_live.mp4`、`postview/frame_*.png`(346 張)、
`prefix_bug_archive/`(前兩輪失敗嘗試的完整記錄,含修 bug 前的數據
與 postview,保留供對照)。

### 14-25. 總結問答:瓶頸是 VIO 還是 AnyLoc 資料庫?→ 下一步是實機建圖飛行(2026-07-26)

Frank 收斂式提問:「目前的瓶頸是 VIO 還是 AnyLoc 資料庫?」「所以我
需要去飛一趟建資料庫的飛行嗎?」——整理整晚證據給出的結論:

**瓶頸是資料庫(衛星圖跟真實空拍的 domain gap),不是 VIO**:
- 對照實驗(§14-20)是最直接的證據——同一套 VIO、同一套 AnyLoc/
  VLAD/DINOv2 程式碼、同一趟飛行,唯一變數是參考資料庫來源:衛星圖
  ~286m mean 誤差,同域(用空拍圖建的資料庫)~8.6m——只換參考影像
  來源就差 33 倍。
- 解析度已被系統化排除(§14-23,清晰度 vs 誤差 r=-0.14~-0.23,
  非單調)——不是「圖不夠清楚」,是「圖的內容/季節/光照跟現場對不
  上」。
- VIO 自己確實有問題(全程未修正漂移 2700-7700m、轉彎追蹤失能已修
  §14-17、尺度/震動問題已修 §14),但這是單眼(monocular)VIO 設計上
  本來就預期需要定期絕對定位修正,不是本專案獨有的缺陷;§14-24 證明當
  修正訊號夠準(同域品質)時,即時閉環可以把誤差壓在 15-50m——問題
  出在修正訊號本身不夠準,而訊號不準的根源是資料庫 domain gap。

**下一步建議:去飛一趟建資料庫的飛行**——這是唯一被證實能讓誤差
量級下降的槓桿。但要注意三個跟今晚驗證範圍的落差,不能假裝解決:
1. 今晚的「同域」測試是把**同一趟飛行**的影格切成 train/query(交錯
   分割),同一天、同一光照、同一季節——不是「先飛一趟建圖、隔天飛
   任務再定位」的真實部署情境,那個情境完全沒測過。跨飛行的季節/
   光照 domain gap 有多大是未知數(預期會比 8.6m 差,但仍應該遠比
   286m 的衛星結果好)。
2. 資料庫必須涵蓋**整個任務區域外加餘裕**,不能只沿著預定航線建一
   條線——§14-24 已證明這條 pipeline 一旦飄出資料庫涵蓋範圍就不會
   自己修回來,涵蓋線性走廊等於沒有安全邊際。
3. 建議建圖飛行後,**再飛一趟獨立的驗證飛行**,拿驗證飛行的影像去
   查詢建圖飛行做出來的資料庫,實測跨飛行的真實誤差,而不是假設它
   會跟今晚的同域數字一樣好。

沿用既有、已寫好的流程即可(不必重新發明):`anyloc/README.md`
Option B + `instructions/field_database_collection.md`——
`record_field.py` → `extract_frames.py --rotate`(§14-23 已修正
sign)→ `build_database_real.py`。

**後續(同晚):實際產出建圖飛行的航點檔**。Frank 決定先在 survey25
現場(不是正式比賽場地——兩者相距約 80km,已先確認過)做第一趟驗證,
範圍縮小到只涵蓋起飛點附近(不是整條航線),飛行參數 30 分鐘、
10 m/s。`tools/gen_survey_waypoints.py`(原本只服務寫死在程式碼裡的
比賽場地座標)加了一組純新增 CLI 參數(`--center-lat/--center-lon/
--width-m/--height-m/--altitude/--speed/--name`,不給就完全重現原本
比賽場地的輸出,已驗證位元組相同)讓它能對任意場地產生航點檔。最終
產出 `field_data/survey25/survey25_dbcollect_full.waypoints`——850m
×850m(margin 後 1020×1020m)、100m AGL、60m 間距(50% 側向重疊)、
10 m/s、17 條掃描線、18.3km、約 31 分鐘——已提醒 10 m/s 比腳本原本
「≤3 m/s 減少動態模糊」的預設快 3 倍以上,建議飛完先檢查影格清晰度
再決定要不要用這批資料建資料庫。

### 14-26. 建圖+驗證飛行真的飛了——跨飛行同域精度實測(2026-07-27)

Frank 2026-07-27 在 survey25 現場飛了兩趟真實飛行,相隔約 15 分鐘:
survey33(建圖,500s,對角多航線割草機式網格,~206×342m 涵蓋範圍)、
survey32(驗證,326s,航線完全落在 survey33 涵蓋範圍內)。沿用既有流程
(§14-25 結尾提到的 Option B,無新工具):`extract_frames.py --rotate
--min-dist 15 --min-agl 90` 抽出 112 張影格 → `build_database_real.py`
建出 `anyloc/database_survey33_vits14` → 用 survey32 自己的影格查詢。

**結果**:跨飛行同域誤差 mean 24.1m(median 17.4m,n=51,t=110-210s)/
巡航窗收緊到 116-197.2s 後 mean 25.1m(n=41)——符合 §14-25 的預測:
比同一趟飛行切分的 8.6m 差(真實跨飛行退化,預期中),但仍比衛星
資料庫的 286m 好 ~12 倍。兩個 ~150m 的離群值(t≈186-188s)查過航向
——不是尖銳轉彎造成(航向在該窗口穩定在 104-106°,與 §14-16/23 的
轉彎病灶特徵不同),機制未解但對整體結果影響小。完整數據:
`field_data/survey32/vio_eval/README.md`。

### 14-27. OpenVINS 執行檔過期地雷:發現 + 根因 + 修復(2026-07-27)

承接上一節要跑真實 VIO+融合閉環測試時,`run_video_msckf` 對**任何**
輸入都 100% crash(glibc heap corruption assert,訊息隨輸入不同)——
連重跑先前跑過的 survey25 指令都 crash,證明是工具鏈本身退化,不是
survey32 資料的問題。根因:執行檔最後連結於 2026-07-22,但
`libov_msckf_lib.so`(執行檔動態載入的 library)在 2026-07-24 23:54
因 gyro-predicted-tracking 改了 class layout 而重建(§14-17 提過的
那次重建),執行檔卻沒有跟著重連結——`make_shared<VioManager>(...)`
在編譯時把舊版(較小)的 `sizeof(VioManager)` 烤進配置大小,但實際
建構子來自新版(較大)的 class,配置空間不夠,第一次深層記憶體配置
就把 heap 弄壞。光重連結不夠(試過仍 crash),`.o` 必須用當前 header
重新編譯;因為這個 shell 對 ROS2/`ament_cmake` 過敏,`cmake`/`make`
一碰就想重新 configure 並失敗,所以改成直接從 CMake 自己快取的
`flags.make`/`link.txt` 複製編譯與連結指令手動執行,完全繞開
`cmake_check_build_system`。修復後驗證(survey25 t=100-300s):2821
筆、0 NaN、23.1fps、`cam_dt` 收斂到 Kalibr 標定值附近——確認修好。
`run_video_msckf_gyrogate`/`_gyrogate_soft` 兩個執行檔仍是舊版、
**未修**,下次要用先照同樣方法重編譯+重連結。完整記錄:
`instructions/vio_data_collection.md` §5、memory
`openvins-first-offline-run`。

### 14-28. 真實 AUTO 模式 SITL 閉環測試 survey32——第一個乾淨、零暴衝的結果(2026-07-27)

工具鏈修好後,把 §14-22/24 那套「用真實飛行航跡建 AUTO 任務、真實
VIO+AnyLoc+融合層即時驅動 `VISION_POSITION_ESTIMATE`」的閉環測試
搬到 survey32(`control/test_full_pipeline_sitl_survey32.py` +
`_live_survey32.py`,重新指向 survey32 的資料 + `database_survey33_vits14`
+ survey32 自己真實的 OpenVINS 軌跡,邏輯完全沿用 survey25 版本已驗證
過的實作)。任務從 survey32 巡航窗(t=116.0-197.2s)產生 6 個航點,
真實飛完:上傳任務→爬升→切 AUTO→飛完全部航點。

**結果(有效任務窗 t=0-92.2s)**:anyloc_error_m mean=32.6m/max=191.2m
(n=46)、localizer_error_m mean=27.1m/max=85.0m、EKF 每 tick 步幅
mean=0.78m/max=12.91m、**glitch(>50m)事件 = 0**——這是本專案第一次
產出乾淨、零暴衝的即時閉環結果(先前每一輪 survey25 的即時閉環測試
都有數個 glitch 事件、誤差達千米級)。直接對應 §14-24 的發現(一旦
飄出資料庫涵蓋範圍就不會自我修正)的正面案例:survey32 航線全程都在
survey33 資料庫涵蓋範圍內,SITL 真實位置全程落在 home 附近 ±44-145m
(x)/-54-75m(y),遠在涵蓋範圍內——留在涵蓋範圍內,閉環是乾淨的。

當時任務仍以 `MAV_CMD_NAV_LAND` 結尾,飛到 LAND 後**沒有自動解鎖**,
卡在原地 armed 狀態約 607 秒,直到一個從 survey25 抄來、沒針對
survey32 航線長度改過的寫死 700 秒逾時把它切斷——這拉低了腳本自己
印出的原始總表統計(混入 ~600 秒卡住不動的資料),上面的數字是手動
限定在真正有效窗口(t=0-92.2s)算出來的。VIO 增量佇列(252s 素材)
在 route_t≈250-390s 某處耗盡,與有效窗口無關。完整記錄:
`field_data/survey32/vio_eval/README.md` 第 7 節。

### 14-29. 拿掉 LAND、改用 LOITER——因為本專案是人工起降,pipeline 只管巡航(2026-07-27)

Frank 直接澄清:「我的計畫只需要巡航,起降是我自己手動飛」——這解掉
上一節的 LAND 卡死問題(對真實部署而言根本不是問題,因為真實飛行
從來不會用自動 LAND),也讓任務設計更貼近實際:人工爬升到巡航高度
→ 交給 pipeline 飛巡航航段 → 人工降落,起降兩端都有 GPS 可用。已把
這條記錄進 memory `project-overview`(取代原本暗示自動起降的敘述)。
連帶修正:`mission_items_from_waypoints()` 最後一項改成
`MAV_CMD_NAV_LOITER_UNLIM`(定高盤旋在最後航點,不降落),任務完成
判定從「等待解鎖」改成「MISSION_CURRENT.seq 到達最後一項」,同時把
即時版腳本裡從 survey25 複製過來、沒跟著改的寫死 700 秒逾時
(`(315.0-170.0)/...`)換成正確對應 survey32 航線長度的
`CRUISE_SPAN_S` 常數。修復後重跑驗證:自然在 ~93 秒內結束(不再卡
607 秒),腳本自己印出的原始總表現在可以直接信任、不需要再手動限定
窗口:anyloc_error_m mean=32.8m/max=191.2m、localizer_error_m
mean=24.0m/max=78.0m、EKF 步幅 mean=0.81m/max=11.84m、**glitch 事件
仍是 0**——與手動限定窗口算出的原始數字一致,交叉確認先前的過濾
是對的。

### 14-30. `--stride 1`(全 30fps)實測結果:不是免費的改善(2026-07-27)

Frank 問「VIO 有沒有用全影格率」——查出用的是 `stride=2`(~15Hz 餵料,
本專案一直以來的標準),從沒真的 A/B 測過。實測 survey32 全程改
`stride=1`:**raw VIO 明顯變差**(巡航窗 rmse 75.8→258.4m,3.4 倍,
max 121.8→625.0m)——反直覺,推測是相鄰影格 baseline(視差)減半
傷到三角量測品質,抵銷了影格間位移變小帶來的追蹤穩定性好處,是單眼
VIO 常見取捨,不是 bug。融合修正後的最終結果反而 stride=1 略好
(巡航窗 rmse 69.2→55.6m)——但這只是單一飛行、單一資料點,不構成
「應該全面換成全 30fps」的結論,兩個方向的訊號互相矛盾。**維持
stride=2 為專案預設**,除非之後累積更多資料支持換掉它。完整數字:
`instructions/vio_data_collection.md` §5。

### 14-31. 融合層真實發散問題:診斷 + 修正(2026-07-27)

檢視 survey32 巡航窗的連續 postview 重播影片時,發現修正層自報的誤差
與其對 GPS 的實際誤差之間,在 t≈154s 處出現 27.7 倍的落差(自報約
5.8m,實際卻是 160.0m),且發布出來的位置也明顯不平滑,單一 tick
之間曾出現高達 98m 的跳動。這與 §14-28/29 report 出的「乾淨結果」
表面上矛盾——glitch 閘門(單 tick >50m)沒被觸發,不代表輸出本身
準確或平滑,只代表沒有大到觸發閘門的單步跳動。

實際以資料檢驗了兩個假設,而非單憑猜測:

1. **航向/平移對齊失準**——改用週期性重新對齊
   (`run_corrector_relock`,測試多種 relock 弧長/窗口組合)後,
   t=154s 的誤差峰值在每一種變體下都維持在相同的 160.0m,排除了
   旋轉漂移是成因。
2. **拖尾窗口的尺度估計**——修正層原本用 `anchor_win=40s` 的拖尾窗口
   估計 VIO 對錨點的尺度比例。改成 `anchor_win=20s`(更短、更即時的
   窗口)後,巡航 RMSE 從 69.2m 降到 36.5m,t≈154s 的誤差峰值降到
   不到原本的一半。

平滑度檢查(每個時間步的最大單步位移量)顯示縮短 `anchor_win` 後輸出
仍不平滑:最大單步 65.4m,超過 5m 的步數共 30 次。套用本專案原本就
有的速率限制器(`control/vpe_slew.py`,§ 更早已為即時 EKF 暴衝防護
而建置)作為輸出最後一道處理,幾乎完全消除了這個問題:最大單步降到
1.17m,零步超過 5m,巡航 RMSE 也進一步改善到 **27.3m**(全程 RMSE
573.8m)。`anchor_win=20` + 速率限制器現已成為本專案**離線/批次**
修正層的預設配置(`field_data/survey32/vio_eval/real_anchor_eval_survey32.py`
是官方評估腳本)。完整推導、每一步的中間數字、圖表:
`field_data/survey32/vio_eval/README.md` 第 10 節。

### 14-32. 同一個修正在即時閉環裡卻讓結果變差 4–5 倍(2026-07-27)

原本預期 §14-31 的離線修正也能直接套到 §14-28/29 的即時閉環 SITL
測試上——實測結果相反。把 `anchor_win=20` + 速率限制器套進
`control/test_full_pipeline_sitl_live_survey32.py` 的 `LiveCorrector`
後,完整跑了一輪 2×2 矩陣(anchor_win × 有無速率限制器,皆為真實
SITL 執行):

| 配置(即時閉環) | AnyLoc 誤差 | Localizer 誤差 | tick 數/時長 |
|---|---|---|---|
| anchor_win=40,無限制器(保留為預設) | 32.6–34.0m | 24.0–28.2m | 44–46 / ~92s |
| anchor_win=20,無限制器 | 34.4m | 30.8m | 49 / ~98s |
| anchor_win=20,+速率限制器 | 116.1m | 109.9m | 79 / ~158s |
| anchor_win=40,+速率限制器 | 131.8m | 123.1m | 75 / ~150s |

速率限制器讓結果變差 4–5 倍,與 `anchor_win` 設多少無關;`anchor_win`
本身在即時閉環裡幾乎不影響結果(20 略差於 40)。機制:離線評估時
真值(GPS)是固定的,不受發布內容影響,所以放慢修正速度只會讓誤差
曲線「看起來」更平滑。但即時閉環裡,發布出去的位置會直接驅動真實的
模擬載具(經 EKF + 位置控制器),而下一次 AnyLoc 查詢是依照「距離
載具目前真實位置最近」的真實錄影影格來選取的——速率限制器讓真實
漂移在被拉回之前累積得更多,而更多的漂移又會導致最近影格搜尋抓到
與目前位置愈來愈不相關(比對品質愈差)的查詢影格,形成一個離線評估
中完全不存在的回饋迴路(與 §14-24 已記錄過的「一旦飄出資料庫涵蓋
範圍就不會自我修正」是同一類機制,只是這次是由修正層自己的速率
限制觸發,而非資料庫涵蓋範圍不足)。

**決定**:離線/批次評估固定用 `anchor_win=20` + 速率限制器(§14-31);
`test_full_pipeline_sitl_live_survey32.py` 的 `LiveCorrector` **刻意
維持** `anchor_win=40`、**不加**速率限制器,並在程式碼裡寫清楚原因
(而不只是寫在這份文件或 memory 裡),避免未來有人不了解這個結論、
把即時路徑「修好」成離線專用的配置。四種變體的原始 log 都保留:
`sitl/full_pipeline_sitl_live_anchorwin40_noslew_FINAL.json`(保留
為預設,對應 §14-29 數字重新驗證一致)、`_anchorwin20_noslew.json`、
`_slew_anchorwin20.json`、`_slew_anchorwin40.json`。完整記錄:
`field_data/survey32/vio_eval/README.md` 第 10 節「But: the same fix
made the LIVE closed loop 4–5x WORSE」。

### 14-33. 三份論文全面更新(2026-07-28)

Frank 要求「make sure all test and everything is updated also
everything in paper, and mention again the project is for cruising,
so all test is done with the cruise, and test include slew, update
the video, all gragh」——把 §14-31/32 的完整結論(離線修正的推導、
即時閉環的反例、2×2 矩陣、split-config 決定)以及巡航限定範圍聲明,
傳播進 `~/rural_NGPS/` 底下的三份論文:`main.tex`(完整版)、
`survey32_33_report.tex`(僅 survey32/33 結果,英文)、
`survey32_33_report_zh.tex`(同內容,繁體中文)。三份論文都新增了
「Divergence Diagnosis and Fix」小節與即時閉環 2×2 矩陣表格,三欄式
融合修正層表格(raw VIO / 融合後(原始)/ 融合後(最終)),並在各自
摘要(Abstract)中明確重申「本專案僅針對巡航飛行設計與驗證,起降由
安全飛手手動操作」。同步更新的圖表(`sitl_path_vs_mission.png`、
`localizer_path_vs_gps.png`、`postview_example.png`、
`postview_offline.mp4`)皆已重新從最終配置(`anchor_win=20`+速率
限制器的離線 CSV、`anchor_win=40`無限制器的即時閉環 log)產生。三份
論文皆已重新編譯確認 0 個未定義參照。

### 14-34. 繁體論文的簡體字地雷:ctex 套件的自動標題文字(2026-07-28)

Frank 發現 `survey32_33_report_zh.tex` 編譯出的 PDF 裡,圖片標題的
「圖」字顯示成簡體「图」。追查發現:所有手寫的內文字元都已用 OpenCC
驗證過完全沒有簡體字,問題出在 `ctexart` 文件類別本身——它載入的
`ctex-name-utf8.cfg` 把 `figurename`(圖/表 的「圖」)、`bibname`/
`refname`(參考文獻)、`contentsname`(目錄)等**所有**自動產生的
標題文字都寫死成簡體中文,完全不受 `\setCJKmainfont` 之類的字型設定
影響——字型只決定字形怎麼畫,不影響巨集展開出的是哪個字。這也代表
先前「對 .tex 原始碼跑 OpenCC」的驗證方式有盲點:這些字串根本不是
原始碼裡的文字,而是文件類別在編譯時生成的,必須改成對**編譯完的
PDF**(`pdftotext` 抽出的文字)跑 OpenCC 才抓得到。

修正方式:在 `\setCJKmainfont` 等指令之後加上
`\ctexset{figurename=圖, bibname=參考文獻, tablename=表, ...}`
明確覆寫成繁體(這個版本的 ctex 不接受 `algorithmname`/`refname`/
`continuation` 這三個 key,會丟 LaTeX3 錯誤,拿掉即可,`bibname` 已
足夠修正參考文獻標題)。修正後重新編譯,對 PDF 抽出文字跑 OpenCC 確認
`图`/`参考文献`/`插图`/`附录`/`证明`/`续` 皆為 0 筆。完整記錄:
memory `ctex-simplified-caption-landmine`。任何未來在本專案底下新增
的繁體中文 `ctexart` 文件都需要套用同樣的 `\ctexset` 覆寫。

### 14-35. 高度是不是 VIO 誤差的元兇?survey30(5m)/survey31(10m)實測對照(2026-08-02)

Frank 提供兩趟新飛行——survey30(~5m AGL 巡航)、survey31(~10m AGL
巡航)——同一套機身/相機(自 Kalibr 標定後未拆裝),問「高 AGL 是不是
造成 VIO 誤差的原因」。用與 survey32/33 完全相同的方法論(同一份
`run_video_msckf` config`survey17_kalibr`、`err_curve`/`report` 的
固定 4DOF(僅航向+平移,不含尺度)對齊、在各自巡航窗前 20 秒對齊、
對整個巡航窗算 RMSE/max)離線跑出兩趟的真實 OpenVINS 軌跡,直接對比
survey32 既有的 100m AGL 結果(新程式:`field_data/vio_agl_comparison.py`,
結果:`field_data/vio_agl_comparison_result.json`、
`field_data/vio_agl_comparison.png`)。

**巡航窗**(從 AGL 曲線量出來,參數與 §14-26 系列一致):survey30
t=145.6–265.6s(~5m AGL,120s,地速僅 1.30 m/s——非常慢,接近定點)、
survey31 t=98.8–174.4s(~10m AGL,75.6s,地速 4.45 m/s,與 survey32/33
實際巡航速度相近)、survey32 t=116.0–197.2s(~100m AGL,81.2s,地速
5.73 m/s)。

**結果**(raw VIO,同一套固定對齊方法,無 AnyLoc/融合層介入):

| 飛行 | AGL | 地速 | RMSE | max | max 佔航跡長% |
|---|---|---|---|---|---|
| survey30 | ~5m | 1.30 m/s | 7.3m | 17.9m | 11.5% |
| survey31 | ~10m | 4.45 m/s | 3.0m | 5.9m | 1.8% |
| survey32 | ~100m | 5.73 m/s | 75.8m | 121.8m | 26.2% |

**結論:高 AGL 確實是這個 pipeline VIO 誤差的主要元兇之一**——100m
AGL 的 RMSE 是 10m AGL 的 25 倍、是 5m AGL 的 10 倍,即使三趟飛行的
地速、航段長度、轉彎次數(逐 sample 航向變化量測過,survey30 反而是
三趟裡轉彎次數最多的,排除「survey32 誤差大是因為轉彎多」這個替代
解釋)都不構成足以解釋這個量級差距的因素。

但更精確的機制不是單純「AGL」本身,而是單目 VIO 深度/尺度可觀測性
的經典驅動因子——**每影格相機位移(baseline)與場景深度的比值**
(對接近水平飛行的下視相機而言,深度 ≈ AGL)。用「地速/AGL」當作
視差率的粗略代理:survey31(0.445/s)最好(RMSE 3.0m)、survey30
(0.26/s)其次——雖然 AGL 比 survey31 低一半,但因為飛得極慢(近乎
定點),parallax 反而比 survey31 差,RMSE 比 survey31 差 2.4 倍;
survey32(0.057/s)最差,因為 100m 深度即使用 5.7 m/s 的地速仍遠遠
不夠彌補。這與這個專案已經記錄過的「cruise-scale collapse」現象
(§14-11,原本歸因於轉彎時 rolling shutter)是同一個更大類別問題的
另一面,也與 memory `vpe-jump-runaway` 裡提過的「FoundLoc 自己的評測
裡 altitude change 是其最差的類別(ATE 22.7m)」互相印證——這不是這個
專案獨有的怪異現象,是單目 VIO 的已知通病。

**對真實部署的意義(可執行結論)**:這個專案的巡航速度被動態模糊
安全上限鎖在 ≤3–7.5 m/s(見 `instructions/field_database_collection.md`
的速度規範),而比賽任務要求 100m AGL 巡航——用「地速/AGL」的角度看,
要達到 survey31(10m AGL)那種視差率,100m AGL 需要地速約 40+ m/s,
遠超安全上限、也遠超這台機身的合理飛行速度。**換句話說,在這個專案
實際會飛的巡航高度,raw 單目 VIO 的大誤差不是一個可以靠飛行參數調整
解決的問題——這正是為什麼 AnyLoc 視覺定位修正層是必要的,不是錦上添花
的選配**,與這份文件從 §14-25 開始建立的「瓶頸是資料庫 domain gap,
VIO 本身也有它自己獨立的高度相關瓶頸」的整體圖像一致。

**已知限制**:每個高度只有一趟飛行(n=1),不是嚴格控制單一變量的
實驗;survey30 的極慢地速是一個真實存在、未特別控制的混雜因子(見上
「地速/AGL」分析,已盡量拆解但無法完全排除);三趟飛行的資料庫/場地
條件相同,但飛行日期、光照、風況等其他變量沒有特別控制。完整程式碼
與數字:`field_data/vio_agl_comparison.py`、
`field_data/vio_agl_comparison_result.json`、
`field_data/vio_agl_comparison.png`。VIO 軌跡:
`field_data/survey30/vio_eval/vio_full_survey30.csv`、
`field_data/survey31/vio_eval/vio_full_survey31.csv`。

### 14-36. 另一種 VPR 方法:ngps_flight(SuperPoint+LightGlue)vs AnyLoc,以及全
pipeline 查詢頻率掃描(2026-08-02)

Frank 指定要試的方法是 GitHub `snktshrma/ngps_flight`——一個下視相機對georeferenced
衛星影像的視覺定位系統,核心是 SuperPoint 關鍵點 + LightGlue 匹配(不是像 AnyLoc
那樣的全域描述子最近鄰檢索),再用內點匹配求 homography、把查詢影格投影進參考影像
的像素座標,最後用參考影像本身的地理配準(仿射轉換)換算成經緯度。

**環境地雷(嚴重,已修正)**:第一次嘗試 `pip install lightglue`/`kornia` 時裝在
**系統 Python**(`/usr/bin/python3`,不是 venv),結果靜默把 `numpy` 從 1.26.4 升到
2.2.6,同時新裝的 `opencv-python` 蓋掉了系統原本有 GStreamer 支援的 cv2(相機
pipeline 靠這個)——`cv2.getBuildInformation()` 確認新裝的是「GStreamer: NO」。
在跑任何實際比對之前就先發現並回復:`pip uninstall opencv-python lightglue kornia
kornia_rs` + `pip install numpy==1.26.4`,確認系統 cv2 版本(4.5.4,來自
`/usr/lib/python3/dist-packages`,非 pip)與 GStreamer 支援都復原。之後全部改在
`python3 -m venv --system-site-packages`(繼承系統的 CUDA-linked torch,但新裝的
套件只落在 venv 自己的 site-packages,不影響系統)裡做,並且把 `kornia`/
`opencv-python` 都釘在 numpy<2 相容的版本(`kornia==0.7.0`、
`opencv-python==4.10.0.84`),避免 torch 的 numpy C API ABI 不匹配警告
(`Failed to initialize NumPy: _ARRAY_API not found`)。**教訓:這台 Jetson 的系統
Python 就是正式飛控 pipeline 在用的直譯器,任何新方法的實驗性套件一律裝進隔離
venv,不裝系統環境。**

**ngps_flight 本身這個 repo 在這台 Jetson 上直接跑不起來**(目標是 JetPack 7.2/
CUDA 13.2/ROS2 Jazzy,這台是 R36.4.7,且它的核心比對腳本需要先把 ONNX 匯出、編譯
TensorRT engine)——改成直接用官方 `cvg/LightGlue`(PyTorch 原生)重現它說明文件
裡描述的方法,離線跑在完全相同的 survey32 查詢影格與 survey33 資料上,確保和
AnyLoc 公平比較。參考影像改用這個專案自己早先建好的 survey33 地理配準拼接圖
(`field_data/survey33/mosaic.png`+`mosaic.pgw`,單一連續影像,而非 AnyLoc 的 112
張離散圖磚)——這才是 ngps_flight 設計文件裡講的「單一 reference_image_path」用法。

**第一輪:純檢索精度比較**(t=116–197.2s,2s cadence,與 AnyLoc 完全相同的 41 個
查詢時刻、相同北向旋轉):

| 方法 | 覆蓋率 | mean | median | RMSE | max |
|---|---|---|---|---|---|
| AnyLoc(全域描述子檢索) | 41/41(100%) | 25.1m | 15.2m | 40.6m | 152.5m |
| ngps 風格(局部特徵+homography) | 21/41(51%) | 22.5m | 25.2m | **24.7m** | **40.9m** |

成功匹配時 ngps 風格明顯更穩(RMSE、max 都遠優於 AnyLoc),但**近半數影格完全匹配
失敗**(RAW 匹配<4 或 RANSAC 內點<8)——AnyLoc 的最近鄰檢索保證每次都有答案(即使
是爛答案),ngps 風格則可能整個 tick 交白卷。精度 vs 可靠度的真實取捨,不是單純
「哪個更好」。

**第二輪:接上完整 pipeline(VIO+修正層+slew),掃描 VPR 查詢頻率**——把每個頻率下
ngps 風格「成功」的匹配(status=="ok")當成真實錨點,餵進與 AnyLoc 完全相同的
`foundloc_corrector_survey32.run_corrector`(`anchor_win=20`,本專案離線正式配置)
+ slew limiter,再用相同的固定 4DOF 對齊方法評估巡航窗 RMSE。測試 period ∈
{0.5, 1.0, 2.0, 3.0, 4.0} 秒(過程中抓到一個真的 bug:第一輪的比對腳本算出
`est_lat`/`est_lon` 但沒存進輸出 JSON,導致錨點沒法重建——修好後整個掃描重跑):

| period | 嘗試次數 | 成功錨點 | 覆蓋率 | 巡航 RMSE | max |
|---|---|---|---|---|---|
| 0.5s | 163 | 85 | 52.1% | **29.11m** | 80.6m |
| 1.0s | 82 | 43 | 52.4% | 31.85m | 88.9m |
| 2.0s | 41 | 21 | 51.2% | 35.00m | 88.9m |
| 3.0s | 28 | 13 | 46.4% | 57.76m | 119.0m |
| 4.0s | 21 | 12 | 57.1% | 47.89m | 100.1m |

**結論:0.5s(所有測試頻率中最高)最好**——查詢愈頻繁,巡航 RMSE 愈低,幾乎單調。
機制:成功率(~50%)基本上跟查詢頻率無關(是匹配品質問題,不是時序問題),所以
唯一能補償這個缺陷的槓桿就是「查更勤」——單位時間內查詢次數愈多,即使成功率不變,
絕對成功錨點數也愈多,能更緊地限制 anchor_win 尺度估計之間的死算推估漂移。0.5s
下的 29.11m 已經很接近 AnyLoc 官方 2s 結果(27.3m,同一套 corrector+slew 配置)但
仍未超越;而在跟 AnyLoc 相同的 2s cadence 下,ngps 風格明顯較差(35.0m vs
27.3m)——同頻率下 AnyLoc 的保底覆蓋率仍然贏。這台 Jetson 上單次匹配平均 216ms,
0.5s 週期還沒撞到算力天花板,更高頻率(如 0.25s)沒測過,可能還有進步空間,但
2.0→1.0→0.5s 之間的改善已經在收斂。

**已知限制**:比對用全域匹配(查詢影格對整張拼接圖,無位置先驗裁切)——這代表
ngps_flight 實際設計(用 UKF 位置先驗裁切參考影像附近區域)下的成功率可能顯著更
高,這裡測的比較接近它的冷啟動/最差情境,不是穩態追蹤情境;RANSAC 閾值(8px)、
最少內點數(8)是我自己選的合理預設,不是原始 repo 的調校值。完整程式碼與數字:
`field_data/survey32/vio_eval/ngps_style_eval_survey32.py`、
`ngps_freq_sweep_corrector.py`、`ngps_style_eval_result_p{period}.json`、
`ngps_freq_sweep_result.json`、`ngps_freq_sweep.png`。

### 14-37. position-prior 裁切版本:全域搜尋的~50%失敗率被根治,還贏過 AnyLoc(2026-08-02)

Frank 要求試 §14-36 開放問題裡提到的 position-prior 裁切版本——不再對整張拼接圖做
全域搜尋,而是模仿 ngps_flight 真正的設計(UKF 位置估計 + 裁切參考影像附近區域再
匹配)。實作為因果(causal,依時間順序、真實運作狀態)版本
(`ngps_style_eval_survey32_cropped.py`):沒有先驗位置時(第一次查詢、或目前為止
每次都失敗)退回全域搜尋;一旦有過一次成功匹配,之後每次查詢改成以「上次成功匹配
的位置」為中心裁切拼接圖,裁切半徑用等速度上界估計(`MAX_SPEED_MPS=15 m/s
×距上次成功匹配經過的秒數 + 30m 安全餘量`)而非讓 VIO 自己的尺度漂移污染裁切中心;
匹配失敗時先驗維持不變(裁切半徑隨經過時間自然變大,行為正確)。

**單一頻率(2.0s,與 AnyLoc 官方相同 cadence)的效果**:一旦進入裁切模式,匹配
密度暴增(全域搜尋時每次 10–30 個原始匹配,裁切後 300–560 個),**成功率從全域
搜尋的 51.2% 跳到 90.2%**(41 次查詢中僅 4 次失敗,且全部集中在最初 bootstrap
的前 4 次查詢,t=116–122s,裁切模式啟動之後 36/36 全部成功)。純檢索精度也大幅
提升:mean 16.9m、median 15.8m、RMSE 18.9m、max 38.8m(對比全域搜尋的 mean
22.5m/RMSE 24.7m/max 40.9m,以及 AnyLoc 的 mean 25.1m/RMSE 40.6m/max 152.5m)。

接上完整 pipeline(同一套 `run_corrector`/anchor_win=20/slew)後,2.0s 頻率下巡航
RMSE=29.08m——比全域搜尋的 35.00m 好上不少,但仍略遜於 AnyLoc 官方的 27.3m。查過
原因:唯一的 4 次失敗全部發生在 bootstrap 窗口(t=116–122s,裁切模式尚未啟動
之前),正好卡進 anchor_win=20 尺度估計閘門最敏感的那段時間——與 §14-31 診斷出的
「閘門在窗口早期特別脆弱」是同一個機制,不是裁切法本身的缺陷。

**全頻率掃描**(period ∈ {0.5, 1.0, 2.0, 3.0, 4.0}s,裁切法 vs 全域搜尋 vs
AnyLoc):

| period | 全域搜尋 RMSE | 全域覆蓋率 | 裁切法 RMSE | 裁切法覆蓋率 |
|---|---|---|---|---|
| 0.5s | 29.11m | 52.1% | **19.13m** | 95.7% |
| 1.0s | 31.85m | 52.4% | 26.66m | 90.2% |
| 2.0s | 35.00m | 51.2% | 29.08m | 90.2% |
| 3.0s | 57.76m | 46.4% | 53.07m | 85.7% |
| 4.0s | 47.89m | 57.1% | 34.92m | 90.5% |

AnyLoc 官方(2.0s):RMSE=27.3m,覆蓋率 100%。

**結論:裁切法在每一個測試頻率下都贏過全域搜尋**(RMSE 更低、覆蓋率更高),而且
**在最高測試頻率(0.5s)下,裁切法巡航 RMSE=19.13m,實際贏過 AnyLoc 官方的
27.3m**——這是這整個 ngps-style 調查(§14-36/37)裡第一次有任何配置真的超越
AnyLoc。機制與 §14-36 一致(高頻率補償 bootstrap 前的空窗期),但裁切法因為穩態
成功率已經高達 85–96%(不像全域搜尋卡在~50%),所以同樣的頻率提升能兌現成更大的
實際效益。

**已知限制**:裁切半徑用固定等速度上界模型,不是真正的 UKF 協方差傳遞,也沒有
把 VIO 自身的尺度漂移誤差回饋進裁切中心的不確定度;bootstrap 階段(拿到第一個
成功匹配之前)仍然是全域搜尋,尚未想辦法縮短這段真空期;RANSAC 閾值、最少內點數
與 §14-36 相同,一樣是我自己的合理預設,不是原始 repo 的調校值;3.0s 那個
RMSE=53.07m 的異常值(比 4.0s 還差)可能只是小樣本雜訊,兩種方法在 3.0s 都出現
類似的異常凸起,值得留意但不影響整體「裁切法系統性優於全域搜尋」的結論。完整
程式碼與數字:`field_data/survey32/vio_eval/ngps_style_eval_survey32_cropped.py`、
`ngps_cropped_eval_result_p{period}.json`、`ngps_cropped_freq_sweep_result.json`、
`ngps_crop_vs_global_sweep.png`。

### 14-38. 三方法 × 五頻率總表:全 pipeline 下誰是最終贏家(2026-08-02)

Frank 要求把「接上 VIO 的完整 pipeline」測試擴大到所有方法——之前 AnyLoc 只在官方
2.0s cadence 測過(§14-26 起沿用的數字),沒有跟兩種 ngps 風格方法一樣做過完整的
頻率掃描,不是真正公平的比較。用 `anyloc/test_accuracy_survey25_time.py --rotate`
在 period ∈ {0.5, 1.0, 3.0, 4.0}s 補測(2.0s 沿用既有的
`anyloc_vs_survey33_db_cruise.json`),同樣接上 `run_corrector`
anchor_win=20 + slew 評估巡航窗 RMSE,與 §14-36/37 完全相同的方法論。

**完整 15 組結果(3 方法 × 5 頻率,巡航窗 RMSE,公尺)**:

| period | AnyLoc | 覆蓋率 | ngps 全域搜尋 | 覆蓋率 | ngps 裁切法 | 覆蓋率 |
|---|---|---|---|---|---|---|
| 0.5s | 22.15 | 100% | 29.11 | 52.1% | **19.13** | 95.7% |
| 1.0s | 21.69 | 100% | 31.85 | 52.4% | 26.66 | 90.2% |
| 2.0s | 27.26 | 100% | 35.00 | 51.2% | 29.08 | 90.2% |
| 3.0s | 32.79 | 100% | 57.76 | 46.4% | 53.07 | 85.7% |
| 4.0s | 35.92 | 100% | 47.89 | 57.1% | 34.92 | 90.5% |

**總冠軍:ngps 裁切法 @ 0.5s,巡航 RMSE=19.13m**——15 組配置裡的最佳結果,贏過
AnyLoc 在任何頻率下的表現(AnyLoc 自己的最佳點是 1.0s=21.69m,不是先前一直被
當作「官方」基準的 2.0s=27.26m)。

**附帶發現:AnyLoc 自己的「官方」2.0s 配置也不是它的最佳頻率**——1.0s(21.69m)
比 2.0s(27.26m)好 20%。這代表這整個專案先前用來當作基準、寫進三份論文的 AnyLoc
2.0s 數字,其實不是 AnyLoc 本身的最佳表現,只是先前選定的固定測試 cadence——每種
方法都吃「查更勤」這個紅利,只是吃到的量不同(覆蓋率已經 100% 的 AnyLoc 吃得比較
少,卡在~50% 覆蓋率的 ngps 全域搜尋幾乎沒吃到,已經被裁切法修到 85-96% 覆蓋率的
ngps 裁切法吃最多)。

**三方法排名總結**(以各自最佳頻率比較):ngps 裁切法(19.13m @ 0.5s)> AnyLoc
(21.69m @ 1.0s)> ngps 全域搜尋(29.11m @ 0.5s)。ngps 裁切法勝出的關鍵不是
SuperPoint+LightGlue 本身比 DINOv2+VLAD 更強,而是它結合了(a)局部特徵匹配在
成功時更精準(§14-36 已量到 RMSE 24.7m vs AnyLoc 的 40.6m)與(b)position-prior
裁切把原本~50% 的失敗率修到 85-96%——兩者疊加後在高頻查詢下反超。

**尚未回答的問題**(留給未來若要繼續深入):ngps 裁切法在其他窗口(全程飛行、
不同場地)是否一樣贏,還是 survey32/33 特有的巧合;bootstrap 前的空窗期(§14-37
提到的全域搜尋 fallback)如果也想辦法縮短,ngps 裁切法可能還有進步空間;三種方法
的運算成本(每次查詢的 GPU 時間)沒有統一比較過,只各自記錄了平均推論時間。完整
程式碼與數字:`field_data/survey32/vio_eval/anyloc_vs_survey33_db_cruise_p{period}.json`、
`anyloc_freq_sweep_result.json`、`full_pipeline_method_freq_comparison.png`。

### 14-39. Frank 直接看路徑圖發現「overshoot 還是 VIO 的問題」——查程式碼證實(2026-08-02)

Frank 看 §14-38 的 `best_config_path_vs_gps.png` 直接提出:融合後的路徑還是有明顯
overshoot/loop 痕跡,看起來還是 VIO 的問題。查 `run_corrector` 逐 tick 迴圈證實
完全正確:`inc = s_est * (p[k] - p[k-1])`——每個 tick 的死算方向**完全來自 VIO
自己原始的 frame-to-frame 位移方向,未經任何旋轉修正**,只有大小被 s_est(緩慢
EMA 更新的純量)縮放;anchor 的拉力只修正**位置**(週期性把位置拉回錨點)和
scale(緩慢更新),從來沒有修正過 VIO 自己的**航向**。實測:融合輸出的 tick-to-
tick 航向變化與原始 VIO 航向變化相關係數 0.69,而且融合輸出的航向變化標準差
(44.6°/tick)還比原始 VIO 自己(31.3°/tick)**更大**——因為每次 anchor pull 是
瞬間位置跳動,疊加在 VIO 自己的雜訊之上,不是抵銷。結論:overshoot 的形狀確實
是 VIO 自己方向誤差的直接反映,corrector 架構上從未修正過這件事。

**Frank 提議:用羅盤(compass)修正方向,因為羅盤應該比較準。** 查
`telemetry.csv` 的 `heading_deg`:5Hz、無缺漏、全程都有。實作
`field_data/survey32/vio_eval/foundloc_corrector_compass.py`
(`run_corrector_compass`):鎖定(`anchor_R`)建立之後,用羅盤讀數換算回 VIO 自己
的座標系(透過同一個鎖定旋轉的反變換),與 VIO 自己的 `quat_yaw` 比較差值
`dyaw`,把每個 tick 的原始位移向量旋轉 `dyaw` 後再用(**只修正方向,不動 VIO 自己
的位移量**)。

**在 0.5s(ngps 裁切法目前的最佳頻率)測試,幾乎沒有效果**:RMSE 19.13m→19.19m,
路徑圖疊在一起幾乎完全重合(平均差異僅 0.22m,最大 0.81m)。診斷:0.5s cadence 下
anchor_pull=0.6 的位置拉力太強太頻繁,兩次錨點之間根本沒有足夠時間讓方向誤差累積
出看得見的位置差異——位置拉力主導了一切,方向修正的效果被蓋掉了。**這預測了在
較低頻率(anchor 之間死算更久)compass 修正應該更有感**——在 3.0s、4.0s 重測
(anchors 來自 §14-37 已經算好的 ngps 裁切法結果):

| period | 原版 RMSE | compass 修正後 RMSE | 差 |
|---|---|---|---|
| 3.0s | 53.07m | 50.27m | -2.80m(約 5.3%) |
| 4.0s | 34.92m | 33.93m | -0.98m(約 2.8%) |

確認預測方向正確,路徑圖也證實在較平滑的擺動段(非糾結段)compass 版本確實走得
更直——但兩者共有的一段真正糾結的路徑(不是方向問題,可能是某次錨點本身品質差
或轉彎追蹤失敗)compass 修正完全沒碰到,維持原樣。**結論:compass 方向修正是真的
有效的機制,但效果溫和(2.8-5.3% RMSE),且只在錨點頻率較低、死算時間較長時才有
明顯空間發揮**;在已經很密集的查詢頻率下(如 0.5s),位置拉力本身就把方向誤差的
影響壓縮到可忽略。程式碼:`foundloc_corrector_compass.py`;圖:
`compass_correction_path_compare.png`(0.5s)、
`compass_correction_path_compare_3_4s.png`(3.0s/4.0s)。

### 14-40. VIO 自己的 yaw 為什麼在低、高 AGL 都一樣「凍結」?——不是 AGL 造成的(2026-08-02)

原本假設「高 AGL 視差差 → 視覺對 yaw 的修正也弱 → VIO yaw 凍結」,直接用資料
反駁自己:比較 survey30(~5m)/survey31(~10m)/survey32(~100m)三趟飛行 VIO 自己
的 yaw tick-to-tick 變化標準差——0.66°/0.86°/0.78°,三個 AGL 幾乎一樣,與
compass 的 tick-to-tick 變化相關係數也全部接近零(-0.07 至 -0.08)。**這個假設
不成立**——如果是 AGL/視差造成的,5m AGL(scale 精度極佳,RMSE 只有 7.3m)應該
明顯不同於 100m AGL,但兩者的 yaw 行為看起來一樣。

進一步檢查 compass 本身是不是只是雜訊(如果 compass 的 tick-to-tick 變化只是
感測器雜訊,那 VIO 平滑、compass 吵鬧,不代表 VIO 有問題):找 survey30 真正靜止
的地面段(t=0-56s,人員還沒搬動載具,航向穩定在 335.2-335.4°)算 tick-to-tick
標準差,只有 **0.018°/tick**——遠低於飛行時的 5-7°/tick,證實 compass 本身雜訊
極低、是可信的感測器,飛行時的高變化是真實訊號,不是雜訊。

**結論**:VIO 自己的 yaw 估計,不論 AGL 高低,都比真實航向動態「凍結」/過度平滑
——這更像是濾波器本身的調校特性(process/measurement noise 參數讓每次視覺修正
只能小幅移動 yaw 狀態),而不是視差不足造成的資料飢餓問題。與 §14-35 的「AGL 造成
scale 誤差」是不同機制,不能用同一套解釋。完整寫在後續 §14-41 的 scale vs
direction 對照裡進一步釐清。

### 14-41. Scale 還是 Direction,哪個才是低高 AGL 準確度差異的關鍵?——資料證實主要是 Scale(2026-08-02)

用兩種獨立方法拆解 survey30/31/32 三趟飛行原始 VIO 誤差:

1. **全域單一 4DOF(僅航向+平移)vs 5DOF(多允許一個全域尺度)對齊比較**——發現
   這個方法在低速/低位移窗口(survey30)會因為擬合窗口位移太小而病態(擬合出
   k=0.183 這種明顯錯誤的尺度,「修正」後反而更差),方法本身有缺陷,棄用。
2. **改用滑動窗口(10s 窗、5s step)分別追蹤「尺度比例」(VIO 弧長/GPS 弧長)與
   「方向一致性」(每個子窗口起訖弦的角度,VIO vs GPS)隨時間的穩定度**——更穩健:

| 飛行 | 尺度比例平均值 | 尺度穩定度(變異係數) | 方向一致性(標準差) |
|---|---|---|---|
| survey30(~5m) | 1.033(≈正確) | 0.05(很穩) | 2.4°(很緊) |
| survey31(~10m) | 1.035(≈正確) | 0.08(穩) | 5.5°(緊) |
| survey32(~100m) | **1.851(明顯偏差)** | **1.05(混亂,0.83x-8.37x 間擺盪)** | **57.2°(鬆)** |

(方向一致性看的是「標準差」不是「平均值」——平均角度差本身很大(69-156°)只是
反映 VIO 自己任意起始座標系跟真北沒對齊,這正是一次性 lock 要修的東西,不代表
方向追蹤本身不準。)

**結論**:兩者隨 AGL 都變差,但 scale 崩壞得遠遠更徹底——低 AGL 時 scale 平均值
接近正確**且**穩定(5-8% 變異);100m AGL 時 scale 平均值明顯偏差(偏大 85%)
**且**同一趟飛行內就能擺盪超過 10 倍(0.83x 到 8.37x)。方向的一致性標準差雖然
也隨 AGL 惡化(2.4°→5.5°→57.2°,約 10-20 倍),但沒有像 scale 那樣「平均值錯
且變異失控」雙重崩壞。這與單目 VIO 的理論預期一致:尺度天生比旋轉更難單靠視覺
觀測(需要真正的視差/深度基線比,旋轉即使基線很小也常能從特徵方位角變化推得)。
**簡答:主要是 scale,方向也受影響但程度輕得多。**

### 14-42. 純 IMU(無視覺)在同一窗口漂移多遠?——證明準確度來自融合,不是 IMU 本身(2026-08-02)

Frank 問「低 AGL VIO 準確,是因為原始 IMU 準,還是整個 VIO 融合的功勞?」——IMU
硬體本身的雜訊/偏置特性物理上跟飛行高度無關(同一顆感測器),所以如果低 AGL
比較準,理論上一定是視覺修正的功勞,不會是「IMU 在低空突然變準」。直接用資料
驗證而非空口假設:對 survey30 原始 `imu.csv`(陀螺+加速度計,無視覺)做純
strapdown 積分(Rodrigues 旋轉更新 + 重力扣除後雙重積分),**用 GPS 真值完美
初始化位置與速度**(對純 IMU 死算最有利的起始條件),積分同一個 120 秒巡航窗:

| 累積時間 | 純 IMU 漂移(GPS 完美初始化) |
|---|---|
| +5s | 5.3m |
| +10s | 39m |
| +20s | 296m |
| +40s | 2.2km |
| +80s | 15.3km |
| +120s(整個窗口) | **41.2km** |

對比同一窗口的完整 VIO(視覺+IMU 融合):RMSE=7.3m。純 IMU 即使給了完美的起始
條件,120 秒內還是飄了四萬多公尺,而融合後的系統同一時間只飄了 7 公尺左右。
**結論:低 AGL 的準確度 100% 是視覺修正的功勞,不是 IMU 本身變準**——這是基礎
慣性導航物理(加速度計偏置經雙重積分會以時間平方成長,再疊加陀螺積分誤差污染
重力扣除),跟飛行高度完全無關;真正隨 AGL 改變的只有相機能提供多好的視覺約束
去反覆拉回這個必然發散的 IMU 死算,這與 §14-35/41 的視差/scale 觀測性故事完全
一致。

### 14-43. 「先衝高頻率」這條路能做到多好?——找到真正的頻率上限,大約 0.5s(2026-08-02)

延續 §14-38 的建議,把 ngps 裁切法的查詢頻率往 0.5s 以下推:額外測 period ∈
{0.2, 0.3, 0.4}s(0.5s 已有),同樣接上 `run_corrector`(anchor_win=20)+ slew
評估巡航窗 RMSE:

| period | 巡航 RMSE | 覆蓋率 |
|---|---|---|
| 0.2s | 19.70m | 96.6% |
| 0.3s | **18.23m**(數字上最佳) | 95.9% |
| 0.4s | 20.01m | 96.6% |
| 0.5s | 19.13m | 95.7% |
| （對照)1.0s | 26.66m | 90.2% |
| （對照)2.0s | 29.08m | 90.2% |

**結論:0.5s 以下報酬完全打平**——0.2-0.4s 全部落在 18-20m 區間,不再是像
4.0s→0.5s 那樣清楚的單調改善曲線,只是雜訊等級的上下擺動,0.3s 數字上最好但
沒有超出誤差範圍的意義。而且 0.2s(200ms 週期)已經低於這台 Jetson 裁切匹配的
平均推論時間(299ms/次)、不可能真的即時跑;0.3s(300ms)也在臨界邊緣。0.4s、
0.5s 才是真正舒服落在算力預算內、可以實際部署的選項。**「拉高查詢頻率」這條路
的真實上限大約在 0.5s、19m RMSE 附近,不是通往零誤差的路徑**——找到了明確的
報酬遞減拐點,之後要繼續進步需要換別的機制(如 §14-41 指出的、針對 scale
不穩定度重新設計自適應速度更快的 scale 估計,而非繼續加快查詢頻率)。完整數字:
`field_data/survey32/vio_eval/ngps_cropped_finer_sweep_result.json`、
`ngps_cropped_full_freq_curve.png`。

### 14-44. attitude.csv 補文件 + 順手查了一下:巡航時鏡頭其實不是純 nadir(2026-08-04)

Frank 問 `attitude.csv` 的欄位(`stamp_ros,recv_unix,qw,qx,qy,qz`)是什麼意思——
寫成完整說明文件 `field_data/attitude_format.md`:兩個時間戳的差別
(`stamp_ros`=FC/MAVROS 訊息自己的時間戳,做跨感測器時間對齊要用這個;
`recv_unix`=Jetson 收到當下的 `time.time()`,只用來查 pipeline 延遲)、四元數
四個分量的幾何意義(`qw=cos(θ/2)` 是角度,`qx,qy,qz=axis×sin(θ/2)` 是軸)、
roll/pitch/yaw 換算公式,以及這個 attitude.csv 用的 Hamilton 慣例跟 VIO 軌跡
CSV 用的 JPL `q_GtoI` 慣例不同,兩者不能直接混用。

順手用這份資料查了 survey32 巡航窗(t=116-197s)的 roll/pitch 實際分佈:roll
mean=-1.40° std=4.68°(範圍 -19.9°至 6.4°,大致貼平但轉彎時有真實傾角);
**pitch mean=+5.63° std=8.87°(範圍 -14.8°至 23.3°)**——巡航時鏡頭並非單純垂直
朝下,而是系統性地前傾約 5-6 度(前飛所需的正常機身姿態),波動可達 23 度。

**這件事目前還沒被本專案任何一處數學處理過**:AnyLoc/VPR 的地面涵蓋範圍計算
(`half_w_m = agl_m * tan(HFOV/2)`,`anyloc/localizer.py` 的 `_sat_crop`)、
§14-35/41 建立的「深度≈AGL」視差率分析、ngps 裁切法用世界檔案做的單純仿射
像素→經緯度換算——這些全部假設純垂直下視相機幾何,實際上鏡頭有實測 5-23 度不等的
真實傾角,理論上會讓真實地面涵蓋範圍(尤其前後方向)systematically 偏離計算值。
**尚未量化這個誤差實際影響多大**,只是在補文件的過程中順便發現、記錄下來——
留給未來一次專門的分析(例如檢查 pitch 大小是否與已知的 retrieval/VIO 誤差
相關)。完整程式碼與數字寫在 `field_data/attitude_format.md` 文末。

### 14-45. Global shutter 相機實測——結果:無法判讀,卡在沒有真標定(2026-08-11)

Frank 用復原後的 AP-IMX900(4mm CS-mount 鏡頭,global shutter)新飛了五趟
(survey38~42),問「global shutter 相機有沒有解決 VIO 問題」。用跟 §14-35
完全相同的方法論(同一個 `run_video_msckf`、固定 4DOF 對齊、AGL 曲線抓
巡航窗)在 survey38(~10m,對比 survey31)、survey41(~65m,無舊機身對照)、
survey42(~100m,對比 survey32)上跑——survey39/40 是建圖用途,本次未跑。

**卡關:`~/openvins_ws/config/` 只有 IMX219 那次(2026-07-23)的真 Kalibr 標定,
復原回來的 AP-IMX900+4mm 新鏡頭組合從未做過 AprilGrid/Kalibr。** 臨時湊了一份
「computed-FOV pinhole intrinsics(HFOV 59.9°/VFOV 46.7°,本身就是算出來、非
實測)+ 沿用 IMX219 那次 Kalibr 的 extrinsics 當種子(理由:機身掛載位置理論
上沒變,|t|=0.358m 是量過的物理事實)+ 三個 calib_cam_* 全開線上細修」的克難
config(`survey38_ap_imx900_globalshutter/`),仿照當年 survey17 未標定首跑
的作法。

**結果:三趟全部災難性發散**,不是「比舊機身差一點」,是差 3-4 個數量級——
survey38 RMSE 20325m(對比 survey31 的 3.0m)、survey42 RMSE 1753m(對比
survey32 的 75.8m),起飛後幾秒內水平速度就衝到 8+ m/s(實際只是緩慢爬升到
10m),典型 filter 發散,不是真實運動。

**診斷:把 survey42 的 `calib_cam_extrinsics/intrinsics/timeoffset` 三個全部
關掉(凍結種子值,不讓線上細修)重跑一次(`survey38_frozen_calib_diag/`)
——結果更糟(229s 內飄到離原點 44.6km),排除「線上細修在搞破壞」這個解釋,
指向種子本身(intrinsics 和/或 extrinsics)離真實幾何太遠,不是細修沒調好。
computed-FOV intrinsics(未實測)和沿用自不同機身的 extrinsics 兩者都是可疑
對象,這次測試無法區分是哪一個(或兩者都是)造成的。

**結論:這個問題目前答不了。** rolling vs global shutter 的比較,前提是兩邊
VIO 都要先收斂到物理上合理的軌跡,新機身這邊做不到——不是「global shutter
沒有比較好」,是「這組克難標定不夠格拿來比」。要往下走,唯一路徑是先幫
現在這組 AP-IMX900+4mm 鏡頭做一次真正的 Kalibr 標定(AprilGrid 錄影 + PC
Docker,程序見 §13 / `KALIBR_PC_README.md`,這次沒做——現有的 calib_2026072*
資料夾全部是 IMX219 那次的,對新鏡頭無效),標定完再重跑這次完全相同的
比較。§14-16/14-17 當年在 IMX219 上得出的「主因是 KLT 追蹤無運動預測,不是
快門類型」這個判讀維持不變(那是獨立的、跟這次標定缺口無關的證據),只是
「換 global shutter 有沒有實測改善」這個獨立問題,目前仍是未知數。

**檔案**:`field_data/vio_globalshutter_test_README.md`(完整記錄)、
`field_data/vio_globalshutter_comparison_result.json`(數字已標註 INVALID)、
`field_data/survey38(41,42)/vio_eval/vio_full_surveyXX.csv`(發散軌跡,留供
標定補上後重新分析用)、`~/openvins_ws/config/survey38_ap_imx900_globalshutter/`
+ `survey38_frozen_calib_diag/`(兩份 config)。

### 14-46. §14-45 後續追問三連——排除旋轉、排除掉幀、釐清「不只是尺度」,並把標定排上計畫(2026-08-11 同日稍晚)

Frank 對 §14-45 的結果連續追問了幾個方向,逐一直接查證(不是理論推測):

**(1) 是不是轉彎/旋轉造成的?**——把 attitude.csv 的真實姿態(四元數→yaw)
跟 VIO 自己的速度爆走時間點對齊畫圖(`field_data/vio_globalshutter_
rotation_check.png`)。結果**跟旋轉沒有關聯,甚至方向相反**:survey38/41
在 yaw 完全平的時候就已經爆到數百 m/s(旋轉都還沒發生,發散已經在先);
survey42 在一段真實 ~40° 的 yaw 轉彎中(t=18-33s)速度反而穩定維持 2.5m/s,
轉完之後才開始發散。同時額外驗證:VIO 自己內部的姿態估計(qx,qy,qz,qw
換算 yaw)跟真實姿態在這段真轉彎中相關係數 **0.9941**(誤差多數 <2°)——
代表陀螺積分出來的姿態本身沒有嚴重跑掉,問題不在姿態/旋轉外參,排除
「旋轉揭露了錯的旋轉外參」這條解釋。

**(2) Frank 自己看影片覺得偶爾有 frame jump,是掉幀嗎?**——寫了逐幀
灰階差異掃描(整支影片,`field_data/vio_globalshutter_framejump_check.png`),
交叉比對每個視覺跳動是否對得上 frame_times.csv 的時間戳缺口。結果:
survey38 全片 648 個視覺跳動、survey42 全片 58 個,但兩者合計 706 個裡
**只有 9 個對應到真的時間異常(且都 <60ms,頂多漏一幀),其餘 98.7% 發生
在完全正常的 ~33ms 間隔上**——是真實的鏡頭/場景運動(起降前後手持搬動、
飛行中真轉彎),不是掉幀或資料損毀,也跟前面的發散無因果關係(集中在
测試窗口之外,或發散早就已經發生之後)。

**(3) 路徑形狀整個歪掉,不是應該「標定只影響尺度」嗎?**——這個直覺不對:
相機-IMU 標定有四個獨立未知數,只有內參(焦距)主要對應尺度;外參
**旋轉**錯了會讓視覺修正量套用到錯的軸向(直接污染姿態估計,但(1)已經
排除這條);外參**平移**(lever arm,這次沿用的是不同機身的 0.358m 舊值)
錯了的話,任何真實角速度(轉彎、甚至震動)都會透過 `v = v_imu + ω×r`
耦合項誤讀成額外平移速度——這會扭曲路徑**形狀**而非只是縮放;時間偏移
錯了同樣是形狀/相位問題。用同一組數據驗證:survey42 真轉彎期間 VIO 自己
的 yaw 精準跟隨真實 yaw(見上),代表問題不在旋轉外參,**更可能是沿用
的平移外參(lever arm)或內參/尺度耦合造成的 runaway feedback,而非旋轉
外參錯誤**——把嫌疑範圍縮小到「這個機身真的需要重新量測」的那幾個參數,
不是全盤都錯。

**(4) 順手查了相機硬體本身的極限**:`v4l2-ctl --list-formats-ext` 確認
2048x1536 MJPG 原生支援 30/60/80fps 三檔;直接繞過 record_field.py 的
encode/串流管線、純 OpenCV 讀取這台相機量到穩定 29.64fps(median dt
33.71ms,無停滯)——**相機硬體本身不是先前 survey39/40/41 fps 偏低
(25.5-26.4fps)的原因**,那是已修的串流阻塞地雷(見 memory
`record-field-stream-stall-fix`)造成的,不是相機能力問題。也重新確認了
這個專案自己 2026-07-27 的 stride A/B 測試結論(§14-30):15Hz(stride=2)
仍是目前實測最好的 VIO 餵料頻率,更高 fps(30Hz/stride=1)已證實讓 raw
VIO 更差,相機原生 60-80fps 能力目前沒有直接用途(除非做動態 stride,
尚未嘗試)。

**這一輪的淨結果**:排除了旋轉、排除了資料損毀/掉幀,把嫌疑更精準地
指向「這組克難標定的外參平移(或內參)量測不準」,而非某種全新的機制。
**下一步已經是唯一路徑**:幫這個 AP-IMX900+4mm 鏡頭機身做一次真正的
Kalibr 標定。完整分階段計畫已經寫好並著手準備第 2 階段(Frank 稍後
會執行錄影,詳見 memory `ap-imx900-kalibr-calibration-plan`):

- `tools/prepare_kalibr_input.py`(從舊標定 session 資料夾裡的一次性腳本
  升格為可重用工具,本身其實已經跟相機無關,不需改動)
- `instructions/KALIBR_PC_README_ap_imx900_globalshutter.md`(新相機專用
  PC 端 Kalibr 操作說明模板,含這台相機的 sanity anchor:
  fx≈1777/fy≈1779/cx≈1024/cy≈768、沿用的旋轉外參錨點、|t|≈0.358m 僅供
  參考待重新量測)

錄影+PC 端 Kalibr 跑完後,把結果接回 OpenVINS config、重跑跟這次完全
相同的 survey38/41/42 評測,才能真正回答「global shutter 有沒有解決
VIO」這個問題。

**檔案**:`field_data/vio_globalshutter_rotation_check.png`、
`field_data/vio_globalshutter_framejump_check.png`。

### 14-47. Kalibr Phase 1 錄影完成 + trim-head 預設值過期地雷(2026-08-11 深夜)

Frank 錄了兩趟 `record_field.py --calib`(FC 上電、串流到 MediaMTX relay 供
即時確認取景)：`calib_20260811_232301`(122.4s,3656 幀)與
`calib_20260811_232621`(86.2s,2546 幀)。

**驗收(Jetson 端,兩趟都做)**:IMU 都乾淨——332.0 Hz 實測(對應
`--imu-hz 333`),間隔標準差極小、**零 >14ms 缺口**;`meta.json` 的
`frame_rotation_deg: 180`、`purpose: calibration` 都確認正確。

**過程中發現的地雷**:一開始用稀疏抽樣(每 10 秒看一張)肉眼檢查,兩趟看起來
差異很大(一趟像是清晰對焦、一趟像是嚴重失焦特寫)——但這只是抽樣運氣問題。
換成程式化逐幀掃描(`cv2.aruco.detectMarkers`,`DICT_APRILTAG_36h11` +
`markerBorderBits=2`,配合 Laplacian 變異數當清晰度指標,這台 OpenCV 是
4.5.4、用的是舊版 `Dictionary_get`/`DetectorParameters_create` API 而非新版
`ArucoDetector`)後才發現:**兩趟一開始都有一段清晰度/tag 偵測數趨近於零的
「暗場/失焦」前導片段**——不是黑幀,是攝影機被撿起來對準標靶之前的過渡畫面
——`calib_20260811_232301` 約持續到 t≈22.3-23.8s,`calib_20260811_232621`
約持續到 t≈26.3-26.8s(兩者之後清晰度與 tag 數都跳升,例如前者從
sharp≈7.9/tags≈1 跳到 sharp≈34-47/tags≈22-30)。

`prepare_kalibr_input.py` 的 `--trim-head` 預設值是 **12.5s**,是照搬
IMX219 那次(2026-07-23)錄影量出來的數字,對這兩趟新錄影**完全不適用**
——如果照預設值跑,會把 10 秒左右的暗場/失焦幀混進 Kalibr 標定輸入。腳本
docstring 本身已經提醒要「自行核對」,這次正是活生生的案例:稀疏肉眼抽樣
會漏看,程式化掃描才抓到。已把這個提醒寫進 `tools/README.md` 的
`prepare_kalibr_input.py` 條目,供以後任何新錄影重複使用同一套掃描方法,
而不是直接信任預設值或抽幾張圖看看。

**去除前導片段後的 tag 偵測掃描**(2Hz 取樣):`calib_20260811_232301`
平均 12.8 tags/幀、中位數 6、39.2% 幀 ≥15 tags、41.8% 幀有 tag 貼近畫面
邊緣(12% 邊界內,corner/edge 覆蓋度尚可);`calib_20260811_232621` 平均
16.8、中位數 17、51.5% 幀 ≥15 tags,但邊緣覆蓋只有 30.3%、去除前導後可用
長度只剩 ~60s(比前者的 ~99s 短)。

**選定 `calib_20260811_232301` 為主要 session**(可用時長較長、邊緣覆蓋
較好),`calib_20260811_232621` 留作備援(若主要 session 標定不收斂或
驗收不過再用)。已把 `tools/prepare_kalibr_input.py` 複製進主要 session
資料夾,並寫入一份填好真實數字(含正確 `--trim-head 23`)的
`KALIBR_PC_README.md`,取代通用模板
`instructions/KALIBR_PC_README_ap_imx900_globalshutter.md` 裡的
`<TODO>` 佔位符。

**下一步(Phase 3-6,Frank 手動)**:量測 AprilGrid 實際列印尺寸填入
`target.yaml`、量測相機到 FC IMU 的實際距離、把 session 資料夾搬到有
Docker 的 PC 上跑 `prepare_kalibr_input.py` + Kalibr 兩階段標定、把結果
接回 OpenVINS config 後重跑 §14-45/14-46 完全相同的 survey38/41/42 評測。

**檔案**:`field_data/calib_20260811_232301/KALIBR_PC_README.md`(填好的
操作說明)、同資料夾 `prepare_kalibr_input.py`(複製自 `tools/`)。

### 14-48. Phase 4-6 完成:真標定接回 OpenVINS——高 AGL 大幅改善,低/中 AGL 仍發散(2026-08-12)

**Kalibr 結果(PC 端跑完,結果搬回 `field_data/calib_20260811_232301/kalibr_output/`)**:
內參合理(fx=1891.3/fy=1890.4,與 FOV 推算的 sanity anchor 1777/1779 相差 ~6%,
在 ±10-15% 容許內);外參旋轉與 nadir 掛載錨點相差僅 ~1.3°。但兩階段
reprojection error 都**高於驗收線**——cam-only std 0.94-1.18px(線是 <0.5px)、
imu-cam mean 1.36px(線是 <1px,IMX219 那次是 0.65px 過關)。外參平移
|t|=0.259m 原本與舊機身量測值(0.358m,不同鏡頭)有落差,**Frank 現場拿尺量
了目前機身的相機到 FC IMU 距離,結果正是 0.259m,完全吻合**——解除了
§14-46 留下的平移不確定性,這是這次標定最強的正面訊號(即使 reprojection
error 略高於線)。

**接回 OpenVINS**:新建 `~/openvins_ws/config/ap_imx900_kalibr/`(鏡像
`survey17_kalibr/` 結構),`kalibr_imucam_chain.yaml`/`kalibr_imu_chain.yaml`
直接抄 Kalibr 輸出(注意 Kalibr 給的是 T_cam_imu,OpenVINS 要的 T_imu_cam 是
其反矩陣,用 `calib_full-results-imucam.txt` 的 T_ic 或自己反矩陣皆可);
`estimator_config.yaml` 把 `up_msckf_sigma_px`/`up_slam_sigma_px` 從 1 調到
1.5,理由是這次量到的 reprojection error 比舊 sigma=1 所依據的 0.65px 高
一倍,若仍用 sigma=1 會讓濾波器對視覺量測過度自信。

**評測方法地雷(這次新發現,與標定本身無關)**:第一次嘗試直接把
`run_video_msckf` 的 `[start_off] [end_off]` 設成 RMSE 評測窗本身(例如
survey38 設 194-313s),結果整段都印
`failed static init: no accel jerk detected, platform moving too much`,
輸出 CSV 只有表頭、零筆資料。查了 `InertialInitializer.cpp` 源碼才確認:
靜態初始化需要看到「先靜止、後有感測到 jerk」的真實過渡(`has_jerk`
邏輯,基於視覺 disparity 判定——兩個半窗都高視為一直在動,判不了),而
`init_dyn_use: false`(繼承自 IMX219 那次的設定,兩份 config 完全沒改到
這條)代表沒有備援的動態初始化路徑。三趟飛行的 RMSE 評測窗本身都在起飛
之後的巡航段,不含真正的起飛前靜置——舊的(近似標定)那次能成功,是因為
`run_video_msckf` 當時餵的區間其實從起飛前靜置段就開始跑(比對舊
`vio_full_surveyXX.csv` 最早一筆時間戳,三趟分別落在各自起飛/jerk 時刻
附近:survey38≈182.4s、survey41≈54.0s、survey42≈73.4s),RMSE 評測窗只是
事後用時間戳篩選出來的子集,不是 `run_video_msckf` 實際餵的範圍。修法:
這次三趟都改成從起飛前靜置段(留一點餘裕)開始餵,例如 survey38 用
`170 313`(起飛 jerk 在 ~183s)、survey41 用 `40 143`、survey42 用
`55 229.4`,讓濾波器在真實靜置轉動的那一刻自然初始化,RMSE 才用
`vio_globalshutter_kalibr_comparison.py`(仿 `vio_agl_comparison.py`
的 fixed-4DOF 對齊法)事後篩選評測窗。**這條「擷取範圍要涵蓋真實靜置段,
評測窗只在事後篩選」的教訓對任何未來的 `run_video_msckf` 呼叫都適用。**

**結果**(`field_data/vio_globalshutter_kalibr_comparison_result.json`):

| flight | AGL | 舊 IMX219 基準 | 近似標定(無效) | 真標定(這次) |
|---|---|---|---|---|
| survey38 | ~10m | 3.0m(survey31) | 20325m | **26320m**——仍災難性發散 |
| survey41 | ~65m | n/a | 20990m | **9445m**(較短、避開 stall gap 的窗,仍災難性發散) |
| survey42 | ~100m | 75.8m(survey32) | 1753m | **301.9m**——進步 5.8 倍,速度曲線不再暴衝(全程 0.5-22 m/s) |

survey38/41 速度曲線與近似標定那次幾乎同一種模式(EKF 正回饋暴衝、每 ~15-20s
倍增),只是這次晚幾秒才開始——真標定沒有解決低/中 AGL 的發散,只是稍微
延後了發生時間。survey42(100m)則是質變:不再暴衝,但比同高度的舊 IMX219
基準仍差約 4 倍。

**結論**:真標定是必要條件但不是充分條件。這個結果收斂回 §14-35/14-41 已
建立的「AGL 才是主因」("AGL is the dominant VIO error driver")——低 AGL
下視差變化快、單目三角測量條件差,不管標定多準、shutter 是哪種,濾波器在
低/中 AGL 的可觀測性都不夠。原始問題「global shutter 相機能不能修好 VIO」
目前仍無法乾淨回答:100m 這裡雖然終於是有效比較,但新相機(global
shutter+4mm CS-mount 鏡頭)在唯一可比的高度上反而比舊相機(rolling
shutter+M12 鏡頭)差,不支持「换 shutter 類型解決問題」的假說;10m/65m
則因兩者都發散,連比較的基礎都沒有。若要真正回答這個問題,下一步可能要:
(a) 針對低 AGL 發散做進一步診斷(是否同 §14-11/14-16 那次 rolling-shutter
病灶類似的機制,還是純粹 AGL 帶來的視差/尺度問題),或 (b) 接受現有證據,
把资源转向已经在做的下游修正(AnyLoc 融合層/AGL 先驗)而非追求原始 VIO
本身收斂。

**檔案**:`~/openvins_ws/config/ap_imx900_kalibr/`(新 config)、
`field_data/survey{38,41,42}/vio_eval/vio_full_surveyXX_kalibr.csv`(新軌跡)、
`field_data/vio_globalshutter_kalibr_comparison.py`(評測腳本)、
`field_data/vio_globalshutter_kalibr_comparison_result.json`(結果)。

### 14-49. 新相機的完整 VPE 融合管線實測——同域資料庫檢索太弱,任何融合都打不贏純 VO(2026-08-12 稍晚)

承接 §14-48(真標定後 100m raw VIO 進步到 301.9m,10m/65m 仍發散),Frank 接著問
「完整 VPE 管線(不只是 raw VIO)在同域資料庫下表現如何」。同域資料庫的價值
在 §14-19/14-20/14-26(舊 IMX219 相機)已經證實比衛星資料庫好 12-33 倍——這次
是幫新相機(AP-IMX900 global shutter)重做同一套驗證。

**建資料庫**:survey40(2026-08-11 錄的「建圖用」飛行之一,~100m AGL 巡航
200s、乾淨無斷訊)用 `tools/extract_frames.py --rotate --min-dist 15 --min-agl
90` 抽出 102 張影格(密度確認良好,鄰近影格間距中位數 15.5m、無大洞),
`anyloc/build_database_real.py` 建出 `anyloc/database_survey40_samedomain_vits14`。
用來查詢的 survey42 與 survey40 是**同一天、同一區域、同一台相機的兩趟不同
飛行**,錄影起始時間相差 **~49 分鐘**。

**結果——比舊相機的跨飛行同域測試差很多**:即使是最寬鬆的全庫(非受限)
查詢,平均誤差也有 **~159m**(`anyloc/logs/survey42_samedomain_vo_fusion.json`),
信心分數普遍偏低(平均 0.22-0.24)且**不分辨對錯**(err<50m 的匹配平均分數
0.203,err≥50m 的平均分數反而略高於 0.223)——跟 2026-07-27 §14-26 那次
24.1m、分數能正確標記壞匹配的結果性質完全不同。密度確認過不是資料庫覆蓋
不足的問題,49 分鐘的光影/陰影落差(比 §14-26 的 15 分鐘長得多)是頭號嫌疑,
但未定案。

**接著測試三種融合方案,全部輸給「乾脆不用 AnyLoc」**:

| 方案 | mean err |
|---|---|
| Raw VIO only(無 AnyLoc) | 217.9m |
| Production anchor-chain(`ros2_node.py` 對應邏輯,`test_vo_fusion_compare.py` 方案 A,永遠信任最新一次匹配) | 161.6m |
| FoundLoc 風格 corrector(DBSCAN 過濾+陀螺輔助鎖定,真實錨點非模擬) | 130.1m |
| FoundLoc-NF(關掉 DBSCAN 過濾的對照組) | 123.8m |
| **VO-primary + 信心分數閘門(`ros2_node_vo_primary.py` 對應邏輯,方案 B)/純 VO** | **36.5m** |

方案 B 在所有測試的 threshold(0.4-0.8)下幾乎完全等同純 VO(接受率
0-1/324)——因為這個資料庫的匹配分數太低,閘門幾乎全部正確拒絕。production
anchor-chain 沒有閘門,直接繼承壞匹配的誤差。FoundLoc corrector 用
DBSCAN 過濾+反覆核對後選擇性接受 41 個錨點中的 24-31 個,但接受的錨點還是
拉歪了 scale 估計(推到 2.2-2.5 倍)——DBSCAN 這次沒有像 §14-19 模擬雜訊
測試那樣「賺回它的價值」(過濾版 130.1m 反而比不過濾版 123.8m 差一點),
因為這裡的問題不是少數離群的假陽性,是**普遍性偏弱**的匹配品質,不是
DBSCAN 設計要抓的那種病灶。

**結論**:錨點品質是這一整條鏈的決定性瓶頸。錨點夠爛時,融合邏輯設計得
再精巧(FoundLoc 的兩個機制、production 的閘門)都無法把雜訊轉成訊號——
唯一穩贏的策略反而是最簡單的(閘門正確識別出「這批錨點不值得信任」然後
退回純 VO)。100m AGL 上單純 VO(36.5m,~108s 窗口)表現不錯,呼應
§14-35/14-48 的「高 AGL 對 VIO/VO 較有利」發現。**下一步(尚未做)**:
若要讓這台新相機的同域資料庫恢復到舊相機的水準,最值得先查的是重新用
「同一天、間隔更短」的建圖飛行建庫(排除 49 分鐘光影落差這個變因),
而不是急著調 FoundLoc 的參數。

**檔案**:`anyloc/database_survey40_samedomain_vits14/`(新資料庫)、
`anyloc/logs/survey42_samedomain_vo_fusion.json`(方案 A/B/VO 比較)、
`field_data/survey42/vio_eval/foundloc_corrector_survey42.py`(真實錨點版
FoundLoc corrector,仿 `field_data/survey32/vio_eval/foundloc_corrector_survey32.py`)、
`field_data/survey42/vio_eval/vio_full_survey42_foundloc.csv`(修正後軌跡)。
