# VIO(OpenVINS)測試資料收集程序

> 日期:2026-07-18
> 目的:讓每次 survey 飛行錄到的資料,足以離線跑完整 OpenVINS 流程
> (標定 → VIO → 與 GPS 真值比對),再決定是否實機整合。
> 背景:SITL 誤差模型實驗顯示 VIO 等級里程計可把定位誤差從 ~15-40 m
> 壓到 ~1-4 m(見 [vpe_jump_runaway_diagnosis.md](vpe_jump_runaway_diagnosis.md) §10)。
> survey13 以前的錄影**完全沒有 IMU**,無法餵 OpenVINS——本程序補齊缺口。

---

## 1. 錄影機新增了什麼(tools/record_field.py,2026-07-18)

| 檔案 | 內容 | 用途 |
|---|---|---|
| `imu.csv` | `stamp_ros, recv_unix, wx, wy, wz, ax, ay, az`(FC 陀螺儀 rad/s + 加速度 m/s²) | OpenVINS 的 IMU 輸入 |
| `attitude.csv` | `stamp_ros, recv_unix, qw, qx, qy, qz`(FC 融合姿態) | VIO 初始化/健檢對照 |
| `meta.json` 新欄位 | `frame_rotation_deg=180`、`purpose`、`imu_requested_hz`、結束時實測 `imu_achieved_hz_at_stop` | 事後判讀 |

機制:
- IMU 記錄由**獨立行程** `tools/imu_logger.py` 執行(錄影機自動啟停)。
  這不是可有可無的架構選擇:相機/編碼主迴圈的 Python GIL 會把同行程的
  ROS executor 榨到只剩 ~60–105 Hz(bench 實測),獨立行程才守得住 200 Hz。
  **不要把 IMU 訂閱搬回錄影機行程內。**
- Sidecar 自動向 FC 發 `SET_MESSAGE_INTERVAL`,要求 `RAW_IMU` 200 Hz、
  `ATTITUDE_QUATERNION` 50 Hz;**每 2 秒重發直到實測 ≥80 Hz**
  (FC 重開機會遺失此設定,重發可自癒)。
- 狀態列即時顯示 `imu=XXXHz`(讀 sidecar 的 `imu_rates.json`),
  **低於 80 Hz 會標 ⚠**;結束時若仍 <80 Hz 會印警告並記在 meta.json。
- `stamp_ros` 是 mavros timesync 對映後的時間(與 `frame_times.csv` 的
  Jetson 時鐘同源);`recv_unix` 是到達時間,供交叉檢查 timesync 品質。

## 2. FC 鏈路速率 — ✅ 已驗證(2026-07-18 bench,FC 實機上電)

**實測結果(921600 serial,與 H.265 錄影同時進行):**

| 項目 | 實測值 | 判定 |
|---|---|---|
| RAW_IMU 速率 | **199.8 Hz**(要求 200) | ✅ 滿速 |
| 取樣間隔 | median 5.0 ms;87 秒內 >20 ms 的斷口只有 1 個(t=0,速率請求生效前) | ✅ |
| timesync 時戳 vs 到達時間 | 偏移 median 0.5 ms、抖動(p95−p5)0.4 ms | ✅ 遠優於預期 |
| ATTITUDE_QUATERNION | 50.0 Hz | ✅ |

結論:**MAVLink IMU 路徑完全滿足 OpenVINS 需求**(200 Hz、毫秒級以下
抖動),不需要獨立 IMU 硬體。第 6 節的 rolling shutter 仍是主要品質風險。

**完整管線驗證(同日,錄影+IMU 同跑 30 秒,sidecar 架構,正常關閉):**
900/900 影格、imu.csv 200.0 Hz **零斷口**(>20 ms 的間隔 = 0)、
attitude 50.0 Hz、meta.json 正確寫入實測速率——收集管線可上場。

**啟動暫態(正常現象):** mavros 剛啟動的前 ~15–30 秒,launch script 自己的
stream-rate 請求迴圈會反覆把 RAW_IMU 蓋回低速(實測卡在 50 Hz),
sidecar 每 2 秒重發直到搶回 200 Hz。**起飛前確認狀態列 `imu=200Hz`
(無 ⚠)即可**——起飛前 60 秒靜置本來就涵蓋這段暫態。
另兩個已修的地雷(勿回退):(1) 對 ArduPilot 發一次 SET_MESSAGE_INTERVAL
只會得到 ~50 Hz,要再發第二次才解鎖全速;(2) 兩個 COMMAND_LONG 並發會在
mavros 內互相踩(RAW_IMU 被套成姿態的 50 Hz)——sidecar 已改成序列化發送。

**OpenHD 即時圖傳(2026-07-18 新增):** 錄影機支援 `--stream-openhd [IP]`
(mode C,H.264 RTP/UDP,預設 192.168.2.2:5601,參數與實測可用的
standalone 管線一致),與 MediaMTX relay(mode B)可同時開;
兩路串流都帶相同的疊加資訊列(LAT/LON+時鐘、AGL/HDG+IMU 速率)。
相機只能被一個行程開啟,所以 OpenHD 串流必須走錄影機內部共享畫面,
**不要**在錄影時另外跑 standalone gst-launch 管線。
`~/Desktop/field_data_collection.sh` 已加上此參數;地面站不在線時
純 UDP 發送無害。已 bench 驗證(RTP pt=96、SPS 正常、~4 Mbps,
錄影 900/900 影格不受影響)。

**MAVLink relay + Mission Planner 地雷(2026-07-21 已修):**
`field_data_collection.sh` 現在同時啟動 MAVLink relay(vehicle client +
`MAVLINK_RELAY=1`),讓 MP 可經網際網路看遙測。但 MP 每 ~15 秒會重發
`REQUEST_DATA_STREAM`(msg 66),在共用通道上會把 sidecar 的 RAW_IMU
200 Hz 蓋回 MP 預設的 2 Hz——imu.csv 每次被蓋都出現數秒 2 Hz 的洞,
對 VIO 是災難。修法:`control/mavlink_relay_client.py` vehicle 端
直接丟棄 GCS 方向的 msg 66(log 出現 `dropped GCS msg id 66` = 正常
運作,非錯誤);已對真實 MP 實測,RAW_IMU 全程守住 200 Hz,MP 的
遙測與控制指令不受影響。副作用:MP 在 relay 連線上改不了 stream rate
(rate 由 launch script 與 SRn 參數決定)。若錄影中 IMU 仍掉到 2 Hz,
先查 5760 埠是不是被**舊版** client 佔住(上一場的launcher視窗停在
「Press Enter to close」不按 Enter,EXIT trap 不會執行,舊 client
就一直活著):`ss -tlnp | grep 5760`。

## 3. 每次 survey 飛行的操作(新增步驟以 ★ 標示)

1. 起動 mavros、record_field.py(與現行流程相同;Desktop 的
   `field_data_collection.sh` 不需改,錄影機內部自動處理 IMU)。
2. ★ 確認狀態列 `imu=XXXHz` 無 ⚠(≥80 Hz)再起飛。
3. ★ **起飛前靜置 ≥60 秒**(馬達未解鎖、飛機完全不動):
   給 OpenVINS 靜態初始化 + 陀螺儀 bias 估計用的資料段。
4. 正常飛 survey(GPS 開著 = 真值;與現行做法相同)。
5. ★ 降落後不要立刻關程式,再靜置 ~10 秒才 Ctrl+C。
6. 檢查 meta.json 的 `imu_achieved_hz_at_stop`。

## 4. 標定(Calibration)——每次相機重新安裝後做一次

OpenVINS 需要:相機內參、相機-IMU 外參(旋轉+平移)、時間偏移。
全部用 Kalibr 從一段標定錄影算出:

1. 印 AprilGrid(kalibr 官方 `aprilgrid.pdf`,A1 以上,貼平板上)
   或大棋盤格;量好格距填進 target.yaml。
2. 錄標定段(**手持整機**或懸停在標的前;要「充分激發」:
   各軸平移+旋轉都要動到,60–90 秒,標的保持在畫面內):
   ```bash
   python3 tools/record_field.py --calib --duration 90
   ```
   輸出到 `field_data/calib_<時間>/`(有 video.mkv + imu.csv)。
3. 轉 rosbag + 跑 Kalibr(在 PC 上做;Jetson 碟太滿):
   video.mkv → 影格(`tools/extract_frames.py`,時間用 frame_times.csv)
   + imu.csv → `kalibr_bagcreater` → `kalibr_calibrate_cameras`
   (pinhole-radtan)→ `kalibr_calibrate_imu_camera`。
   IMU noise 參數先用 Pixhawk 級 IMU 通用值
   (σ_g≈1.7e-4, σ_a≈2e-3, σ_bg≈2e-5, σ_ba≈3e-3),之後再精修。
4. 注意:標定必須在**錄好的影像方向**上做——錄影機存檔前已把畫面轉 180°
   (meta.json `frame_rotation_deg: 180`),不要再自己翻轉。

## 5. 離線測試整個流程(拿到第一筆完整資料後)

1. 資料 → rosbag(同上 bagcreater;或用 OpenVINS 的 ROS-free 介面)。
2. OpenVINS `run_serial` 跑該 bag,產出軌跡。
3. 與 telemetry.csv 的 GPS 真值比對(可仿照
   `anyloc/test_vo_fusion_compare.py` 的誤差統計)。
   評估重點:漂移 %(目標 <1% 距離)、初始化是否成功、震動段表現。
4. 若達標,再評估 Jetson 上的即時整合
   (取代 `anyloc/vo_refiner.py`,plan-B 融合邏輯與 slew limiter 不變,
   jump gate 可收緊到 ~10 m + 0.2 m/s,見 SITL §10)。

## 6. 已知限制(誠實清單)

- **IMX219 是 rolling shutter**——OpenVINS 假設 global shutter;
  低速平飛影響有限,劇烈機動段誤差會變大。
- **無硬體同步**:相機時間戳來自 Jetson appsink 讀取時刻
  (含 ISP 管線固定延遲 ~數十 ms),IMU 時間戳經 mavros timesync
  (毫秒級抖動)。固定偏移 Kalibr 能吸收,抖動吸收不了——
  這是 MAVLink IMU 路徑的主要品質風險。
- 錄影中相機斷線重連(dropout)會在 frame_times.csv 留下時間缺口,
  OpenVINS 會在缺口處重置——分析時以缺口切段。
- `--calib` 手持錄影時 FC 必須上電(IMU 來自 FC),整機一起動。
