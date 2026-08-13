# VIO(OpenVINS)測試資料收集程序

> 日期:2026-07-18 / 更新 2026-07-23(Kalibr 標定完成;IMU 串流建議改
> `--imu-hz 333`,桌面 launcher 已帶;離線評估流程已實跑兩輪,
> 結果與下一步見 vpe_jump_runaway_diagnosis.md §14)/ 更新 2026-07-27
> (①`run_video_msckf` 執行檔過期地雷:與 library 沒同步重連結會導致
> heap corruption crash,修法見下方§5;②`--stride 1`(全 30fps)實測
> 反而讓 raw VIO 更差,不是免費的改善,見§5)
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
- Sidecar 自動向 FC 發 `SET_MESSAGE_INTERVAL`,要求 `RAW_IMU`
  `--imu-hz`(預設 200)、`ATTITUDE_QUATERNION` 50 Hz;**每 2 秒重發
  直到實測 ≥ 請求值的 40%**(FC 重開機會遺失此設定,重發可自癒)。
- **2026-07-23 起外場標準是 `--imu-hz 333`**(桌面
  `field_data_collection.sh` 已帶):FC 韌體上限 333 Hz(400 會被拒),
  實際達成 ~346 Hz ≈ 87% 的 loop-rate 樣本,消除 400→200 串流減採樣
  的震動混疊(診斷與量測:vpe_jump_runaway_diagnosis.md §14-3/14-6)。
  狀態列顯示 `imu=346Hz` 左右是**正常值**。
- 狀態列即時顯示 `imu=XXXHz`(讀 sidecar 的 `imu_rates.json`),
  **低於請求值 40% 會標 ⚠**;結束時仍過低會印警告並記在 meta.json。
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

結論:**MAVLink IMU 路徑本身(速率/時戳品質)滿足 OpenVINS 需求**,
不需要獨立 IMU 硬體。但 2026-07-23 用 survey17 頻譜證實:飛行中槳/馬達
震動經串流減採樣**混疊**進資料(靜置乾淨、飛行中地板平坦到 Nyquist)
——這是 VIO 尺度塌縮與爬升發散的主因,修正進行中(`--imu-hz 333` +
FC notch,見 vpe_jump_runaway_diagnosis.md §14)。rolling shutter 次之。

**完整管線驗證(同日,錄影+IMU 同跑 30 秒,sidecar 架構,正常關閉):**
900/900 影格、imu.csv 200.0 Hz **零斷口**(>20 ms 的間隔 = 0)、
attitude 50.0 Hz、meta.json 正確寫入實測速率——收集管線可上場。

**啟動暫態(正常現象):** mavros 剛啟動的前 ~15–30 秒,launch script 自己的
stream-rate 請求迴圈會反覆把 RAW_IMU 蓋回低速(實測卡在 50 Hz),
sidecar 每 2 秒重發直到搶回全速。**起飛前確認狀態列 `imu=346Hz`
(`--imu-hz 333` 時;預設 200 時看 `imu=200Hz`,無 ⚠)即可**——
起飛前 60 秒靜置本來就涵蓋這段暫態。
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
先查 5760 埠是不是被**舊版** client 佔住(launcher 現已在啟動時自動
pkill 舊 client;手動跑 client 時仍要留意):`ss -tlnp | grep 5760`。

**出向流量也要濾(同日第二地雷,已修):** mavros 會把 FC 通道上的
全部訊息鏡射給 gcs bridge——包括 sidecar 專用的 200 Hz RAW_IMU +
50 Hz ATT_QUAT(~29 KB/s TCP),跟 MediaMTX 影像推流搶同一條 LTE
上行,實測把影像串流塞到延遲 5–6 秒(RTT 32 ms→1.4 s)。vehicle
client 現在同時丟棄出向的 msg 27/31(log:`dropped outbound msg id
27`,每 5000 筆才記一次),relay 流量降回 ~11 KB/s、RTT ~50 ms;
MP 的 HUD 用的是 10 Hz ATTITUDE(msg 30),完全不受影響。

## 3. 每次 survey 飛行的操作(新增步驟以 ★ 標示)

1. 起動 mavros、record_field.py(與現行流程相同;Desktop 的
   `field_data_collection.sh` 自動處理 IMU,2026-07-23 起帶
   `--imu-hz 333`)。
2. ★ 確認狀態列 `imu=XXXHz` 無 ⚠(≥ 請求值 40%;333 模式正常顯示
   ~346Hz)再起飛。
3. ★ **起飛前靜置 ≥60 秒**(馬達未解鎖、飛機完全不動):
   給 OpenVINS 靜態初始化 + 陀螺儀 bias 估計用的資料段。
4. 正常飛 survey(GPS 開著 = 真值;與現行做法相同)。
5. ★ 降落後不要立刻關程式,再靜置 ~10 秒才 Ctrl+C。
6. 檢查 meta.json 的 `imu_achieved_hz_at_stop`。

## 4. 標定(Calibration)——每次相機重新安裝後做一次

> ⚠️ **舊機身(IMX219)已完成 2026-07-23,但已失效**:session
> `field_data/calib_20260723_001209`(第 5 次錄製通過)→ PC Kalibr → 結果
> 在該資料夾 `kalibr_output/`(內參 fx/fy 1339.3/1335.9、radtan 畸變、外參
> |t|=0.358 m 已實機確認、時間偏移 −56 ms),曾寫入
> `~/openvins_ws/config/survey17_kalibr[_dyn]`。完整驗收表:
> vpe_jump_runaway_diagnosis.md §14-1。**2026-08-09 相機改回 AP-IMX900
> (USB3,新 4mm CS-mount 鏡頭)後這份標定不再適用**(不同感光元件/鏡頭,
> 內參/畸變全變,外參平移也可能因鏡頭殼體變大而偏移)。
>
> **現在進行中(2026-08-11 晚)**:錄影已完成(`field_data/
> calib_20260811_232301` 為主要 session,122s、trim-head 需設 23s 而非腳本
> 預設的 12.5s;`calib_20260811_232621` 86s 留作備援)、session 專屬
> `KALIBR_PC_README.md` 已寫入該資料夾。下一步是 Frank 拿去 PC 端跑 Kalibr
> Docker(§Phase 3-6,見 memory `ap-imx900-kalibr-calibration-plan`)。
> 下面是**通用程序**,留給這次和之後每次相機重裝後參考用。

OpenVINS 需要:相機內參、相機-IMU 外參(旋轉+平移)、時間偏移。
全部用 Kalibr 從一段標定錄影算出:

1. 印 AprilGrid(kalibr 官方 `aprilgrid.pdf`,A1 以上,貼平板上)
   或大棋盤格;量好格距填進 target.yaml。
2. 錄標定段(**手持整機**或懸停在標的前;要「充分激發」:
   各軸平移+旋轉都要動到,60–90 秒,標的保持在畫面內):
   ```bash
   source control/ros2_env.sh
   python3 tools/record_field.py --calib --duration 90 \
       --stream-server 118.232.160.227
   ```
   輸出到 `field_data/calib_<時間>/`(有 video.mkv + imu.csv)。
   `--calib` 只是標記用途,**不會**自動開串流;要看即時畫面確認標靶
   有在框內,必須自己加 `--stream-server`(瀏覽器開
   http://118.232.160.227:8889/drone)。注意串流畫面是 4:3 中裁成
   16:9,上下比錄影檔少——串流裡看得到標靶,錄影檔一定也有。
3. 轉 rosbag + 跑 Kalibr(在 PC 上做):
   video.mkv → 影格(ffmpeg 全抽 + 按 frame_times.csv 改名奈秒時戳;
   ⚠ 不要用 `tools/extract_frames.py`,那是 AnyLoc DB 建置器,會按
   GPS/AGL 過濾,手持標定段會被濾成 0 張)
   + imu.csv → `kalibr_bagcreater` → `kalibr_calibrate_cameras`
   (pinhole-radtan)→ `kalibr_calibrate_imu_camera`。
   完整逐步版:vpe_jump_runaway_diagnosis.md §13。
   IMU noise 參數先用 Pixhawk 級 IMU 通用值
   (σ_g≈1.7e-4, σ_a≈2e-3, σ_bg≈2e-5, σ_ba≈3e-3),之後再精修。
4. 注意:標定必須在**錄好的影像方向**上做——錄影機存檔前已把畫面轉 180°
   (meta.json `frame_rotation_deg: 180`),不要再自己翻轉。

## 5. 離線測試整個流程 — ✅ 工具鏈已建好、已實跑兩輪

實際流程(不用 rosbag,Jetson 上直接跑):

1. `~/openvins_ws/build-ov/run_video_msckf <config> video.mkv
   frame_times.csv imu.csv out.csv [start_off] [end_off] [stride]`
   (ROS-free 餵料器;設定檔在 `~/openvins_ws/config/`,
   標定後版本 = `survey17_kalibr`(靜態初始化)/`survey17_kalibr_dyn`
   (空中動態初始化);餵入必須從乾淨靜止窗開始;**config 參數要給到
   `estimator_config.yaml` 這個檔案本身,不是資料夾**——給資料夾會被
   `cv::FileStorage::open` 當成無效輸入,丟出乾淨的 exception,不是
   下面那種 heap corruption)。同目錄下另有
   `run_video_msckf_aglprior`(AGL 深度先驗,多一個
   `[gyro_predict_track 0|1]` 參數——用陀螺積分預測 KLT 搜尋種子,
   修正轉彎時追蹤失敗的根因,見 §14-17)、`run_video_msckf_gyrogate`
   /`_gyrogate_soft`(轉彎時硬/軟門控相機更新)。**⚠ 這幾個從不是真的
   CMake target**(裸 .o、無 flags.make/link.txt)——`make` 可能靜默
   no-op,任何新結果先確認 binary timestamp/checksum 再信,見
   §14-13 landmine。

   **⚠ 姊妹地雷(2026-07-27 發現並修正):執行檔與 library 沒同步重連結
   會導致確定性的 heap corruption crash**——`run_video_msckf`(連最基本、
   一直在用的那個,不是新加的變體)在 2026-07-24 23:54 `libov_msckf_lib.so`
   因 gyro-predicted-tracking 改了 class layout 而重建之後,自己卻沒有
   跟著重連結(執行檔仍是 2026-07-22 版本)。症狀是**每次執行都 100% crash**
   (`malloc.c:2617 sysmalloc`、`malloc(): invalid size` 等,依輸入不同
   訊息不同,是 ABI 不匹配的典型特徵)。根因:`run_video_msckf.cpp` 的
   `make_shared<VioManager>(...)` 在編譯時就把 `sizeof(VioManager)` 烤進
   shared_ptr 控制區塊的配置大小,但實際執行的建構子來自後來變大的新
   library——配置空間不夠,第一次深層記憶體配置(這次是 `cv::setNumThreads`
   內部)就把 heap 弄壞(**crash 位置不等於問題根源位置**,查這類 crash
   要往回找)。**光是重連結不夠**(試過,同樣 crash)——`.o` 檔本身要用
   當前 header 重新編譯。修法(避開這個 shell 對 ROS2/`ament_cmake` 過敏、
   `cmake`/`make` 一碰就想重新 configure 失敗的問題):直接從
   `CMakeFiles/<target>.dir/flags.make`(編譯指令)與 `link.txt`(連結
   指令)複製指令手動執行,完全不觸碰 `CMakeCache.txt`、不觸發
   `cmake_check_build_system`。**任何在 2026-07-24 23:54(`libov_msckf_lib.so`
   的 mtime)之前連結、之後沒動過的執行檔都要懷疑同樣的病**——目前查過:
   `run_video_msckf`(已修)、`run_video_msckf_aglprior`(比 library 晚
   1 秒連結,安全)沒問題;`run_video_msckf_gyrogate`/`_gyrogate_soft`
   **仍是舊的,未修**,用之前先照上面的方法重編譯+重連結。

   **`--stride` 不是免費的加速旋鈕(2026-07-27 實測,反直覺結果)**:survey32
   全程資料上把 `stride` 從 2(~15Hz 餵料,這專案一直以來的標準)改成 1
   (全 30fps)實測 raw VIO **明顯變差**(cruise 窗 rmse 75.8→258.4m,
   3.4x),推測是相鄰影格 baseline(視差)減半傷到三角量測品質,抵銷了
   影格間位移變小的好處——這是單目 VIO 常見的取捨,不是 bug。融合修正後
   的最終結果反而 stride=1 略好(69.2→55.6m)但這只是一次飛行的單一
   資料點,不構成「應該全面換成全 30fps」的結論——**維持 stride=2 為
   預設**,除非之後有更多資料支持換。
2. `python3 ~/openvins_ws/compare_vio_gps.py out.csv telemetry.csv`
   (4-DOF 對齊 vs GPS 真值);分段統計/畫圖範本在
   `field_data/survey17/vio_eval/`(`vio_kalibr_stats.py`、
   `vio_path_compare_kalibr.py`、`imu_vibe_spectrum.py`)。
3. 現況(截至 2026-07-25,詳表 vpe_jump_runaway_diagnosis.md
   §11~§14-21):標定不是瓶頸(§14-2)。真正病因鏈:IMU 震動混疊
   (§14-3,notch 修正見 imu_vibration_aliasing_fix 記憶)+
   **OpenVINS KLT 追蹤器在轉彎時無運動預測**(§14-17,搜尋窗 15px vs
   尖峰轉速下位移 ~38px/frame)——後者才是巡航尺度塌縮/轉彎失敗的
   主因,不是單純震動;已修(gyro-predicted KLT,見上)。輸出端尺度
   誤差另有 output-space 修正層(`foundloc_corrector.py`,§14-14~19)
   ,目前最佳驗證過結果 rmse≈78m/max≈164m(模擬 anchor 雜訊)——真實
   AnyLoc 對接後被域差距(satellite vs 空拍實景,§14-20)拖累,不是
   VIO 或融合本身的問題。
4. 達標後才評估 Jetson 即時整合
   (取代 `anyloc/vo_refiner.py`,plan-B 融合邏輯與 slew limiter 不變,
   jump gate 可收緊到 ~10 m + 0.2 m/s,見 SITL §10)。

## 6. 已知限制(誠實清單)

- ~~IMX219 是 rolling shutter~~——2026-08-09 相機改回 AP-IMX900(USB3,
  4mm CS-mount 鏡頭),Sony/Appropho 規格確認為 **global shutter**
  (Pregius S 系列)。
  (歷史記錄:IMX219 使用期間,2026-07-25 直接驗證(§14-16)發現轉彎失敗的
  實際訊號特徵(與轉彎「持續時間」相關,而非「尖峰角速度」)**不符合**
  rolling shutter 的機制特徵——真正主因是 KLT 追蹤器搜尋窗無運動
  預測(§14-17,已修),rolling shutter 本身當時也未證實是主要瓶頸。)
  **2026-08-11 實測嘗試(§14-45)未能驗證這個假設**:survey38/41/42 用
  復原後的 AP-IMX900 實飛,但這個機身+新鏡頭組合從未做過 Kalibr 標定,
  臨時湊的 computed-FOV+沿用舊機身 extrinsics 種子讓 OpenVINS 三趟全部
  災難性發散(RMSE 差 3-4 個數量級,凍結線上細修後更糟)——不是「global
  shutter 沒用」,是這組克難標定不夠格拿來測。**這個假設目前仍是懸而未決,
  不是已排除**;需要先幫這個機身+鏡頭做一次真正的 Kalibr 標定才能重測。
  同日後續追問(§14-46)已排除旋轉觸發、排除掉幀/資料損毀兩個替代解釋,
  嫌疑縮小到沿用的外參平移(lever arm)或內參本身量測不準。標定分階段
  計畫已寫好,Phase 0-2(PC 端操作說明 + 擷取腳本)已備妥、**Phase 1 錄影
  已完成**(2026-08-11 晚,`field_data/calib_20260811_232301` 為主要
  session)、session 專屬 `KALIBR_PC_README.md` 已寫入該資料夾——下一步是
  Frank 拿去 PC 端跑 Kalibr Docker(Phase 3-6),詳見 memory
  `ap-imx900-kalibr-calibration-plan` 與本文件第 4 節。
- **無硬體同步**:相機時間戳來自 Jetson appsink 讀取時刻
  (含 ISP 管線固定延遲 ~數十 ms),IMU 時間戳經 mavros timesync
  (毫秒級抖動)。固定偏移 Kalibr 能吸收,抖動吸收不了——
  這是 MAVLink IMU 路徑的主要品質風險。
- 錄影中相機斷線重連(dropout)會在 frame_times.csv 留下時間缺口,
  OpenVINS 會在缺口處重置——分析時以缺口切段。
- `--calib` 手持錄影時 FC 必須上電(IMU 來自 FC),整機一起動。
- **震動混疊(2026-07-23 實證,修正中)**:飛行中槳/馬達諧波混疊進
  IMU 串流,是 VIO 尺度/爬升問題的成因之一——不是「路徑」問題
  (靜置頻譜乾淨),是取樣鏈問題;`--imu-hz 333` + FC notch 修正中,
  見 vpe_jump_runaway_diagnosis.md §14。**另一個獨立成因**(2026-07-25
  §14-17 找到並已修正):OpenVINS KLT 追蹤器轉彎時無運動預測,搜尋窗
  15px vs 尖峰轉速位移 ~38px/frame——gyro-predicted KLT 修正後全程
  rmse 7703→2722m(仍未完全收斂,爬升/下降段殘留發散待查)。
