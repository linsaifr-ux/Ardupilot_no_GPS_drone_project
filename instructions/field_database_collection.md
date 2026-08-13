# Field AnyLoc Database Collection

Build a real-imagery AnyLoc database by flying a grid survey, recording video + telemetry, then running the offline pipeline.

**Hardware:** Jetson Orin NX · AP-IMX900-Mini-USB3-I5 camera (USB3, custom `usb_camera_node.py`, MJPG; device auto-detected by USB name, usually `/dev/video0`), 4mm CS-mount lens · ArduPilot FC  
**Scripts:** `tools/record_field.py` · `tools/extract_frames.py` · `anyloc/build_database_real.py`

---

## Why a real database

The default `build_database.py` generates synthetic crops from NLSC satellite tiles.  
A real-imagery database uses the actual camera, actual lighting, and actual terrain texture — closer to what the drone sees at inference time.

**This is not a theoretical nicety — it's the dominant real-world accuracy bottleneck.** A controlled comparison on survey25 (identical VIO, identical AnyLoc/VLAD/DINOv2 code, only the reference database source changed) found ~286m mean retrieval error against the satellite-tile database vs. ~8.6m against a same-domain (real-footage) database — a 33x difference from swapping only the reference imagery. Resolution/clarity was separately ruled out as the cause (a systematic sharpness-vs-error test found essentially no correlation) — it's specifically the season/lighting/content mismatch between satellite tiles and a live drone photo. Details: `instructions/vpe_jump_runaway_diagnosis.md` §14-20/§14-23/§14-25.

That test split one flight's own frames into database vs. query (same day, same lighting) — it does not directly tell you how much accuracy holds up when the database comes from a separate mapping flight flown before the actual mission (season/lighting drift between sessions). Two things matter when planning a real collection flight, not just "fly some frames somewhere":

1. **Cover the whole mission/contest area with real margin, not just a line along the planned route.** A closed-loop SITL test found this pipeline does not self-correct once it drifts outside the database's covered area — a narrow-corridor database leaves zero safety margin if the vehicle strays off-track (`instructions/vpe_jump_runaway_diagnosis.md` §14-24).
2. **Fly a second, separate validation flight afterward** and query the resulting database with it, to measure the real cross-session accuracy directly instead of assuming it matches the same-day 8.6m number.

> **Both now done (2026-07-27), full writeup `field_data/survey32/vio_eval/README.md`:** a mapping flight (survey33, diagonal multi-leg lawnmower grid, ~206×342m coverage) followed ~15 min later by a separate test flight (survey32, nested inside that coverage). Cross-session retrieval accuracy: **~24-25 m mean** — worse than the same-flight 8.6m (real cross-session degradation, as expected) but still ~12x better than the satellite-tile database. Fed through a real closed-loop AUTO-mode SITL test with the route kept inside coverage: **zero EKF glitch(>50m) events** — the first clean result of that kind this project has produced, directly attributable to point 1 above (staying inside database coverage). This validates the two-point plan above; it does not mean either point can be skipped on a future collection — the SITL result specifically depended on point 1 being followed.

---

## How recording works

`record_field.py` records **three streams in parallel**:

| Stream | Transport | Source |
|---|---|---|
| Video | OpenCV/V4L2 capture (MJPG) → GStreamer H.265 encode, not through ROS | H.265, 2048×1536 30fps, ~60 MB/min (8 Mbps default) |
| Telemetry | ROS2 subscriptions | lat/lon/AGL/heading + RC channels at 5 Hz → CSV |
| IMU (VIO) | `tools/imu_logger.py` **sidecar process** (auto-started; must stay a separate process — the camera loop's GIL starves an in-process subscriber) | FC gyro+accel at `--imu-hz` (200 default; field standard 333 → ~346 Hz actual since 2026-07-23) + attitude at 50 Hz → CSV (see `instructions/vio_data_collection.md`) |

Video goes **directly through V4L2, not through ROS**. This means:

- `launch_camera.sh` must **not** be running during collection — it also opens `/dev/video0` and only one process can hold the device at a time
- AnyLoc and YOLO nodes must also be stopped for the same reason
- Only MAVROS is needed (it doesn't touch the camera)

**Optional second camera (`--imx219`, added 2026-08-13):** also records the IMX219 CSI camera concurrently, as a fully independent GStreamer pipeline (`nvarguscamerasrc` → HW H.265 encode → `video_imx219.mkv`) that never touches the primary capture/encode loop, so it can't stall the main recording. The two cameras sit on separate buses (USB3 vs CSI/MIPI) with separate HW encode sessions — bench-tested running both concurrently at full res/fps for 20s with 0 dropped frames on either side and CPU/GPU/thermal headroom to spare. If the IMX219 isn't connected or another process (e.g. `csi_camera_node.py`) already holds it, it degrades to primary-only recording with a warning instead of failing the whole flight capture. The Desktop launcher (below) passes `--imx219` by default.

---

## Camera FOV at 65 m AGL

The camera's 4mm CS-mount lens gives **HFOV≈59.9° × VFOV≈46.7°** at native 4:3 —
computed from sensor geometry (2048×1536 native, 2.25µm px), not a datasheet
spec, since none is available for this lens. Recording (`record_field.py`)
publishes at **2048×1536** (native); live inference (`launch_camera.sh` via
`usb_camera_node.py`, MJPG) publishes at **1280×960** — do not switch to a 16:9 mode
like 1280×720, which crops the sensor vertically and reduces VFOV.

| | Ground width | Ground height | GSD |
|---|---|---|---|
| 2048×1536 (recording) | 74.9 m | 56.1 m | 3.66 cm/px |
| 1280×960 (live inference) | 74.9 m | 56.1 m | 5.85 cm/px |

---

## Flight plan

- **Pattern:** lawnmower (boustrophedon) grid
- **Altitude:** 65 m AGL constant (matches operational mission altitude)
- **Strip direction:** E-W (long side ~1 743 m), advancing N-S — generated by `tools/gen_survey_waypoints.py`
- **Strip spacing:** 37 m → 50% side overlap (28 strips over 2 091 m × 1 025 m expanded area)
- **Ground speed:** ≤ 3 m/s (reduces motion blur)
- **Heading:** any — `extract_frames.py --rotate` corrects to North-up in post
- **Area:** full operational zone + 20% margin
- **Lighting:** fly at the same time of day as planned GPS-denied missions (shadows are strong VLAD features and shift between morning and afternoon)

**Estimated storage:** ~60 MB/min → ≈ 1.2 GB for a 20-minute survey  
**Estimated flight:** ~331 min total at 3 m/s (~17 batteries @ 20 min each) — use `--split N` to divide into sub-missions

Generate waypoints:
```bash
python3 tools/gen_survey_waypoints.py                # full mission (survey_mission_full.waypoints)
python3 tools/gen_survey_waypoints.py --split 4      # 4 N-S sub-missions
python3 tools/gen_survey_waypoints.py --spacing 21   # 21 m spacing (72 % sidelap, denser)
```

The defaults above target the contest zone's hardcoded `CORNERS` at 65 m AGL. For a collection flight at a **different site** (e.g. validating against a specific test flight instead of the contest zone), override the box/altitude/speed directly rather than editing the constants — all pure additions, contest-zone defaults unchanged if omitted:
```bash
python3 tools/gen_survey_waypoints.py \
    --center-lat 22.777521 --center-lon 120.550655 \
    --width-m 850 --height-m 850 \
    --altitude 100 --speed 10 \
    --name survey25_dbcollect --outdir field_data/survey25
```
`--width-m`/`--height-m` are the box size *before* the standard 20% margin expansion. `--speed` also feeds the printed time/battery estimate — note the 3 m/s default exists specifically to limit motion blur; faster speeds trade that off, so spot-check frame sharpness after the flight before trusting the resulting database. `--spacing`/`--altitude` interact (spacing auto-recomputes for 50% sidelap at the given altitude unless `--spacing` is also given explicitly).

### Overlap guide

Spacing values are for the AP-IMX900 4mm lens's ~74.9 m footprint width at 65 m AGL — recompute with `gen_survey_waypoints.py`'s `FOOTPRINT_W_M` if altitude or camera changes (or just pass `--altitude`, which does this automatically).

| Strip spacing | Side overlap | |
|---|---|---|
| 21 m | 72% | optimal |
| 37 m | 50% | **current default** |
| 45 m | 40% | too low |

| Frame spacing (`--min-dist`) | Forward overlap | |
|---|---|---|
| 25 m | 70% | optimal |
| 30 m | 64% | acceptable |
| 50 m | 40% | too low |

---

## Terminal setup during collection flight

```
Terminal 1   bash control/launch_mavros_real.sh
Terminal 2   source /opt/ros/humble/setup.bash && python3 tools/record_field.py --output field_data/survey1 --stream-host <GS_IP> --imu-hz 333
             # or stream via MediaMTX relay server (no GStreamer needed on ground station):
             source /opt/ros/humble/setup.bash && python3 tools/record_field.py --output field_data/survey1 --stream-server 118.232.160.227 --imu-hz 333
```

The recorder reads GPS, AGL, and heading **directly from MAVROS** (`/mavros/global_position/global`, `/mavros/global_position/rel_alt`, `/mavros/global_position/compass_hdg`) — `hw_bridge.py` is not needed for collection.

The Desktop `~/Desktop/field_data_collection.sh` launcher wraps all of this
(auto-numbered `field_data/surveyN`, MediaMTX + OpenHD streams) and since
2026-07-21 also starts the MAVLink relay (`mavlink_relay_client.py --role
vehicle` + `MAVLINK_RELAY=1` mavros) so Mission Planner can watch telemetry
over the internet during the flight — see `streaming/mavlink_relay_setup.md`
(the relay client filters MP's stream-rate stomps and the outbound
high-rate IMU mirror, so neither the IMU recording nor the video stream's
LTE bandwidth is affected by MP being connected). Since 2026-07-23 the
launcher also passes `--imu-hz 333` to the recorder, and since 2026-08-13
also `--imx219` (records the IMX219 CSI camera alongside the primary
AP-IMX900, see above). Close the launcher window (press Enter at its final
prompt) when done; the launcher also reaps any stale relay client from a
previous session at startup.

Do **not** run `launch_camera.sh`, `anyloc/ros2_node.py`, or `detection/ros2_node.py` — they all compete for the same camera device.

---

## Step 1 — Record

All frames are **rotated 180°** after capture before recording and streaming.

```bash
source /opt/ros/humble/setup.bash
# --imu-hz 333 on all variants = field standard since 2026-07-23 (VIO datasets)

# Without stream
python3 tools/record_field.py --output field_data/survey1 --imu-hz 333

# Mode A — live preview streamed directly to ground station (UDP, requires GStreamer on GS)
python3 tools/record_field.py --output field_data/survey1 --stream-host <GS_IP> --imu-hz 333

# Mode B — live preview pushed to MediaMTX relay server (RTSP, watch in VLC or browser)
python3 tools/record_field.py --output field_data/survey1 --stream-server 118.232.160.227 --imu-hz 333
```

**Mode A — receive on ground station:**
```bash
gst-launch-1.0 udpsrc port=5000 ! \
  application/x-rtp,encoding-name=H265,payload=96 ! \
  rtph265depay ! h265parse ! avdec_h265 ! \
  videoconvert ! autovideosink sync=false
```

**Mode B — watch from MediaMTX relay (no setup needed on ground station):**
```
VLC:     rtsp://118.232.160.227:8554/drone
Browser: http://118.232.160.227:8889/drone  (WebRTC, ~200 ms)
Browser: http://118.232.160.227:8888/drone  (HLS, ~5 s, mobile)
```

**Mode C — OpenHD (H.264 RTP/UDP to 192.168.2.2:5601, coexists with A/B):**
the OpenHD ground station picks the stream up directly; the encoder settings
match the field-tested standalone pipeline. Never run a separate capture of
`/dev/video0` while recording — only one process can hold the device at a
time; mode C shares the recorder's capture instead.

Options:

| Flag | Default | Description |
|---|---|---|
| `--stream-host` | off | Ground station IP — direct UDP stream (mode A) |
| `--stream-port` | 5000 | UDP port (mode A) |
| `--stream-server` | off | MediaMTX relay server IP — RTSP push (mode B) |
| `--stream-rtsp-path` | `/drone` | RTSP path (mode B) |
| `--stream-bitrate` | 1000000 | H.265 stream bitrate in bps (modes A/B) |
| `--stream-openhd [IP]` | off (IP defaults to 192.168.2.2) | OpenHD ground station — H.264 RTP/UDP (mode C, runs **alongside** A or B; same overlay view) |
| `--openhd-port` | 5601 | UDP port (mode C) |
| `--openhd-bitrate` | 4000000 | H.264 bitrate in bps (mode C) |
| `--bitrate` | 8000000 | H.265 recording bitrate in bps |
| `--duration` | 0 | Stop after N seconds (0 = Ctrl+C) |
| `--calib` | off | Tag as camera-IMU calibration session (`field_data/calib_<ts>/`) |
| `--imu-hz` | 200 | RAW_IMU stream rate. Field standard since 2026-07-23: **333** (~346 Hz actual, firmware's grantable max — removes stream-decimation vibration aliasing, `vpe_jump_runaway_diagnosis.md` §14-6). The Desktop launcher passes it. |

All stream views carry the same telemetry overlay bar: `LAT LON clock` /
`AGL HDG IMU-rate`.

Live status printed to terminal:
```
[REC]    42s  lat=23.451234  lon=120.287654  agl=65.2 m  hdg=045°  imu=200Hz
```
`imu=` shows the FC IMU stream rate; a `⚠` marks <40% of the requested rate
(insufficient for VIO). Expect ~15–30 s at 50 Hz right after mavros starts
before it locks at full rate — wait for `imu=346Hz` (with `--imu-hz 333`;
`imu=200Hz` at the default) before takeoff. If the rate keeps dipping to
~2 Hz mid-recording while Mission Planner is connected via the MAVLink
relay, an outdated relay client is in the path — since 2026-07-21 the
vehicle client filters MP's stream-rate stomps (msg 66); see the
troubleshooting table in `streaming/mavlink_relay_setup.md`. (Details in
`instructions/vio_data_collection.md`).

> **Known issue:** if GPS fix hasn't been acquired yet, status shows `waiting for GPS …` instead. Recording still runs — video and non-GPS telemetry columns (AGL, heading) still write to CSV. Wait for GPS lock before starting the collection flight, or accept that lat/lon columns will be empty for the first few seconds.

Press **Ctrl+C** to stop.

Output in `field_data/survey1/`:
```
video.mkv         H.265, 2048×1536 30fps (MKV — stays playable after power-off)
telemetry.csv     unix_time, lat, lon, alt_amsl, alt_agl, heading_deg, rc_channels  (5 Hz)
                  rc_channels = raw /mavros/rc/in PWM list — check the EKF-source
                  switch (RCx_OPTION=90) stayed LOW (GPS) throughout if this
                  recording needs to be trusted as GPS ground truth
meta.json         video_start_unix, fps, width, height, frame_rotation_deg=180,
                  purpose, imu_requested_hz + achieved IMU/attitude rates at stop
frame_times.csv   frame_idx, unix_time — actual capture time per frame, logged
                  directly so it stays correct across camera dropouts/reconnects
                  (video_start_unix + frame_idx/fps assumes constant fps and drifts
                  after a dropout — prefer this file for timing-sensitive work)
imu.csv           stamp_ros, recv_unix, wx, wy, wz, ax, ay, az — FC IMU at the
                  requested rate (200 default / ~346 actual with --imu-hz 333;
                  written by the imu_logger.py sidecar; VIO input)
attitude.csv      stamp_ros, recv_unix, qw, qx, qy, qz — fused FC attitude, 50 Hz
                  (column meanings, quaternion math, roll/pitch/yaw conversion:
                  field_data/attitude_format.md)
imu_rates.json    live 2 s rate report from the sidecar
```

With `--imx219` (Desktop launcher default since 2026-08-13), also:
```
video_imx219.mkv         H.265, 1640×1232 30fps — IMX219 CSI camera
frame_times_imx219.csv   frame_idx, unix_time — same convention as frame_times.csv,
                          for the IMX219 stream. telemetry.csv/imu.csv/attitude.csv
                          are per-flight, not per-camera — shared with the primary
                          recording above.
```

---

## Step 2 — Extract frames

```bash
python3 tools/extract_frames.py field_data/survey1/ --rotate --min-dist 25
```

Options:

| Flag | Default | Description |
|---|---|---|
| `--min-dist` | 30 m | Minimum ground distance between saved frames (25 m recommended) |
| `--min-agl` | 50 m | Skip frames below this AGL |
| `--max-agl` | 1000 m | Skip frames above this AGL |
| `--max-time-gap` | off | Also save a frame after this many seconds even if `--min-dist` wasn't reached — needed if the flight includes a stationary loiter/hold, which produces zero ground-distance travel and would otherwise leave a coverage gap in the database |
| `--rotate` | off | Rotate each frame to North-up using heading (sign fixed 2026-07-25 — was rotating the wrong way; verified via a live matching-accuracy A/B on survey25, see `instructions/vpe_jump_runaway_diagnosis.md` §14-23) |

Output in `field_data/survey1/`:
```
frames/000000.jpg, 000001.jpg, …   geo-tagged frames
frames.csv                          path, lat, lon, alt_amsl, alt_agl, heading_deg
```

---

## Step 3 — Build the database

```bash
/home/jetson/venv/anyloc/bin/python3 anyloc/build_database_real.py field_data/survey1/
```

Options:

| Flag | Default | Description |
|---|---|---|
| `--model` | `vits14` | DINOv2 backbone — must match the inference node |
| `--db-dir` | `anyloc/database_real` | Output directory |
| `--rebuild` | off | Overwrite existing database |

Output in `anyloc/database_real/`:
```
database.pt         split-format pointer (identical layout to build_database.py output)
database_meta.pt    lats, lons, alts (AGL), codebook, model_name
database_vlads.pt   (N × D) VLAD matrix
db_images/          640×480 thumbnails for localizer visualisation fallback
```

---

## Step 3.5 — Visually check coverage (optional, recommended, added 2026-07-27)

```bash
/home/jetson/venv/anyloc/bin/python3 tools/build_frame_mosaic.py field_data/survey1/
```

Stitches the extracted frames into a georeferenced visual mosaic (`mosaic.png` + `.tif` +
world file, feathered blending at frame edges) so you can see what area actually got mapped
before trusting the database — no GDAL needed (`tools/png_to_geotiff.py` writes the `.tif` via
`tifffile`). Not real photogrammetric orthorectification — GPS/heading-placed frames only, so
treat pixel positions as approximate.

---

## Step 4 — Activate

```bash
ln -sfn database_real anyloc/database
```

Verify:
```bash
ls -la anyloc/database
/home/jetson/venv/anyloc/bin/python3 -c "
import torch
db = torch.load('anyloc/database/database.pt', weights_only=False)
meta = torch.load(db['meta'], weights_only=False)
vlads = torch.load(db['vlads'], weights_only=False)
print(f'entries={vlads.shape[0]}  vlad_dim={vlads.shape[1]}  model={meta[\"model_name\"]}')
"
```

---

## Switching back to satellite database

```bash
ln -sfn database_zone_z20_vits14 anyloc/database   # active — zone-sized, zoom-20 (882 entries)
# ln -sfn database_zone_vits14 anyloc/database   # zoom-18 predecessor (same grid)
# ln -sfn database_vits14 anyloc/database        # old full-radius fallback (2821 entries)
```

---

## Files summary

| File | Purpose |
|---|---|
| `tools/record_field.py` | Record video + telemetry + high-rate IMU (200/333 Hz) during the survey flight |
| `tools/imu_logger.py` | IMU sidecar (auto-spawned by record_field.py — never run the subscription in-process) |
| `tools/extract_frames.py` | Extract geo-tagged frames from the recording |
| `anyloc/build_database_real.py` | Build AnyLoc database from extracted frames |
| `control/launch_camera.sh` | **Do not run during collection** — conflicts with recorder |
| `field_data/<session>/video.mkv` | Raw recording (keep until DB verified) |
| `field_data/<session>/frames.csv` | Frame index with GPS + heading |
| `anyloc/database_real/` | Ready-to-use database (activate with symlink) |
