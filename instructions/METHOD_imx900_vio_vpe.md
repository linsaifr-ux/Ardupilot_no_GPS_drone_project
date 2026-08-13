# Method: GPS-denied localization on the imx900 rig (VIO + VPE + fusion)

Complete, self-contained description of the pipeline as it stands on
2026-08-13, written so it can be reproduced or ported to the Jetson without
reading the session logs. Every constant here was measured from flight data;
where a number is uncertain or second-hand that is stated.

Results and the experiments that produced them are in
`session_2026-08-12_imx900_vio.md`. This file is the *method*.

---

## 1. What the pipeline does

```
video.mkv ─┬─> [VIO]  OpenVINS MSCKF ──────> relative pose, 15 Hz, drifts
imu.csv  ──┘                                        │
                                                    ├─> [FUSION] 4-state KF ─> position
video.mkv ───> [VPE]  SuperPoint+LightGlue ──> absolute fix, ~1 Hz
               against a frame map built                │
               from an earlier flight ──────────────────┘
```

VIO is smooth but drifts and has a scale error. VPE is absolute but sparse and
heavy-tailed. The fusion filter reconciles them and is the only thing the
autopilot would ever see.

**Current honest status:** VIO is usable at 10 m AGL (3.9 m rmse) and unusable
at 65 m (diverges). VPE is excellent at 65 m (4.4 m, 100% fix rate) and
mediocre at 10 m against a 65 m map (63% fix rate). Fusion is bounded in both
regimes but does not beat the better source in either. See §10.

---

## 2. Data contract

Everything downstream assumes the logger produces these five files per flight,
all on one clock (unix seconds):

| file | columns | rate |
|---|---|---|
| `video.mkv` | H.264, 2048x1536 | 30 fps |
| `frame_times.csv` | `frame_idx,unix_time` | per frame |
| `imu.csv` | `stamp_ros,recv_unix,wx,wy,wz,ax,ay,az` | ~340 Hz |
| `attitude.csv` | `stamp_ros,recv_unix,qw,qx,qy,qz` | 50 Hz |
| `telemetry.csv` | `unix_time,lat,lon,alt_amsl,alt_agl,heading_deg,rc_channels` | ~5 Hz |
| `meta.json` | `video_start_unix`, `fps`, `width`, `height`, ... | once |

`frame_times.csv` must have exactly one row per decoded video frame — the map
builder and the VIO runner both index frames positionally against it.

**`meta.json`'s `frame_rotation_deg: 180` is wrong for this camera.** It
describes the older 1640x1232 rig. Ignore it; see §3.

Telemetry is used for map building and scoring only. Nothing in the live path
requires GPS.

---

## 3. Frames and conventions

Getting these wrong is the single most expensive class of mistake in this
project, so each is stated with how it was verified rather than asserted.

**IMU is FLU** — x forward, y left, z up. Verified: `az` reads +9.78 m/s² at
rest, and roll/pitch/yaw rates from `attitude.csv` correlate +0.79 / +0.86 /
+0.99 with `wx` / `wy` / `wz`.

**Camera is canonical nadir** — image-up is aircraft-forward, image-right is
aircraft-right, optical axis straight down. Verified two independent ways:

- Wide-baseline pairs: body-forward motion lands at **+91.5°** in the image
  with 0.98 angular concentration over 40 pairs (`calib_focal_widebaseline.py`).
- Gyro sign: image rotation between frames has the *same* sign as `wz`,
  correlation 0.990. A downward-looking camera flips the sign twice; an
  up-looking or mirrored one would not (`calib_cam_imu_timeshift.py`).

So in the camera optical frame (x = image right, y = image down, z = out of
lens):

```
x_cam = aircraft right = -y_imu
y_cam = aircraft back  = -x_imu
z_cam = down           = -z_imu

R_CtoI = [[ 0, -1,  0],
          [-1,  0,  0],
          [ 0,  0, -1]]      det = +1
```

**ENU origin** is fixed in `imx900_geo_common.py` at 22.574900 N, 120.550600 E
— survey44's first airborne fix, hardcoded so survey43/44/45 share one frame.
This is **not** survey33's origin (22.7775 N); the two sites are ~22 km apart
and positions are not interchangeable.

**Map de-rotation**: tiles are rotated by `-heading_deg` to become north-up.
The sign is verified by `imx900_frame_map.py --check-rotation`, which matches
frames taken within 12 m of each other but 80–180° apart in heading; correct
convention gives a median residual rotation of **0.9°**, a flipped sign would
give roughly twice the heading difference.

---

## 4. Camera calibration

There is no Kalibr calibration for this camera. All of it was recovered from
flight data. Values live in
`openvins_offline/config_imx900/kalibr_imucam_chain.yaml` and
`imx900_geo_common.py` — keep the two in step.

| parameter | value | source |
|---|---|---|
| focal (2048x1536) | **1873.6 / 1870.7 px** | Kalibr `calib_20260812_124944` cam-only, ±4.4 px |
| focal (1024x768 feed) | 936.79 / 935.33 | half the above |
| principal point (1024x768) | 515.87, 354.97 | same |
| distortion (radtan) | **-0.4808, 0.2194, 0.00137, -0.00242** | same |
| `T_imu_cam` (R_CtoI, p_CinI) | Kalibr `calib_20260811_232301` | see below |
| p_CinI | (-0.031, 0.067, -0.248) m | same — a real 25 cm lever arm |
| cam→IMU timeshift | **-0.06295 s** | same |
| GSD constant | **k = 1/1873.6 = 5.337e-4** m/px per m AGL | from focal |

**Adopted 2026-08-13, replacing the flight-derived + online-self-calibrated
values.** Measured on survey43 (10 m, full flight, identical window; runs are
bit-for-bit deterministic, verified over 3 repeats):

| calibration | rmse_xy | max_xy | rmse_z |
|---|---|---|---|
| flight-derived + OpenVINS online self-calib | 3.40 m | 6.51 m | 1.94 m |
| Kalibr `calib_20260811_232301` as-is | 3.82 m | 6.47 m | 0.80 m |
| **Kalibr hybrid (the table above)** | **1.07 m** | **1.97 m** | 0.91 m |

### 4.0 Why the pieces come from two different Kalibr sessions

There are five Kalibr sessions. Two things make picking one non-obvious.

**The camera model drifts across the day** — 1891.3 (Aug 11 23:23) → 1883.3
(23:26) → 1867.5 (Aug 12 12:23) → 1873.6 (12:49), with OpenVINS converging to
1790 on the 17:22 flight. So take the solve closest in time with the tightest
uncertainty: `calib_20260812_124944`, ±4.4 px, ~4.5 h before the flights.

**Three of the five imu-cam chains are corrupt.** `calib_20260812_122311` and
`calib_20260812_124944` both carry **1626.3 px** in their
`camchain-imucam.yaml`, which is the camera model from the bad
`calib_20260812_112821` run (±14.5 px, k1 -0.3695). That stale camchain was fed
into their imu-cam solve, so their extrinsics and timeshifts were fitted
against a 13%-wrong focal and **must not be used**. Always cross-check each
session's `calib_4hz-camchain.yaml` against its `calib_full-camchain-imucam.yaml`
before trusting the latter:

```bash
python3 -c "import yaml;print(yaml.safe_load(open('calib_4hz-camchain.yaml'))['cam0']['intrinsics'][0],
                              yaml.safe_load(open('calib_full-camchain-imucam.yaml'))['cam0']['intrinsics'][0])"
```

`calib_20260811_232301` is the only self-consistent one (1891.3 in both), which
is why the extrinsic and timeshift come from it while the camera model comes
from the later session. Its reprojection error is also the best of the three
imu-cam solves (1.358 px vs 1.729 and 1.655).

**Two independent checks that the Kalibr numbers are sane:**

- Kalibr's `R_CtoI` is within **1.51°** of the mount rotation measured from the
  flights themselves (+91.5°, 0.98 angular concentration).
- The timeshift **sign** is confirmed by flipping it: +0.0629 gives 1412 m rmse
  against 3.8 m for -0.0629. Kalibr's convention (`t_imu = t_cam + shift`)
  matches OpenVINS' `calib_camimu_dt` directly.

Kalibr's -0.063 s also agrees with the gyro cross-correlation measured on the
flight (-0.050 s) and with the two other sessions (-0.0655, -0.0620). OpenVINS'
online estimate of -0.0184 s is the outlier.

### 4.1 How to calibrate a new camera from flights

**Do this on the ground with an AprilGrid if you possibly can** — that is now
settled, not advice: Kalibr beat the flight-derived route 1.07 m to 3.40 m.
The procedure below is the fallback for when no calibration exists, and it left
distortion unmeasured, which cost 3437 m of VIO error.

1. **Focal length — wide baseline only.**
   ```
   python3 calib_focal_widebaseline.py --survey <mapflight> --min-sep 30 --max-sep 60
   ```
   Matches frame pairs 30–60 m apart at matching heading and AGL, where GPS
   noise is under 1% of the baseline, and reports `f = |pixel shift| * AGL /
   |ground shift|`. **Validate by checking it is flat against baseline length**
   (measured 1799 / 1891 / 1674 px across the 15–25, 25–40, 40–60 m bins). A
   fixed pixel bias trends with separation; a real focal length does not.

   The same run reports the **mount angle** and its angular concentration.
   Anything below ~0.9 concentration means the pairs are not clean.

2. **Time offset.**
   ```
   python3 calib_cam_imu_timeshift.py --survey <flight> --t0 <cruise start> --t1 <end>
   ```
   Cross-correlates image yaw rate against gyro `wz`. Expect correlation >0.98;
   the peak location is the lag. The curve is flat near the peak, so treat this
   as a sign-and-order-of-magnitude check, not a precise value.

3. **Distortion — you cannot get this from the flight geometry.** Run VIO once
   with `calib_cam_intrinsics: true` on a low-altitude flight and read the
   converged `cam0 intrinsics` line out of the log, then bake it in and turn
   online calibration back off. This is what produced the k1,k2 above, and it
   is the weakest link in the whole pipeline.

**Do not use** `calib_focal_lsq.py`, `calib_focal_track.py` or
`calib_focal_from_flight.py`. They are kept only as documented negative
results: over a fraction of a second, tilt change and translation sweep the
image by comparable amounts and are correlated, so short-baseline fits cannot
separate them. They read 2500–2800 px against the true 1790.

---

## 5. VIO

`openvins_offline/` — a standalone C++ runner that links the built OpenVINS
libraries directly, with no ROS node and no rosbag.

### 5.1 Build

```bash
source /opt/ros/jazzy/setup.bash     # needed: libov_msckf carries DT_NEEDED on rclcpp/tf2
cd survey25/vio_eval/openvins_offline
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release && cmake --build build -j8
```

### 5.2 Run

```bash
./build/run_video_msckf config_imx900/estimator_config.yaml \
    <flight>/video.mkv <flight>/frame_times.csv <flight>/imu.csv \
    out.csv <start_off_s> <end_off_s> <frame_stride>
```

Offsets are seconds relative to the first video frame. Stride 2 gives a 15 Hz
feed, matching `track_frequency: 15.0`. Output columns:
`t,px,py,pz,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz` in OpenVINS' own
gravity-aligned world frame.

**Start the window before takeoff.** Static initialisation needs a stationary
stretch and initialises from the window just *before* it detects motion;
`init_imu_thresh: 1.0` sits above prop wash (~0.68 m/s² std on the ground) and
below the takeoff transient. Starting mid-flight fails to initialise entirely.

### 5.3 Config notes that matter

| setting | value | why |
|---|---|---|
| `try_zupt` | **false** | once fired 4137× in flight, zeroing velocity at speed |
| `init_dyn_use` | false | segfaults this build of ov_init |
| `init_imu_thresh` | 1.0 | above prop wash, below takeoff |
| `calib_cam_*` | false | on only when recovering distortion (§4.1 step 3) |
| accel noise density | 3.0e-2 | props-ON measurement; the props-off floor (3e-4) makes the filter over-trust the accelerometer |
| gyro noise density | 7.0e-3 | props-ON |

### 5.4 The runner bug to never reintroduce

IMU must be fed to **`frame_time + 0.10 s`**, not `<= frame_time`. The
propagator interpolates between the samples *bracketing* the camera instant;
with no trailing sample it emits `Missing inertial measurements to propagate
with` at DEBUG verbosity only and otherwise silently does nothing, leaving the
filter on pure dead reckoning with **zero** features in every update.

**Diagnostic:** if VIO looks like dead reckoning, run at `verbosity: DEBUG` and
check `MSCKF update (N feats)` / `SLAM update (N feats)`. All-zero means
plumbing, not calibration.

---

## 6. VPE map (the database)

`imx900_frame_map.py`. Each map tile is one real video frame, undistorted, made
north-up, resampled to a common ground sample distance, and tagged with its
centre's ENU position from telemetry.

Per frame:
1. **undistort** with K and the radtan coefficients from §4;
2. resample by `scale = (K_GSD * agl) / COMMON_GSD`, `COMMON_GSD = 0.10` m/px;
3. rotate by `-heading_deg` into a **black-padded** square canvas of side
   `hypot(w, h)`.

Black padding rather than an inscribed crop: cropping discards ~50% of the
field of view and leaves consecutive tiles non-overlapping. Black is safe
because SuperPoint finds nothing in it — unlike `BORDER_REFLECT`, which
fabricates mirrored structure a matcher will latch onto.

```bash
python3 imx900_frame_map.py --survey survey44 --check-rotation   # ALWAYS first
python3 imx900_frame_map.py --survey survey44 --every 3.0 --min-agl 25
```

survey44 at 3 s spacing above 25 m AGL gives **71 tiles**, ~74 m footprint each
at 65 m AGL, covering E -101..9 / N -70..34 m. Tiles and their SuperPoint
descriptors are cached under `/tmp/imx900_map/`; the cache key includes
`K_GSD` and the distortion coefficients, so changing calibration invalidates it
automatically.

**Never build the map and the queries from the same flight.** A same-flight
split lets a query match its own frame and reverses which method looks better.

---

## 7. VPE localization

`vpe_localize_imx900.py`. SuperPoint (1024 keypoints) + LightGlue
(`filter_threshold=0.5`), both from the `lightglue` package.

Query frames go through the **same** `prepare()` as map tiles, so both are
north-up at the same GSD and a correct match is a near-pure translation.

Per candidate tile:
1. LightGlue match, require ≥18 correspondences;
2. `cv.estimateAffinePartial2D` with RANSAC (reproj threshold 4 px);
3. reject unless inliers ≥18 **and** inlier ratio ≥0.15;
4. reject unless fitted scale is within ±25% of 1.0 and rotation within 25° of
   identity — both images are already north-up at one scale, so a wrong tile
   shows up as absurd similarity parameters;
5. keep the tile with the most inliers.

Position from the winning tile:
```
p  = M[:, :2] @ query_centre + M[:, 2]      # query centre, in tile pixels
east  = tile.east  + (p[0] - tile_w/2) * COMMON_GSD
north = tile.north - (p[1] - tile_h/2) * COMMON_GSD    # image y is south
```

```bash
python3 vpe_localize_imx900.py --db-survey survey44 --q-survey survey45 \
    --q-every 1.0 --out vio_out/vpe_s44db_s45q_1hz.json
```

`--search-radius <m>` restricts candidates to tiles near a prior. Omitted, it
brute-forces every tile — correct for offline evaluation because each fix is
then independent, but see §11: onboard it is mandatory.

---

## 8. Fusion

`fusion_filter.py` (shared with the sim) driven by `fuse_vio_vpe_imx900.py`.

4-state horizontal Kalman filter, `x = [n, e, vn, ve]` in NED, constant-velocity
dynamics with piecewise-constant acceleration process noise (`q_a = 1.5`).
Horizontal only — ArduPilot's own EKF does attitude and altitude better than a
bolt-on filter should attempt.

- **Absolute fix** (`update_position`) — Mahalanobis-gated at χ² = 9.21 (2 DOF,
  99%), then bled in over 10 predict steps rather than applied as a jump.
  Bootstraps the filter on the first fix, so **VIO is never required to start**.
- **Velocity** (`update_velocity`) — gated on a speed envelope (`V_MAX_MS =
  12.0`, WPNAV_SPEED) and a scale ratio clamp `[0.2, 5.0]`.
- **Reject-run escape hatch** (`MAX_CONSEC_POS_REJECTS = 3`) — a Mahalanobis
  gate protects against a bad *fix* but cannot distinguish that from a bad
  *state*. Diverging VIO once got 35 of 43 correct fixes gated out permanently,
  scoring 635 m. After 3 consecutive rejections the state is treated as the
  thing at fault and re-bootstrapped from the rejected fix (635 m → 37 m). Set
  to 0 for the old behaviour.

**The VIO→ENU yaw is fitted from the VPE fixes, never from GPS.** OpenVINS'
world yaw is arbitrary; a real GPS-denied aircraft has to resolve it from its
own absolute fixes, and using GPS makes the whole evaluation circular.
`yaw_from_vpe()` does a robust complex least squares over the first 40 s of
fixes, constrained to rotation only (never scale).

**Set each sigma to that source's measured error**, not to whatever minimises
the GPS residual — the latter is tuning on the answer. Measured:

| flight | VIO velocity error | VPE error | sigmas used |
|---|---|---|---|
| survey43 @ 10 m | 0.48 m/s mean | 18.9 m mean | 0.5 m/s, 19 m |
| survey45 @ 65 m | 13–30 m/s | 4.8 m mean | 20 m/s, 5 m |

---

## 9. Evaluation protocol

Three rules, each learned the hard way.

1. **Align VIO to GPS with 4 DOF only** — yaw plus translation, never
   roll/pitch (gravity already fixes those) and never scale (recovering metric
   scale is the point of a visual-*inertial* system). Report best-fit scale
   separately as a diagnostic. `eval_vio_vs_gps.py` does this.

2. **Align on the opening airborne segment**, not the whole run. Aligning over
   a diverged run fits the transform to garbage and makes even the good early
   part score badly. And **refuse to report scale when it is not identifiable**
   — both flights open with a near-vertical climb, where a scale fit returned
   0.005 before that guard existed.

3. **Compare a continuous estimate against interpolated fixes, not against
   fixes at fix times.** A filter scored at 10 Hz and a fix set scored only at
   its own timestamps are not comparable. Straight-line interpolation between
   VPE fixes is the cheapest continuous baseline; anything the filter adds over
   that is what the odometry and dynamics model are actually worth. On
   survey45 the filter (13.0 m) *loses* to interpolation (6.2 m).

Cross-flight, never same-flight. Cross-altitude is a separate and harder
problem — say which you measured.

---

## 10. Measured performance

Desktop: RTX 2080 Ti, 12 cores, `num_opencv_threads: 4`.

| stage | cost | memory |
|---|---|---|
| VIO filter | **~10 ms/frame** at 1024x768 (13 s for 1305 frames, decode excluded) | 294 MB peak RSS |
| video decode | 289 fps (2048x1536 H.264, CPU) | — |
| VPE, 71 tiles brute force | **933 ms/query** (~1.1 Hz) | — |
| VPE, 42 candidates (60 m radius) | 564 ms (1.8 Hz) | — |
| VPE, 25 candidates (35 m radius) | 356 ms (2.8 Hz) | — |
| per candidate tile | **~12.8 ms** | — |
| tile images | — | 162 MB (71 tiles) |
| SuperPoint descriptors | — | 74 MB |

Accuracy, from `session_2026-08-12_imx900_vio.md`:

| | survey43 @ 10 m | survey45 @ 65 m |
|---|---|---|
| VIO only | **3.9 m** rmse | 130.3 m (diverges) |
| VPE fixes | 18.9 m mean, 63% fix rate | **4.4–4.8 m mean, 100%** |
| VPE interpolated 10 Hz | 32.2 m rmse | **6.2 m** rmse |
| VIO + VPE fused | 13.6 m rmse | 13.0 m rmse |

---

## 11. Porting to the Jetson

What changes, in order of how much work it is.

**1. Local search is mandatory, not optional.** Brute force is 933 ms/query on
a 2080 Ti and the Orin GPU is slower. Cost is linear in candidates (12.8
ms/tile), so drive candidate selection from the fusion filter's current state
and covariance: `--search-radius` of 3σ, floor ~35 m. That is 25 candidates,
356 ms on desktop. Budget for a few times that onboard and design for ~0.5–1 Hz
VPE, which is enough — 1 Hz versus 3 s queries changed accuracy by 0.4 m.

**2. Drop tile images after feature extraction.** Matching needs only
keypoints and descriptors plus each tile's centre and pixel size. That is 74 MB
instead of 236 MB. The map should ship as a descriptor file, not as imagery —
build it offline on the desktop and load it read-only.

**3. Replace file inputs with live streams.** The offline runner reads
`video.mkv` + `frame_times.csv` + `imu.csv`. Onboard, `VioManager` takes the
same `ov_core::ImuData` / `ov_core::CameraData` structs from the camera and FC
drivers — the feed logic in `run_video_msckf.cpp` transfers directly, including
the **`frame_time + 0.10 s` IMU lookahead** (§5.4), which onboard means holding
each frame until IMU has passed it. That is a real ~100 ms latency cost and it
is not optional.

**4. Use hardware decode.** CPU decode is 289 fps on desktop but will compete
with VIO for cores on the Jetson. Use NVDEC, or take frames before encoding.

**5. Feed the fusion output, not the raw estimators, to ArduPilot.** The whole
point of `fusion_filter.py` is that the autopilot never sees raw estimator
output — previously VIO divergence went straight into flight-critical code and
killed SITL with a floating-point exception. Publish
`VISION_POSITION_ESTIMATE` from the filter only, and stop publishing when the
filter has had no measurement for N seconds rather than continuing to emit
confident poses.

**6. Numerical caution.** `torch` + LightGlue on Jetson needs the NVIDIA
PyTorch build; the desktop wheel will not work. Verify SuperPoint output
matches the desktop on a fixed frame before trusting any onboard number,
especially if TensorRT or fp16 is introduced.

### Not yet solved before flying this

- **VIO is unusable at 65 m.** Do not plan an onboard VIO+VPE fusion at survey
  altitude until §12 item 1 is resolved. At 65 m today, VPE alone is the
  system.
- **Distortion is second-hand.** A 20-minute chessboard run would replace the
  weakest constant in the pipeline.

---

## 12. Known limits

1. **65 m AGL VIO diverges and the filter breaks down** (negative covariance
   diagonal at +55 s on survey45, +159 s on survey44) even with distortion
   corrected. Remaining suspects are the ones 10 m does not exercise:
   baseline/depth ratio (6.5× worse) and the near-planar scene.
2. **VIO scale error is 1.163 at 10 m** — 16% over. `scale_corrector.py` exists
   and has not been applied here.
3. **VPE errors are heavy-tailed** (median 2.3 m, max 58 m cross-altitude).
   This tail is what costs the fusion. DBSCAN false-positive filtering, or
   requiring consistency across consecutive fixes, is the obvious next step.
4. **Fusion beats neither input** in either regime. It buys robustness, not
   accuracy. If the altitude regime is known, pick the better source.
5. **No low-altitude map exists**, so the fusion has never been tested where
   both sources are good — the only condition under which it should be expected
   to win.
6. **Distortion coefficients come from OpenVINS' own online estimate**, not a
   chessboard.
7. **Single site, single day, three flights.** Nothing here has been tested
   across seasons, lighting, or a different site.
8. **ngps only.** AnyLoc/foundloc has not been run on this rig, so this method
   makes no claim about which VPR approach is better here.

---

## 13. File map

| file | role |
|---|---|
| `imx900_geo_common.py` | ENU origin + camera constants (K_GSD, distortion, mount) |
| `imx900_frame_map.py` | map tiles; `--check-rotation` self-test |
| `vpe_localize_imx900.py` | SuperPoint+LightGlue VPE fixes |
| `fusion_filter.py` | 4-state KF, gates, reject-run escape hatch |
| `fuse_vio_vpe_imx900.py` | fusion driver + baselines + figure |
| `eval_vio_vs_gps.py` | 4-DOF alignment and per-flight VIO plot |
| `plot_vio_summary_imx900.py` | cross-flight VIO summary figure |
| `calib_focal_widebaseline.py` | focal + mount angle (**the method that works**) |
| `calib_cam_imu_timeshift.py` | cam-IMU offset from gyro correlation |
| `calib_focal_{lsq,track,from_flight}.py` | negative results — do not reuse |
| `openvins_offline/run_video_msckf.cpp` | offline VIO runner |
| `openvins_offline/config_imx900/` | estimator + camera + IMU chains |
| `openvins_offline/{sweep_configs,ablate_calib}.sh` | config sweep / calibration ablation |
| `paper/imx900_vio_vpe_zh.tex` | 繁體中文 report — `xelatex` twice to build |
| `paper/make_figures_imx900.py` | its figures (reads the result JSONs, so they cannot drift) |
