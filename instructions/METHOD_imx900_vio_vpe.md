# Method: GPS-denied localization on the imx900 rig (VIO + VPE + fusion)

Complete, self-contained description of the pipeline as it stands on
2026-08-13 (revised after the survey47/48 flights and the SITL wiring),
written so it can be reproduced or ported to the Jetson without reading the
session logs. Every constant here was measured from flight data;
where a number is uncertain or second-hand that is stated.

Results and the experiments that produced them are in
`session_2026-08-12_imx900_vio.md`. This file is the *method*.

**Live on the Jetson (2026-08-14):** this method now has a running
implementation at `vio_vpe/` (git-tracked, ported from the desktop deploy
package at `instructions/jetson_deploy/` — see that folder's README) — a
shadow-mode observer that runs this exact pipeline live during a flight,
publishing nothing to the flight controller. See `vio_vpe/README.md` for
architecture, what's been validated on real hardware, and what hasn't
(notably: no real flight yet, only bench/replay testing). §11 below describes
the porting plan as it stood before that work; treat `vio_vpe/README.md` as
the current status over this section where they disagree.

---

## 1. What the pipeline does

```
video.mkv ─┬─> [VIO]  OpenVINS MSCKF ──────> relative pose, 15 Hz, drifts
imu.csv  ──┘                                        │
                                                    ├─> [FUSION] 4-state KF
video.mkv ───> [VPE]  SuperPoint+LightGlue ──> absolute fix, ~1 Hz    │
               against a frame map built                │             │
               from an earlier flight ──────────────────┘             │
                                                                      v
                                              [PUBLISH GATE] output(dt)
                                        slew limit + staleness withhold
                                                                      │
                                                                      v
                                        ONE VISION_POSITION_ESTIMATE stream
```

VIO is smooth but drifts and has a scale error. VPE is absolute but sparse and
heavy-tailed. The fusion filter reconciles them; the **publish gate** is what
the autopilot actually sees, and it is a separate concern from accuracy (§8.2).

**The VPE method is ngps** — SuperPoint + LightGlue. AnyLoc/foundloc has never
been run on this rig, so nothing here claims one VPR method beats the other.

**Current honest status**, by regime:

| | VIO | VPE | fusion vs causal baseline |
|---|---|---|---|
| 10 m AGL (survey43) | 1.1 m rmse (0.5% of path) | 63% fix rate cross-altitude | 2.6x better than ZOH |
| ~55 m, moving (survey48) | 14.7 m (1.7% of path) | 100%, 10.0 m mean | 1.1x better than ZOH |
| ~50 m, stop-start (survey45) | **diverges** (scale 0.217) | 100%, 4.6 m mean | worse than ZOH |

VIO quality is the whole story, and it is not a function of altitude alone --
survey45 and survey48 flew within 5 m of the same height with opposite results.
See §12.

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

### Flights available

| flight | AGL | airborne | GPS path | extent | median speed | role |
|---|---|---|---|---|---|---|
| `survey43` | 9 m | 80 s | 212 m | 57 x 28 m | 2.3 m/s | low-altitude test |
| `survey44` | 55 m | 249 s | 717 m | 112 x 104 m | 2.5 m/s | map |
| `survey45` | 48 m | 167 s | 348 m | 89 x 68 m | 0.1 m/s | test (VIO diverges) |
| `survey47` | 56 m | 274 s | — | 329 x 203 m | 6.4 m/s | **map, best coverage** |
| `survey48` | 51 m | 221 s | 869 m | 234 x 115 m | 2.8 m/s | **test, best VIO** |

All at the same site (22.5747-22.5749 N). survey47/48 additionally carry an
**imx219** second camera (`video_imx219.mkv`, 1640x1232) — unused here, the
Kalibr calibration covers only the imx900 in `video.mkv`.

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
# current best pair
python3 vpe_localize_imx900.py --db-survey survey47 --q-survey survey48 \
    --q-every 1.0 --out vio_out/vpe_s47db_s48q.json
```

`--search-radius <m>` restricts candidates to tiles near a prior. Omitted, it
brute-forces every tile — correct for offline evaluation because each fix is
then independent, but see §11: onboard it is mandatory.

---

## 8. Fusion

`fusion_filter.py` (shared with the sim) driven by `fuse_vio_vpe_imx900.py`.

4-state horizontal Kalman filter, `x = [n, e, vn, ve]` in NED, constant-velocity
dynamics. Horizontal only — ArduPilot's own EKF does attitude and altitude
better than a bolt-on filter should attempt.

### 8.1 Estimation

| parameter | value | why this value |
|---|---|---|
| `accel_process_noise` | **5.0** | a tight model lags 1 Hz fixes. survey45 rmse: q=0.1 -> 21.3 m, 1.5 -> 12.5, **5 -> 9.0**, 50 -> 12.1 |
| `VPS_CHI2_THRESHOLD` | **23.0** (99.999%) | at 9.21 (99%) the gate rejected fixes worse than average but far better than the drifting state replacing them, then triggered resets: 12.25 m with 6 rejections vs 9.17 m with none |
| `VPS_SOFT_FRAMES` | **1** in the state | smoothing belongs on the published output (§8.2), not in the state; leave at 5 if publishing `state()` directly |
| `MAX_CONSEC_POS_REJECTS` | **3** | see below |
| `MIN_VEL_ACCEPT_RATE` | **0.70** | see below |

- **Absolute fix** (`update_position`) — Mahalanobis-gated, then bled in rather
  than applied as a jump. Bootstraps on the first fix, so **VIO is never
  required to start**.
- **Velocity** (`update_velocity`) — gated on a speed envelope (`V_MAX_MS =
  12.0`, WPNAV_SPEED) and a scale ratio clamp `[0.2, 5.0]`.
- **Reject-run escape hatch** (`MAX_CONSEC_POS_REJECTS`) — a Mahalanobis gate
  protects against a bad *fix* but cannot tell that from a bad *state*.
  Diverging VIO once got 35 of 43 correct fixes gated out permanently, scoring
  635 m. After 3 consecutive rejections the state is treated as the thing at
  fault. It **inflates covariance, it does NOT teleport to the fix** — the
  original teleport produced a 52 m step in a single 100 ms output.
- **Velocity health** (`MIN_VEL_ACCEPT_RATE`) — the constant-velocity model is
  only worth having if the velocity input is real. Below a 70% rolling
  acceptance rate the velocity state is held at zero, degrading the filter to a
  position random walk. The filter's own gates separate the cases cleanly:
  survey45 accepts 52%, survey43 89%, survey48 100%.

### 8.2 The publish gate — `output(dt)`, never `state()`

**This is a flight-safety concern and it is invisible in accuracy statistics.**
A track can sit at 11 m rmse and contain a 52 m step in one 100 ms sample —
520 m/s of apparent velocity, which EKF3 fuses and the position controller then
chases.

`output(dt)` walks the published position toward the state at a bounded total
speed, and returns `None` when the estimate has nothing behind it:

| parameter | value | notes |
|---|---|---|
| `MAX_CORRECTION_MS` | **5.0** m/s | bound is on TOTAL motion (`V_MAX_MS + this`), not on a correction added to a feed-forward term. When velocity is held at zero there IS no feed-forward, and a correction-only budget leaves the output permanently saturated (measured: 45% of samples, path visibly cutting corners). |
| `MAX_FIX_AGE_S` | **5.0** s offline | withhold once the newest **absolute** fix is older than this. **Must scale with the VPE rate: >= 5x the median inter-fix interval.** 15 s in the sim, where AnyLoc runs at 2 s intervals plus ~2 s per NO FIX. |

Measured slew trade-off on survey45 (max published step vs accuracy):

| limit | max step | implied | rmse | median |
|---|---|---|---|---|
| 1.0 m/s | 0.64 m | 6.4 m/s | 40.9 | 21.8 — cannot keep up |
| **5.0** | **1.07** | **10.7** | 23.3 | 3.5 |
| none | **59.70** | **597** | 14.6 | 3.4 — unsafe |

**Gate on fix AGE, not on covariance.** Covariance gating was tried first and
rejected: on survey45 the correlation between `pos_sigma` and true error is
**0.109**, and the bands are non-monotonic (sigma 0-5 m contains 69.5 m errors
while sigma 20-50 m tops out at 5.0 m). The filter does not know when it is
wrong.

**Withholding is only safe if the autopilot has a failsafe for losing its
vision source.** In SITL the ground-truth geofence was that response. A real
aircraft needs an EKF failsafe to LAND/RTL; that is not yet configured.

### 8.3 The VIO->ENU yaw — fitted from VPE, never from GPS

OpenVINS' world yaw is arbitrary. A real GPS-denied aircraft has to resolve it
from its own absolute fixes; using GPS makes the evaluation circular.

**The fit window must contain horizontal MOTION, not just elapsed time.**
`pick_yaw_window()` extends the window until the fixes span `--yaw-fit-span`
metres (default 150). Without it, survey48's first 40 s spans **3.8 m** while
climbing and returns **-139.6 deg against a true +20.4 deg** — a 160 deg error
that rotates a good VIO velocity backwards:

| yaw window | spatial span | fitted yaw |
|---|---|---|
| 40 s (time only) | 3.8 m | **-139.6 deg** |
| 60 s | 152 m | +21.3 |
| truth (GPS, reference only) | — | **+20.4** |

Effect on survey48, changing nothing else: VIO-only 183.2 -> **14.7 m**,
fused 49.7 -> **16.3 m**. Same class of failure as the "scale not identifiable
during a climb" guard in `eval_vio_vs_gps`; the yaw fit simply never had one.

### 8.4 Sigmas

**Set each sigma to that source's measured error**, not to whatever minimises
the GPS residual — the latter is tuning on the answer.

| flight | VIO velocity error | VPE error | sigmas used |
|---|---|---|---|
| survey43 @ 10 m | 0.48 m/s | 18.9 m | 0.5 m/s, 19 m |
| survey45 @ 50 m | 35.7 m/s | 4.6 m | 36 m/s, 5 m |
| survey48 @ 54 m | 1.10 m/s | 10.0 m | 1.1 m/s, 10 m |

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

3. **The causal baseline is ZERO-ORDER HOLD, not interpolation.** A filter
   scored at 10 Hz and a fix set scored only at its own timestamps are not
   comparable, so a continuous baseline is needed — but linear interpolation
   uses the *next* fix, which in flight has not arrived. Holding the newest
   fix received is what a real system could actually do. On survey48:
   interpolation 14.2 m (not causal), ZOH 17.2 m (causal), fused 15.4 m. Quote
   both, and say which is which.

4. **Measure the per-sample STEP, not only the error.** Accuracy statistics
   hide the publish-path failure mode entirely — an 11 m rmse track contained a
   52 m instantaneous step. `analyze_fusion_jumps.py` reports it; run it on any
   config before flying it.

5. **Separate contiguous steps from gap-resumes.** After the staleness gate
   withholds for >= `MAX_FIX_AGE_S`, resuming re-anchors the output. That is a
   source re-acquisition the autopilot handles, not an in-stream jump, but a
   naive diff of consecutive published samples counts it as a 38 m step.

6. **Normalise VIO drift by path length.** 14.7 m rmse sounds worse than 1.1 m
   until you note the flights were 869 m and 212 m: 1.7% versus 0.5%.

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

### VIO drift, normalised by distance flown

| flight | AGL | GPS path | VIO rmse | drift |
|---|---|---|---|---|
| survey43 | 9 m | 212 m | 1.1 m | **0.51%** |
| survey48 | 51 m | 869 m | 14.7 m | **1.69%** |
| survey45 | 48 m | 348 m | 1867 m | diverged (scale 0.217) |

### End-to-end, survey47 map -> survey48 test (the current best case)

| | rmse | median | max | causal? |
|---|---|---|---|---|
| VIO only | 14.7 | 9.5 | 62.1 | yes |
| VPE at fix times | 14.9 | 3.7 | 42.1 | yes |
| VPE linear interpolation | 14.2 | 3.7 | 42.0 | **no** |
| VPE zero-order hold | 17.2 | **6.8** | 50.4 | yes |
| **fused, PUBLISHED** | **15.4** | 9.1 | **45.9** | yes |

Fusion beats the causal baseline on rmse and worst case, **but ZOH still wins
on median** — a modest win on two of three statistics, not a decisive one.

Publish path on that run: max step 1.13 m (11.3 m/s), **0 slew-limited, 0
resets**, 99% published. With correct sigmas the state never jumps, so the
limiter sits idle — that is what a healthy configuration looks like. Contrast
survey45: 60-72 m raw steps, limiter active 17-44% of samples.

Older results, from `session_2026-08-12_imx900_vio.md`:

| | survey43 @ 10 m | survey45 @ 65 m |
|---|---|---|
| VIO only | **3.9 m** rmse | 130.3 m (diverges) |
| VPE fixes | 18.9 m mean, 63% fix rate | **4.4–4.8 m mean, 100%** |
| VPE interpolated 10 Hz | 32.2 m rmse | **6.2 m** rmse |
| VIO + VPE fused | 13.6 m rmse | 13.0 m rmse |

---

## 10.5 The live path (Gazebo/ArduPilot, and the template for onboard)

`gazebo_mission.py --fused` publishes **`filt.output(dt)`**, skips the send when
it returns `None`, and counts what it withheld. Flags: `--slew-limit`,
`--max-fix-age` (15 s in the sim, not the offline 5 s), `--loc-ready-timeout`.

**Build the localizer BEFORE arming the publish path.** AnyLoc spends 60+ s
loading DINOv2 and building its VLAD codebook. Arming the fused feeder first
leaves the staleness gate with only the bootstrap to publish, so it correctly
withholds after `--max-fix-age` and the aircraft loses aiding entirely —
`[AnyLoc] localizer ready` printed *after* `fused feeder stopped`. The old
design survived this only by having the staleness bug. `_loc_ready` is now set
when the localizer object is constructed and the handover waits on it.

SITL status: `output()`, the slew limiter, the staleness gate and the readiness
handshake all behave as designed (run with fixes: 279 messages, 0 withheld, 98
slew-limited; run without: 150 messages then withheld, exactly 15 s at 10 Hz).
**Not yet shown**: that EKF3 tracks the slew-limited stream *better* than the
raw one. That needs a same-seed A/B of `state()` vs `output()` with a
well-behaved localizer, and the sim does not currently provide one (§12).

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

- **VIO reliability at survey altitude is not established** (§12.1). survey48
  works, survey45 does not, at the same height. Do not plan an onboard fusion
  that assumes VIO is available until that is understood.
- **No EKF failsafe** for the withheld-vision case. The gate makes "no data"
  explicit; something has to act on it.
- **The `state()` vs `output()` benefit is unproven in the loop** (§10.5).

---

## 12. Known limits

1. **VIO failure is not a function of altitude alone.** survey45 (48 m) and
   survey48 (51 m) flew within 5 m of the same height: one diverged with
   velocity scale 0.217, the other tracked at 1.7% of path with scale 1.076.
   survey45 is stop-start (median speed 0.1 m/s) and survey48 is not, which is
   the leading hypothesis, but it is **not established** — survey45's full path
   is 348 m, so it is not simply motion-starved. Until this is understood, VIO
   cannot be relied on at survey altitude.
2. **Fusion beats the causal baseline only narrowly, and only on 2 of 3
   statistics** (survey48). Where VIO is bad it is worse than a zero-order hold.
3. **VPE errors are heavy-tailed** (survey48: median 3.7 m, max 42.1 m). The
   inlier ratio predicts the error strongly (Spearman -0.858 on survey45) but
   **gating or per-fix adaptive sigma on it did not help** (14.6 -> 14.6 / 15.0
   / 15.4 m) — the tail is not what costs the fusion.
4. **Map tile density matters**: survey47 covers 3x survey44's area at the same
   3 s spacing, and VPE mean error went 4.6 -> 10.0 m. Sample denser for large
   maps.
5. **Distortion coefficients are Kalibr's**, but three of the five calibration
   sessions have corrupt imu-cam chains (§4.0) — always cross-check before
   reusing one.
6. **The sim's ngps localizer returns zero fixes**, cause unknown. Porting the
   frame map into it and matching GSDs (both real defects, both fixed) did not
   change it. Untested candidates: the fixed 90 deg `QUERY_ROT` versus a
   body-fixed camera on a turning aircraft; a rendered-vs-real appearance gap.
   This is why SITL testing has had to use AnyLoc as a fixture.
7. **No EKF failsafe is configured** for the vision source being withheld.
8. **Single site, single day per pair, five flights.** Nothing tested across
   seasons, lighting, or a different site.
9. **ngps only.** foundloc/AnyLoc has never been run on this rig, so no claim
   is made about which VPR method is better here.

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
| `analyze_fusion_jumps.py` | **per-sample published step** — run before flying any config |
| `plot_slew_path.py` | published path vs raw state vs GPS, plus the step trace |
| `gazebo_mission.py` | live path; `--fused` publishes `output(dt)` |
| `paper/imx900_vio_vpe_zh.tex` | 繁體中文 report — `xelatex` twice to build |
| `paper/make_figures_imx900.py` | its figures (reads the result JSONs, so they cannot drift) |
