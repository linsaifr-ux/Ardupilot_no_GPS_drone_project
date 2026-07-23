# survey17 OpenVINS offline evaluation — 2026-07-22

First offline OpenVINS run (plan step 5 of instructions/vio_data_collection.md),
**without a Kalibr calibration** — intrinsics FOV-derived, zero distortion,
extrinsics a hand-derived nadir guess, time offset estimated online. All
numbers below carry that caveat.

## Setup
- OpenVINS v2.6.3, built ROS-free on the Jetson at `~/openvins_ws/`
  (local Ceres 2.1.0 in `deps/`, custom standalone feeder
  `ov_msckf/src/run_video_msckf.cpp`: video.mkv + frame_times.csv + imu.csv → traj csv).
- Comparison: `~/openvins_ws/compare_vio_gps.py` (4-DOF yaw+translation alignment
  vs telemetry.csv GPS; GPS was genuine SRC1 truth, FC at default params).
- Configs in `config_used/` (static init, used for full run) and
  `config_used_dyn/` (dynamic init, used for cruise-only run). ~15 Hz feed
  (stride 2), images downsampled to 820x616, ~26-36 fps offline on Orin NX CPU.

## Flight timeline (video-relative seconds)
0–195 static · 195–210 hand-carry to pad · 210–258 static on pad ·
~260 liftoff · 285–395 low pass at ~20 m AGL · 400–440 vertical climb
20→104 m (hover, 2.5 m/s) · 440–850 survey legs at ~104 m AGL ·
850–960 descent+landing. GPS path ≈ 3.8 km.

## Results
| Segment | Result |
|---|---|
| Early flight (225–430 s, 744 m, 20 m AGL) | drift-aligned err 3–6 m through 650 m (<1%); 4DOF ATE rmse 17.9 m; best-fit scale 0.79, scale-corrected rmse **5.2 m** |
| Vertical climb (400–440 s) | **breaks the filter**: vertical motion overestimated 1.45x, accel-x bias runaway 0.15→0.29 m/s², filter then invents a descent and diverges (full-flight run ends 87 km off — meaningless) |
| Cruise re-init (dyn init at 448 s, 2.7 km of legs) | shape good (all legs/turns visible), alt within ~5 m, but XY **scale collapsed to 0.43x**; 4DOF ATE rmse ~110 m; scale-corrected rmse 75 m |
| Descent (850 s+) | breaks the filter again |

## Interpretation
- Local odometry shape is genuinely good given zero calibration — supports the
  VIO direction (goal from the SITL study: ~1 m error class needs calibrated VIO).
- Two failure modes, both expected for this setup:
  1. **Scale instability** (0.79x early, 2.35x-off in cruise): monocular scale is
     held only by the accelerometer; constant-velocity cruise gives no excitation,
     and rough calibration + vibration eat the rest. External absolute fixes
     (AnyLoc / plan-B fusion) or a rangefinder/baro height factor would bound it.
  2. **High-throttle vertical phases diverge**: candidate causes, in likelihood
     order: vibration aliasing on the raw 200 Hz MAVLink RAW_IMU stream (FC EKF
     uses filtered delta-velocities; we get unfiltered samples), unmodeled lens
     distortion under pure radial flow, rolling shutter.
- Verdict: **not usable as-is; do the Kalibr session before judging OpenVINS.**
  Calibration is the gate for the intrinsics/extrinsics/time-offset unknowns;
  if climb divergence persists afterward, the IMU path (aliasing) is the culprit
  and needs INS_ notch/filter review or a dedicated IMU.

## Altitude reference check: GPS vs baro AGL (2026-07-22)

Question: is VIO altitude error smaller against baro AGL than against GPS alt?
Answer: **the comparison cannot distinguish them on this recording.** telemetry.csv
carries only one independent vertical signal — `alt_amsl` (global_position/global)
and `alt_agl` (rel_alt) are the same EKF vertical state, offset by home AMSL
(~91 m; they differ by ≤0.8 m, a step when home was reset at arming). With FC at
default params (EK3_SRC1_POSZ=1) that EKF alt is **already baro-primary**, so the
baro AGL was effectively the reference all along. Raw GPS altitude was not
recorded (would need /mavros/global_position/raw/fix).

VIO altitude error vs baro AGL (identical vs GPS-frame alt to ±0.02 m), healthy
windows only, init-anchored / [mean-aligned]:

| Segment | rmse | mean | median | p95 | max |
|---|---|---|---|---|---|
| Early 225–430 s (incl. climb-overshoot onset) | 7.2 [6.3] | 3.8 [4.1] | 1.4 [2.7] | 20.0 [16.5] | 29.7 [26.2] m |
| Early, pre-climb only (225–400 s, init-anchored) | 1.8 | 1.5 | 1.1 | 4.0 | 5.1 m |
| Cruise 448–775 s (clipped before divergence at ~777 s) | 6.3 [4.2] | 4.7 [3.0] | 3.6 [2.0] | 14.1 [9.5] | 18.5 [13.9] m |

(The earlier "alt within ~5 m" claim holds for the first ~250 s of cruise; VIO
alt then drifts down ~-13 m by 750 s and the run diverges at ~777 s, earlier
than the 850 s descent — the 110 m XY ATE reproduces only when clipped there.)
Script: scratchpad vio_alt_baro_compare.py → `vio_alt_baro_compare.png`.
Takeaway: for a baro-vs-GPS reference test, record raw GPS fix alongside; for
bounding VIO scale/alt in integration, baro AGL is available and is exactly what
the FC already flies on.

## Re-run with real Kalibr calibration (2026-07-23)

Kalibr session calib_20260723_001209 (run on the PC, results in that folder's
`kalibr_output/`) wired into `~/openvins_ws/config/survey17_kalibr` (static
init) and `survey17_kalibr_dyn` (dynamic init): fx/fy 1339.3/1335.9,
cx/cy 818.6/639.8, radtan [0.064, -0.204, 0.0003, -0.0022], Kalibr T_ic
extrinsics (|t| = 0.358 m — confirmed against the airframe: camera really
sits 35.8 cm from the FC IMU),
timeshift_cam_imu -0.056 s, sigma_px 1.5→1, online refinement kept on.
Online estimates agreed with Kalibr (dt converged -0.059 s; cruise extrinsic
z stayed -0.363 m), which cross-validates the calibration.

Same segments, same methodology (`vio_kalibr_stats.py`):

| Segment | Uncalibrated | Kalibr-calibrated |
|---|---|---|
| Early 225–430 s (667 m) ATE2D rmse | 16.7 m (scale corr 0.80 → 4.7 m) | 18.3 m (scale corr 0.78 → 5.9 m) — **unchanged** |
| Climb 400–440 s | diverges ~483 s | **still diverges ~473 s** |
| Cruise 448–775 s ATE2D rmse | 108.8 m (VIO scale 0.34x, corr 35.3 m) | **85.9 m** (VIO scale 0.48x, corr 27.5 m) |
| Cruise 448–850 s ATE2D rmse | 348.1 m (diverged at ~777 s) | **88.7 m** (survives to descent ~815 s) |
| Cruise alt vs baro AGL rmse (448–775) | 6.2 m | 9.3 m (max 22.5; no -13 m ramp, but dips to -25 m near 810 s) |

Verdict of the calibration gate:
- Calibration **is not what was limiting**. Early-segment accuracy and scale
  are unchanged; the 1.26x early / ~0.5x cruise scale error and the
  high-throttle climb divergence all persist with a good camera model.
- What it did buy: cruise robustness (no mid-cruise divergence at 777 s —
  runs until the descent) and a ~20 % better cruise ATE.
- Per the pre-registered suspect list, the remaining failure modes now point
  at the **IMU path: vibration aliasing on raw 200 Hz MAVLink RAW_IMU**
  (unfiltered, unlike the FC-EKF's delta-velocities) — next steps would be
  INS_ notch/filter review, feeding FC-filtered IMU, or a dedicated IMU —
  and at rolling shutter. Monocular scale still needs an external bound
  (baro/AnyLoc) in any real integration regardless.
- Plot: `vio_path_compare_kalibr.png` (+ `.py`); runs `vio_full_kalibr.csv`
  (static init 210 s), `vio_cruise_kalibr.csv` (dyn init 448 s), logs
  `run_*_kalibr.log`.

## IMU vibration-aliasing evidence (2026-07-23)

`imu_vibe_spectrum.py` → `imu_vibe_spectrum.png`: Welch spectra of imu.csv
per flight phase. Motors-off pad window is clean (accel std 0.004–0.007
m/s²); every in-flight window shows a ~60 dB-raised floor flat to the
100 Hz Nyquist (the aliasing signature — folded prop/motor harmonics) plus
wandering ~78 Hz lines in the climb. This, with the §14-2 result above,
shifts the blame for the remaining VIO failures from calibration to the
IMU path. Fix plan and live FC measurements:
`instructions/vpe_jump_runaway_diagnosis.md` §14-4~14-7.

## Files
- `vio_full.csv` — full-flight run (static init at liftoff; diverges post-climb)
- `vio_cruise.csv` — cruise run (dynamic init at 448 s; diverges at descent)
- `vio_full_kalibr.csv` / `vio_cruise_kalibr.csv` — same runs with the real
  Kalibr calibration (2026-07-23; gitignored like all traj CSVs)
- `vio_early_seg.png`, `vio_cruise_seg.png`, `vio_full_compare.png` — plots
  (uncalibrated); `vio_path_compare_kalibr.png` — calibrated counterpart
- `vio_kalibr_stats.py` — baseline-vs-calibrated segment stats table
- `imu_vibe_spectrum.py` / `.png` — vibration aliasing evidence (above)
- traj csv columns: t(unix), px..pz (m, world z-up), qx..qw (JPL q_GtoI),
  vx..vz, cam_dt, gyro/accel biases
