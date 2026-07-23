# Kalibr calibration — PC instructions for session calib_20260723_001209

> ✅ **COMPLETED 2026-07-23** — results copied back to `kalibr_output/`,
> all acceptance criteria passed, wired into
> `~/openvins_ws/config/survey17_kalibr[_dyn]` on the Jetson, survey17
> re-eval done (verdict: `instructions/vpe_jump_runaway_diagnosis.md` §14).
> One correction to the criteria below: the "translation … within ~10 cm"
> expectation in step 5 was wrong — the camera really sits **35.8 cm** from
> the FC IMU on this airframe (physically confirmed); Kalibr's
> |t| = 0.358 m is correct. Kept for reference for future re-calibrations.

Self-contained instructions to run the camera + camera-IMU calibration for
this recording on a PC. Written so a Claude Code session on the PC can
execute it end-to-end; a human can follow it too.

## Context (why this exists)

No-GPS drone project (ArduPilot + Jetson Orin NX). OpenVINS VIO was run
offline on a real flight (survey17) with **guessed** intrinsics (FOV-derived,
zero distortion) and **guessed** nadir extrinsics; result: trajectory shape
good but monocular scale collapsed (0.43x on cruise) and climb phases
diverge. This Kalibr session replaces the guesses — it is the gate for the
whole VIO evaluation.

- Camera: IMX219 CSI, 1640x1232 @ 30 fps, fixed focus, rolling shutter.
- IMU: FC (Pixhawk 6C) raw IMU over MAVLink at 200 Hz — not a Jetson-local
  IMU. No hardware sync; timestamps are FC-timesync-mapped to the same epoch
  as the frame timestamps (residual offset ~ms; Kalibr estimates it).
- Target: Kalibr AprilGrid 6x6 (36h11), printed at 100% on A0
  (nominal tagSize 0.088 m, tagSpacing 0.3).

**This session was validated on the Jetson and ACCEPTED** (5th take; earlier
takes had dead IMU / defocused close-ups / motion blur):

- 121 s video, 3600 frames, intact (ffprobe count == frame_times.csv rows).
- IMU 200.0 Hz, zero gaps > 20 ms, covers the video to the end.
- Tag-detection sweep (OpenCV proxy): mean 15.4/36 tags, median 17, 55 % of
  frames ≥ 15 tags; sharpness holds during motions.
- First ~12.5 s = defocused close-up prefix, zero detections → the prepare
  script trims it.

## What's in this folder

| file | meaning |
|---|---|
| `video.mkv` | H.265, 1640x1232@30, **already rotated 180°** (recorded orientation) |
| `frame_times.csv` | `frame_idx,unix_time` — row n ↔ video frame n |
| `imu.csv` | `stamp_ros,recv_unix,wx,wy,wz,ax,ay,az` (rad/s, m/s²) 200 Hz |
| `attitude.csv`, `telemetry.csv` | not needed for calibration |
| `meta.json` | note `frame_rotation_deg: 180`, `purpose: calibration` |
| `prepare_kalibr_input.py` | converts the above into Kalibr input (step 2) |

## Hard rules

1. **Never rotate/flip the images.** The 180° rotation is already baked into
   video.mkv, and the flight-time OpenVINS feed uses the same orientation.
   Calibration must match it.
2. Frame↔timestamp mapping is *by index* (frame n ↔ csv row n). Don't
   re-extract frames with anything that can drop frames silently; use the
   provided script (it verifies the count).
3. Requirements: `docker`, `ffmpeg`, `python3`. ~8 GB free disk for PNGs
   + ~7 GB for the rosbags. No ROS needed on the host (Kalibr runs in Docker).

## Step 1 — measure the printed grid, fill target.yaml

Run step 2 first if `target.yaml` doesn't exist yet (the script writes a
template), then **measure the physical print with a ruler** and edit:

- `tagSize`: edge of one black tag square in metres (nominal 0.088),
- `tagSpacing`: (gap between adjacent tags) / tagSize (nominal 0.3,
  i.e. gap ≈ 26.4 mm).

Printers rescale; measured values are required, ask the user to measure if
needed. A wrong absolute size mostly rescales the extrinsic translation, but
there is no reason to accept that error.

## Step 2 — convert to Kalibr input

```bash
cd calib_20260723_001209
python3 prepare_kalibr_input.py        # ~5 min; writes kalibr_input_full/ + kalibr_input_4hz/
```

`kalibr_input_full/` = all frames after the 12.5 s trim @30 fps (for the
imu-camera stage); `kalibr_input_4hz/` = every 8th frame (~3.75 Hz, for the
intrinsics stage — full rate would make it needlessly slow). Both contain
the same `imu0.csv` (ns timestamps).

## Step 3 — Kalibr via Docker

```bash
git clone https://github.com/ethz-asl/kalibr.git
cd kalibr && docker build -t kalibr -f Dockerfile_ros1_20_04 .   # once, ~15 min

FOLDER=/absolute/path/to/calib_20260723_001209
docker run -it -v "$FOLDER:/data" kalibr
# ---- everything below runs INSIDE the container ----
source devel/setup.bash
cd /data/kalibr_input_4hz  && rosrun kalibr kalibr_bagcreater --folder . --output-bag /data/calib_4hz.bag
cd /data/kalibr_input_full && rosrun kalibr kalibr_bagcreater --folder . --output-bag /data/calib_full.bag
```

(No X11/display needed: pass `--dont-show-report` below; the PDF reports are
written to disk regardless.)

## Step 4 — camera intrinsics

```bash
cd /data
rosrun kalibr kalibr_calibrate_cameras --bag calib_4hz.bag \
    --topics /cam0/image_raw --models pinhole-radtan \
    --target target.yaml --dont-show-report
```

**Accept if:** mean reprojection error **< 0.5 px**; distortion coefficients
non-zero but sane (|k1|,|k2| < 0.5 for this lens). Sanity anchor: the
FOV-derived guess was fx≈fy≈1359, cx≈820, cy≈616 — Kalibr's result should
land within roughly ±10 % of that; a wildly different focal length means
something is wrong (target.yaml sizes, wrong topic, rotated images).

Output: `camchain-calib_4hz.yaml` + report PDF + results txt.

## Step 5 — camera-IMU extrinsics + time offset

```bash
rosrun kalibr kalibr_calibrate_imu_camera --bag calib_full.bag \
    --cam camchain-calib_4hz.yaml --imu imu.yaml \
    --target target.yaml --dont-show-report
```

Takes a while (~3200 frames; an hour+ is normal). **Accept if:**

- reprojection error < 1 px; gyro/accel error plots look like white noise
  (visible structure = insufficient excitation → would need a re-record);
- estimated time offset is ms-scale and stable in the report (not drifting);
- `T_cam_imu` rotation ≈ the known mounting, camera looking straight down,
  image-top = body-forward. The OpenVINS-convention guess used on survey17
  (R camera→IMU/FLU-body, i.e. the transpose of Kalibr's imu→cam rotation)
  was:

  ```
  [ 0 -1  0 ]
  [-1  0  0 ]
  [ 0  0 -1 ]
  ```

  Expect roughly this ± a few degrees. If it looks like a completely
  different axis permutation, the frames were flipped somewhere — stop and
  re-check rule 1.
- translation magnitude plausible for the airframe: camera is within ~10 cm
  of the FC.

Outputs: `camchain-imucam-calib_full.yaml`, `imu-calib_full.yaml`,
`results-imucam-calib_full.txt`, report PDF.

## Step 6 — what to bring back to the Jetson

Copy back into this session folder on the Jetson
(`field_data/calib_20260723_001209/kalibr_output/`):

- `camchain-calib_4hz.yaml` (intrinsics)
- `camchain-imucam-calib_full.yaml` (extrinsics + time offset)
- `imu-calib_full.yaml`
- both `results-*.txt` and report PDFs

Next step there: write these into the OpenVINS config (replacing the guessed
`kalibr_imucam_chain.yaml` / `kalibr_imu_chain.yaml` under
`field_data/survey17/vio_eval/config_used/`) and re-run the survey17 offline
eval (`~/openvins_ws` `run_video_msckf`, three segments) to judge scale
collapse and climb divergence against the uncalibrated baseline.

## Known gotchas

- `kalibr_calibrate_cameras` needing more init iterations on wide-margin
  frames is normal; if it fails to initialize focal length, retry with
  `--approx-sync 0.04`.
- If validating detections yourself with OpenCV instead of Kalibr: aruco
  `DICT_APRILTAG_36h11` needs `markerBorderBits=2` for Kalibr aprilgrids
  (default 1 detects nothing).
- The board print is glossy — a few frames have specular glare washing out
  tags. Detection statistics above already account for it; Kalibr simply
  skips unreadable corners.
- imu0.csv timestamps and frame timestamps share the same epoch (unix ns).
  `kalibr_bagcreater` accepts them as-is; do not re-zero them.
