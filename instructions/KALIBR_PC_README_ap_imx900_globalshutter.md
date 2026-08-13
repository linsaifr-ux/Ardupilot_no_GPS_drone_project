# Kalibr calibration — PC instructions for the AP-IMX900 (4mm CS-mount, global shutter) rig

> **TEMPLATE.** A real recording now exists and its own filled-in copy of this
> file lives at `field_data/calib_20260811_232301/KALIBR_PC_README.md` (primary
> session, 122s; a backup take `calib_20260811_232621` was also recorded but not
> yet processed) — **use that one**, not this template, to actually run Kalibr.
> This template is kept for the *next* time the camera/lens is swapped again:
> copy it into the new `field_data/calib_<timestamp>/` session folder, fill in
> the `<SESSION_ID>` / `<TODO>` placeholders with real values from that
> recording's `meta.json` / `imu_rates.json` / a tag-detection sweep (see
> `tools/README.md`'s `prepare_kalibr_input.py` section for how — **do not
> trust the script's `--trim-head` default**, it's stale and must be
> re-derived per-recording), and update the banner once results are accepted —
> mirrors `field_data/calib_20260723_001209/KALIBR_PC_README.md`, the completed
> calibration for the IMX219 camera before this one.

Self-contained instructions to run the camera + camera-IMU calibration for this
recording on a PC. Written so a Claude Code session on the PC can execute it
end-to-end; a human can follow it too.

## Context (why this exists)

No-GPS drone project (ArduPilot + Jetson Orin NX). The camera was reverted
2026-08-09 from IMX219 (CSI, rolling shutter) back to the original AP-IMX900-Mini-USB3
body, now fitted with a new 4mm CS-mount lens (Sony/Appropho spec: **global shutter**,
Pregius S series). No real calibration has ever been done for this exact body+lens
combination — only the IMX219's 2026-07-23 calibration exists, which does not apply
(different sensor, different lens, different intrinsics/distortion, and the reused
extrinsics are only an approximate physical-mount guess).

An offline OpenVINS test on this camera (survey38/41/42, 2026-08-11) using an
**approximate** config (FOV-computed-not-measured intrinsics + reused IMX219-rig
extrinsics as an online-refined seed) diverged catastrophically (3-4 orders of
magnitude worse than the old camera's own uncalibrated baseline) — a diagnostic run
with online refinement frozen diverged even worse, ruling out "refinement is
destabilizing a good seed" and pointing at the seed itself being too far off. This
Kalibr session is the fix: replace the guesses with a real measurement, same as the
IMX219 session did on 2026-07-23 (full history: memory `vio-globalshutter-test-
inconclusive`, `instructions/vpe_jump_runaway_diagnosis.md` §14-45).

- Camera: AP-IMX900-Mini-USB3, USB (MJPG capture via `control/usb_camera_node.py` /
  OpenCV `CAP_V4L2`), **2048x1536 @ 30 fps**, 4mm CS-mount lens, **global shutter**.
- IMU: FC (Pixhawk 6C) raw IMU over MAVLink — not a Jetson-local IMU. Requested rate
  is configurable (`--imu-hz`, default 200, this project's current standing practice
  for flight recordings is `--imu-hz 333`, achieving ~320 Hz in practice) — **use
  whichever you actually recorded with, and set `imu.yaml`'s `update_rate` to the
  real achieved rate from that session's `imu_rates.json`, not the requested one.**
  No hardware sync; timestamps are FC-timesync-mapped to the same epoch as the frame
  timestamps (residual offset sub-ms to ms; Kalibr estimates the residual).
- Target: Kalibr AprilGrid 6x6 (36h11), `instructions/april_6x6_80x80cm_A0.pdf`,
  printed at 100% on A0 (nominal tagSize 0.088 m, tagSpacing 0.3) — reuse the same
  physical print as the IMX219 session if you still have it; the target itself hasn't
  changed, only the camera has.

**This session's Jetson-side recording has NOT been validated yet.** Fill in once
recorded (see the IMX219 session's README for the shape of what "accepted" looked
like: tag-detection sweep stats, IMU rate/gaps, sharpness through motion, any
defocused-prefix seconds to trim):

- `<TODO>` duration, frame count, IMU rate/gaps
- `<TODO>` tag-detection sweep (mean/median tags per frame, % frames ≥15 tags)
- `<TODO>` defocused-prefix seconds (if any) to pass as `--trim-head`

## What's in this folder (once recorded)

| file | meaning |
|---|---|
| `video.mkv` | MJPG-sourced H.265, 2048x1536@30, **already rotated 180°** (recorded orientation, same convention as the IMX219 rig) |
| `frame_times.csv` | `frame_idx,unix_time` — row n ↔ video frame n |
| `imu.csv` | `stamp_ros,recv_unix,wx,wy,wz,ax,ay,az` (rad/s, m/s²) |
| `attitude.csv`, `telemetry.csv` | not needed for calibration |
| `meta.json` | note `frame_rotation_deg: 180`, `purpose: calibration`, real `width`/`height`/`imu_requested_hz` |
| `prepare_kalibr_input.py` | copy from `tools/prepare_kalibr_input.py` (kept camera-agnostic there) — converts the above into Kalibr input (step 2) |

## Hard rules

1. **Never rotate/flip the images.** The 180° rotation is already baked into
   video.mkv, and the flight-time OpenVINS feed uses the same orientation.
   Calibration must match it.
2. Frame↔timestamp mapping is *by index* (frame n ↔ csv row n). Don't re-extract
   frames with anything that can drop frames silently; use the provided script (it
   verifies the count).
3. Requirements: `docker`, `ffmpeg`, `python3`. A few GB free disk for PNGs + a
   similar amount for the rosbags. No ROS needed on the host (Kalibr runs in Docker).

## Step 1 — measure the printed grid, fill target.yaml

Run step 2 first if `target.yaml` doesn't exist yet (the script writes a template),
then **measure the physical print with a ruler** and edit:

- `tagSize`: edge of one black tag square in metres (nominal 0.088),
- `tagSpacing`: (gap between adjacent tags) / tagSize (nominal 0.3, i.e. gap ≈ 26.4 mm).

Printers rescale; measured values are required, ask the user to measure if needed. A
wrong absolute size mostly rescales the extrinsic translation, but there is no reason
to accept that error.

## Step 2 — convert to Kalibr input

```bash
cd field_data/calib_<SESSION_ID>
cp /path/to/repo/tools/prepare_kalibr_input.py .
python3 prepare_kalibr_input.py --trim-head <TODO seconds, verify against this recording>
```

`kalibr_input_full/` = all frames after the trim @30 fps (for the imu-camera stage);
`kalibr_input_4hz/` = every 8th frame (~3.75 Hz, for the intrinsics stage — full rate
would make it needlessly slow). Both contain the same `imu0.csv` (ns timestamps).

**Before running Kalibr**, set `imu.yaml`'s `update_rate` to this recording's real
achieved IMU rate (`imu_rates.json`'s `imu_hz`), which the script does NOT know and
leaves at a 200.0 placeholder.

## Step 3 — Kalibr via Docker

```bash
git clone https://github.com/ethz-asl/kalibr.git
cd kalibr && docker build -t kalibr -f Dockerfile_ros1_20_04 .   # once, ~15 min

FOLDER=/absolute/path/to/calib_<SESSION_ID>
docker run -it -v "$FOLDER:/data" kalibr
# ---- everything below runs INSIDE the container ----
source devel/setup.bash
cd /data/kalibr_input_4hz  && rosrun kalibr kalibr_bagcreater --folder . --output-bag /data/calib_4hz.bag
cd /data/kalibr_input_full && rosrun kalibr kalibr_bagcreater --folder . --output-bag /data/calib_full.bag
```

(No X11/display needed: pass `--dont-show-report` below; the PDF reports are written
to disk regardless.)

## Step 4 — camera intrinsics

```bash
cd /data
rosrun kalibr kalibr_calibrate_cameras --bag calib_4hz.bag \
    --topics /cam0/image_raw --models pinhole-radtan \
    --target target.yaml --dont-show-report
```

**Accept if:** mean reprojection error **< 0.5 px**; distortion coefficients non-zero
but sane (|k1|,|k2| < 0.5 for this lens class). **Sanity anchor for THIS camera**
(computed from Appropho's IMX900 sensor spec + the 4mm lens's HFOV≈59.9°/VFOV≈46.7°,
at 2048x1536 — see memory `camera-ap-imx900-revert`):

```
fx ≈ 1777    fy ≈ 1779    cx ≈ 1024    cy ≈ 768
```

Kalibr's result should land within roughly ±10-15% of this (looser than the IMX219
session's anchor, since this FOV was computed from a datasheet, not independently
measured — a real datasheet for the 4mm lens was never found). A wildly different
focal length (e.g. off by 2x+) means something is wrong (target.yaml sizes, wrong
topic, rotated images) rather than just "the FOV guess was a bit off."

Output: `camchain-calib_4hz.yaml` + report PDF + results txt.

## Step 5 — camera-IMU extrinsics + time offset

```bash
rosrun kalibr kalibr_calibrate_imu_camera --bag calib_full.bag \
    --cam camchain-calib_4hz.yaml --imu imu.yaml \
    --target target.yaml --dont-show-report
```

Takes a while (an hour+ is normal for a few thousand frames). **Accept if:**

- reprojection error < 1 px; gyro/accel error plots look like white noise (visible
  structure = insufficient excitation → would need a re-record);
- estimated time offset is ms-scale and stable in the report (not drifting);
- `T_cam_imu` rotation ≈ the known mounting, camera looking straight down,
  image-top = body-forward. This is the **same physical camera body and mount
  location** as the IMX219-era rig (only the lens changed), so the IMX219 session's
  accepted rotation is a reasonable anchor to expect again:

  ```
  [ 0 -1  0 ]
  [-1  0  0 ]
  [ 0  0 -1 ]
  ```

  (R camera→IMU/FLU-body, i.e. the transpose of Kalibr's imu→cam rotation.) Expect
  roughly this ± a few degrees. If it looks like a completely different axis
  permutation, the frames were flipped somewhere — stop and re-check rule 1.
- **translation magnitude**: the IMX219 session measured |t| = 0.358 m for this
  airframe and Frank confirmed it physically correct at the time. Since this is the
  same body/mount location, expect something similar (~0.30-0.40 m) — **but the new
  CS-mount lens housing is physically larger than the old M12 lens, so the optical
  center may have shifted somewhat; get a real ruler measurement of the current setup
  before calibrating (see Phase 0 of the calibration plan) and treat 0.358 m as a
  rough prior to check against, not a hard target.**

Outputs: `camchain-imucam-calib_full.yaml`, `imu-calib_full.yaml`,
`results-imucam-calib_full.txt`, report PDF.

## Step 6 — what to bring back to the Jetson

Copy back into this session folder on the Jetson (`field_data/calib_<SESSION_ID>/kalibr_output/`):

- `camchain-calib_4hz.yaml` (intrinsics)
- `camchain-imucam-calib_full.yaml` (extrinsics + time offset)
- `imu-calib_full.yaml`
- both `results-*.txt` and report PDFs

Next step there: write these into a new OpenVINS config directory (mirroring
`~/openvins_ws/config/survey17_kalibr/`'s structure — `estimator_config.yaml`,
`kalibr_imucam_chain.yaml`, `kalibr_imu_chain.yaml`), then re-run the exact same
evaluation already done for the uncalibrated approximate config:

```
~/openvins_ws/build-ov/run_video_msckf <new config>.yaml \
  field_data/survey42/video.mkv field_data/survey42/frame_times.csv field_data/survey42/imu.csv \
  field_data/survey42/vio_eval/vio_full_survey42_kalibr.csv <start> <end> 2
```

on survey38 (10m), survey41 (65m — pick a window avoiding the known 7.3s stream-stall
gap at t≈150.5s, or segment around it), and survey42 (100m), then recompute RMSE with
the same fixed-4DOF alignment method as `field_data/vio_agl_comparison.py` and compare
against both the old IMX219 baseline (survey31: 3.0m@10m, survey32: 75.8m@100m) and
this session's broken uncalibrated numbers
(`field_data/vio_globalshutter_comparison_result.json`). This is the step that
actually answers "does the global-shutter camera fix VIO."

## Known gotchas (carried over from the IMX219 session, likely still apply)

- `kalibr_calibrate_cameras` needing more init iterations on wide-margin frames is
  normal; if it fails to initialize focal length, retry with `--approx-sync 0.04`.
- If validating detections yourself with OpenCV instead of Kalibr: aruco
  `DICT_APRILTAG_36h11` needs `markerBorderBits=2` for Kalibr aprilgrids (default 1
  detects nothing).
- If the board print is glossy, a few frames may have specular glare washing out
  tags — Kalibr simply skips unreadable corners, not fatal unless most frames are hit.
- imu0.csv timestamps and frame timestamps share the same epoch (unix ns).
  `kalibr_bagcreater` accepts them as-is; do not re-zero them.
- **New for this lens (untested)**: if the CS-mount lens is adjustable-focus (unlike
  the old fixed-focus M12 lens), make sure focus is locked at a normal working
  distance BEFORE recording and not touched again afterward — refocusing changes the
  intrinsics and invalidates the calibration.
