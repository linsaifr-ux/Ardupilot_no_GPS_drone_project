#!/usr/bin/env python3
"""Convert a camera-IMU calibration recording into Kalibr input folders (run on the PC).

Run from inside a `field_data/calib_<timestamp>/` session directory recorded with
`record_field.py --calib`:

    python3 prepare_kalibr_input.py            # default: trim first 12.5 s
    python3 prepare_kalibr_input.py --trim-head 0    # no defocused-prefix to trim

Produces (in the session directory):
    kalibr_input_full/  cam0/<ns>.png @ recording fps + imu0.csv   -> imu-camera calib
    kalibr_input_4hz/   cam0/<ns>.png every 8th + imu0.csv          -> camera intrinsics
    target.yaml          AprilGrid definition  (EDIT: fill measured sizes!)
    imu.yaml              IMU noise model for kalibr_calibrate_imu_camera

This script is camera/session agnostic (no hardcoded resolution or session ID) — it
works on any `--calib` recording made with this project's `record_field.py`, whichever
camera body/lens was mounted at the time. Camera-specific sanity-check numbers (FOV
guess, mounting translation, resolution) belong in a per-session README, not here.

Notes that always apply to this project's recordings:
  - video.mkv frames are recorded with meta.json's frame_rotation_deg already baked in
    (currently 180 for both camera bodies used so far) — the runtime OpenVINS feed uses
    the same recorded orientation. Do NOT rotate or flip anything here or later.
  - frame n of video.mkv corresponds to row n of frame_times.csv. The script verifies
    this (ffmpeg frame count == csv row count) and aborts if it doesn't match.
  - the first few seconds of a handheld take are often a defocused close-up (camera
    resting near the board before it's picked up) with zero tag detections -> check
    your OWN recording for this (play the video / look at early tag-detection stats)
    before trusting the default --trim-head; it is NOT calibrated to this recording.
  - imu.csv stamp_ros is the FC-timesync ROS stamp, same epoch as frame_times.csv
    unix_time (offset sub-ms to ~ms). Kalibr still estimates the residual time offset.
    Units already rad/s and m/s^2 — no conversion needed.
Needs: ffmpeg, python3 (stdlib only). Peak disk use is roughly 2-3x the recording's
frame count in PNGs (a few GB for a ~2 minute take).
"""
import argparse, csv, os, shutil, subprocess, sys

SUBSAMPLE = 8          # recording fps / 8 -> the 4hz-ish intrinsics-only subset

IMU_HEADER = 'timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z'

TARGET_YAML = """\
# Kalibr AprilGrid definition — instructions/april_6x6_80x80cm_A0.pdf
# printed at 100% on A0. Values below are the NOMINAL 100%-A0 numbers.
# >>> MEASURE THE PRINT WITH A RULER AND OVERWRITE BEFORE CALIBRATING <<<
#   tagSize    = edge length of one black tag square, in metres (~0.088)
#   tagSpacing = (gap between two tags) / tagSize          (~0.3 -> gap ~26.4 mm)
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088        # TODO measure
tagSpacing: 0.3       # TODO measure
"""

IMU_YAML = """\
# Pixhawk-class generic IMU noise values (vpe_jump_runaway_diagnosis.md §13-E).
# Good enough for Kalibr. (The 5-10x inflation note applies to the OpenVINS
# runtime config later, NOT here.)
# >>> update_rate: set this to the ACTUAL achieved rate from this recording's
#     imu_rates.json ("imu_hz"), not the requested rate — check it before running. <<<
rostopic: /imu0
update_rate: 200.0
accelerometer_noise_density: 2.0e-3
accelerometer_random_walk: 3.0e-3
gyroscope_noise_density: 1.7e-4
gyroscope_random_walk: 2.0e-5
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trim-head', type=float, default=12.5,
                    help='seconds of video to drop from the start (default 12.5 — '
                         'VERIFY against your own recording, see module docstring)')
    args = ap.parse_args()

    for f in ('video.mkv', 'frame_times.csv', 'imu.csv'):
        if not os.path.exists(f):
            sys.exit(f'ERROR: {f} not found — run from inside the session directory.')

    with open('frame_times.csv') as f:
        rows = list(csv.DictReader(f))
    frame_ts = [float(r['unix_time']) for r in rows]
    t0 = frame_ts[0]
    print(f'frame_times.csv: {len(rows)} rows, span {frame_ts[-1]-t0:.1f}s')

    # --- extract every frame, then rename by frame index <-> csv row ---
    tmp = 'tmp_frames'
    if os.path.isdir(tmp):
        shutil.rmtree(tmp)
    os.makedirs(tmp)
    print('extracting frames (several minutes)...')
    subprocess.run(['ffmpeg', '-v', 'error', '-i', 'video.mkv', '-vsync', '0',
                    os.path.join(tmp, '%06d.png')], check=True)
    pngs = sorted(os.listdir(tmp))
    if len(pngs) != len(rows):
        sys.exit(f'ERROR: extracted {len(pngs)} frames but frame_times.csv has '
                 f'{len(rows)} rows — frame<->timestamp mapping is unsafe. '
                 f'Re-copy the session and retry, or check for a partial video.mkv.')

    full_cam = os.path.join('kalibr_input_full', 'cam0')
    sub_cam = os.path.join('kalibr_input_4hz', 'cam0')
    for d in (full_cam, sub_cam):
        shutil.rmtree(os.path.dirname(d), ignore_errors=True)
        os.makedirs(d)

    kept = sub = 0
    for i, (png, t) in enumerate(zip(pngs, frame_ts)):
        if t - t0 < args.trim_head:
            continue
        name = f'{int(round(t * 1e9)):019d}.png'
        dst = os.path.join(full_cam, name)
        os.replace(os.path.join(tmp, png), dst)
        kept += 1
        if i % SUBSAMPLE == 0:
            os.link(dst, os.path.join(sub_cam, name))
            sub += 1
    shutil.rmtree(tmp)
    print(f'cam0: kept {kept} frames (trimmed first {args.trim_head}s), '
          f'{sub} in the ~1/{SUBSAMPLE} subset')

    # --- imu0.csv (ns timestamps, gyro rad/s, accel m/s^2) ---
    n = 0
    with open('imu.csv') as fin, \
         open(os.path.join('kalibr_input_full', 'imu0.csv'), 'w') as fout:
        fout.write(IMU_HEADER + '\n')
        for r in csv.DictReader(fin):
            fout.write(f"{int(round(float(r['stamp_ros'])*1e9))},"
                       f"{r['wx']},{r['wy']},{r['wz']},"
                       f"{r['ax']},{r['ay']},{r['az']}\n")
            n += 1
    os.link(os.path.join('kalibr_input_full', 'imu0.csv'),
            os.path.join('kalibr_input_4hz', 'imu0.csv'))
    print(f'imu0.csv: {n} samples')

    for name, content in (('target.yaml', TARGET_YAML), ('imu.yaml', IMU_YAML)):
        if not os.path.exists(name):
            with open(name, 'w') as f:
                f.write(content)
            print(f'wrote {name}')

    print('\nDone. Next (see this session\'s KALIBR_PC_README.md):')
    print('  1. measure the printed grid, edit target.yaml')
    print('  2. set imu.yaml update_rate to this recording\'s real imu_rates.json value')
    print('  3. kalibr_bagcreater on kalibr_input_4hz/ and kalibr_input_full/')
    print('  4. kalibr_calibrate_cameras (4hz bag) -> kalibr_calibrate_imu_camera (full bag)')


if __name__ == '__main__':
    main()
