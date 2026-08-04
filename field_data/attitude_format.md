# `attitude.csv` format

Written by `tools/imu_logger.py` (the recording sidecar used by `tools/record_field.py`) for
every survey/calib recording in this directory. Source: `/mavros/imu/data`, requested via
MAVLink `ATTITUDE_QUATERNION` at 50 Hz — this is the flight controller's own fused attitude
estimate (its onboard EKF/AHRS output), not a raw sensor reading.

## Columns

```
stamp_ros,recv_unix,qw,qx,qy,qz
```

| Column | Meaning |
|---|---|
| `stamp_ros` | The timestamp the FC/MAVROS attached to the message itself (`msg.header.stamp.sec + nanosec`) — i.e. when the FC says this attitude was actually measured. Unix epoch seconds. **Use this one** for time-aligning against other sensors (GPS, VIO, video frames). |
| `recv_unix` | `time.time()` on the Jetson at the moment this logger process received and wrote the message. Unix epoch seconds. Useful for diagnosing pipeline latency (`recv_unix - stamp_ros` = MAVLink transmission + MAVROS processing + queueing delay), not for time-alignment. |
| `qw, qx, qy, qz` | The orientation quaternion — see below. |

## What the quaternion components mean

A quaternion encodes a 3D rotation as *how much* + *around what axis*, packed into 4 numbers:

- **`qw`** (the "real"/scalar part) encodes the rotation **angle**: `qw = cos(θ/2)`, where θ is
  how far the vehicle has rotated from the reference orientation.
- **`qx, qy, qz`** (the "imaginary"/vector part) encode the rotation **axis**, scaled by the same
  angle: `(qx, qy, qz) = (axis_x, axis_y, axis_z) × sin(θ/2)`, where `(axis_x, axis_y, axis_z)` is
  a unit vector along the axis being rotated around.

It's a unit vector in 4D: `qw² + qx² + qy² + qz² = 1`. Always sanity-check this on real data
before trusting it — a properly normalized quaternion should have magnitude ≈ 1.

Example: a pure 90° yaw turn (rotating only around the vertical/z-axis) is
`qw = cos(45°) ≈ 0.707`, `qz = sin(45°) ≈ 0.707`, `qx = qy = 0` — only `qz` is nonzero because the
axis is purely z.

**Why a quaternion instead of roll/pitch/yaw directly?** Euler angles (roll/pitch/yaw) have a
singularity (gimbal lock) where a degree of freedom is lost at certain orientations (pitch near
±90°); quaternions don't have this problem, and they compose/interpolate more cleanly for filter
math (FC EKF, VIO, etc.).

## Converting to roll/pitch/yaw

Standard aerospace ZYX Euler extraction from a Hamilton-convention quaternion:

```python
import math

def quat_to_rpy(qw, qx, qy, qz):
    roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    pitch = math.asin(max(-1, min(1, 2*(qw*qy - qz*qx))))
    yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return roll, pitch, yaw  # radians
```

This project's VIO/corrector code (`field_data/*/vio_eval/foundloc_corrector*.py`) uses a
yaw-only version of the same formula, `quat_yaw()`, to pull just the heading back out of a
quaternion without needing the full roll/pitch/yaw decomposition.

**Convention warning**: `attitude.csv`'s quaternion (MAVROS/`/mavros/imu/data`, Hamilton
convention) is a *different* convention from the VIO trajectory CSVs' quaternion elsewhere in
this project (JPL `q_GtoI`, noted in the corrector code comments) — don't mix the two without
converting; a JPL quaternion has the opposite sign convention on the vector part relative to
Hamilton for the same physical rotation.

## Example finding (survey32 cruise window, t=116-197s)

Quick sanity-check analysis run on this data (2026-08-04):

- Roll: mean -1.4°, std 4.7°, range -19.9° to +6.4°
- Pitch: mean **+5.6°**, std 8.9°, range -14.8° to +23.3°

The nonzero mean pitch (systematic forward tilt during cruise, expected for forward flight) is
worth keeping in mind wherever this project assumes a purely nadir-pointing camera (AnyLoc/VPR
footprint-size geometry, the depth≈AGL assumption behind the AGL/parallax analysis, NGPS-style
homography-based position estimates) — real tilt, especially the swings up to 23°, is a
simplification these all currently ignore. Not yet quantified how much this actually matters.
