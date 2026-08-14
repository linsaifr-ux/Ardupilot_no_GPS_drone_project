# vio_vpe/ — Live VIO+VPE shadow-mode observer

Runs the OpenVINS-based VIO + SuperPoint/LightGlue VPE + 4-state Kalman fusion
pipeline **live, during an actual flight**, so the pilot/ground crew can watch
the estimate and its error against GPS in real time — **without ever
publishing anything to the flight controller.** The pilot flies manually
(RC/GCS) exactly as today; this stack is a passive observer that could be
killed at any point with zero effect on the aircraft. See
`fusion_live_node.py`'s header for the enforced invariant (verify with
`grep -n "vision_pose\|mavlink_ctrl\|ardupilot_commander" vio_vpe/*.py` — must
return nothing but the invariant's own doc comment).

The method this implements is `instructions/METHOD_imx900_vio_vpe.md`. The
constants (Kalibr calibration, fusion filter gates, VPE map) all come from
that doc and `instructions/jetson_deploy/` (the desktop-generated deploy
package this was built from — see "Provenance" below).

---

## Quick start

```bash
bash vio_vpe/launch_shadow_mode.sh
```

No flags needed for a normal run — it streams to the project's standard RTSP
relay (`118.232.160.227`) **and** an OpenHD ground station (`192.168.2.2`)
simultaneously by default, and starts YOLO for ground-crew situational
awareness alongside the localization stack. See `launch_shadow_mode.sh`'s own
header comment for every override flag
(`--stream-server`/`--stream-host`/`--openhd-ip`/`--no-openhd`/`--no-stream`/`--no-yolo`/`--sigma-vio`/`--sigma-vpe`/`--map`).

**Do not also run `tools/record_field.py`** — it opens the camera device
directly and will fight `usb_camera_node.py` for it, same constraint as every
other camera-topic-subscribing tool in this project.

Stop with Ctrl+C, or a plain `kill <pid>` from another terminal — both work
cleanly (see "Shutdown" below for why that needed a real fix, not just a
`kill`).

---

## Architecture

```
/mavros/imu/data_raw ──┐
/drone/camera/image_raw┼──> [vio_live_node, C++/rclcpp] ──> /vio/odom (yaw-arbitrary)
                        │
/drone/camera/image_raw┼──> [vpe_live_node, Python/rclpy] ──> /vpe/fix
/drone/agl              │        PrebuiltVpeLocalizer(maps/imx900_survey47.vpemap)
/mavros/global_position/compass_hdg
/fusion/state (prior, for search_radius) ◄────────────────────┘

/vio/odom, /vpe/fix ──> [fusion_live_node, Python/rclpy]
    - online yaw fit (VIO->ENU) from VPE fixes only, never GPS (yaw_align.py)
    - FusionFilter.predict/update_velocity/update_position/output(dt)  [unmodified]
    - publishes /fusion/state (prior for vpe_live_node's search radius)
    - ground truth for the live error number: /mavros/global_position/global
      (raw GPS), never /drone/pose — keeps the number independent of whichever
      source EKF3 currently has active
    - writes logs/shadow_<ts>.csv (every field, every 20 Hz tick) and
      latest_estimate.json (polled by tools/ground_view_stream.py's overlay)
    - NEVER calls anything MAVROS-vision-facing
```

---

## Files

| file | role |
|---|---|
| `imx900_geo_common.py` | ENU origin + camera constants (K_GSD, distortion, mount) — from Kalibr, matches the map's own constants (fatal mismatch check on load) |
| `imx900_frame_map.py` | tile prep (`prepare()`, undistort+north-up+common-GSD) — the map-*building* half (`sample_frames`/`build_tiles`) is desktop-only, not used at runtime here |
| `vpe_localize_imx900.py` | SuperPoint+LightGlue matching; `PrebuiltVpeLocalizer` is the deployment-time entry point (no video, no map build, just load-and-query) |
| `fusion_filter.py` | 4-state horizontal KF, gates, reject-run escape hatch, the publish gate (`output(dt)`) — unmodified from the deploy package, every constant in it is a measured lesson, don't retune without new data |
| `vio/run_video_msckf.cpp` | offline runner (unmodified, kept for regression testing against `session_2026-08-12_imx900_vio.md`'s numbers) |
| `vio/run_video_msckf_live.cpp` | **new** — live rclcpp node, same OpenVINS feed logic as the offline runner (including the `frame_time+0.10s` IMU lookahead), subscribes IMU/camera topics instead of files, publishes `/vio/odom` |
| `vio/CMakeLists.txt` | builds both VIO binaries against `~/openvins_ws`'s already-built ARM OpenVINS install (not a colcon `install/` tree — see the file's own comments for why) |
| `vio/config_imx900/` | estimator + camera + IMU chain configs — `use_imuavg`/`use_rk4int` added vs. the original deploy package (this Jetson's OpenVINS build requires them; missing them makes both VIO binaries refuse to start) |
| `vpe_live_node.py` | **new** — live VPE ROS2 node, wraps `PrebuiltVpeLocalizer`, search radius driven by `/fusion/state`'s covariance (3σ, floor 35 m) |
| `yaw_align.py` | **new** — online VIO→ENU yaw fit from a trailing window of VPE fixes, closed-form (same math as `~/openvins_ws/compare_vio_gps.py`'s `yaw_align()`), gated on spatial span (150 m default) not elapsed time |
| `fusion_live_node.py` | **new** — the shadow-mode core; owns the invariant that nothing here ever reaches the flight controller |
| `launch_shadow_mode.sh` | **new** — full-stack launcher; see "Shutdown" below for its signal-handling design |
| `replay_flight.py` | **new** — replays a recorded flight (`field_data/<survey>/`) onto the real live topics at real-time pace, for full-stack testing without a real aircraft |
| `maps/imx900_survey47.vpemap` | prebuilt map: 68 tiles, descriptors + geometry only (70 MB, no imagery) — built from survey47 at ~56 m AGL |
| `logs/` | `shadow_<ts>.csv` (fusion), `vpe_live_<ts>.csv` (every VPE query attempt, fixed or not), `vio_live_<ts>.csv` (VIO state, only once initialized) — gitignored |

---

## Provenance

`instructions/METHOD_imx900_vio_vpe.md` documents this pipeline, but the code
it describes wasn't in this git repo or anywhere on this Jetson — it turned
out to live at `instructions/jetson_deploy/`, an untracked "desktop → Jetson
deploy package" folder (see that folder's own README: *"copy THIS folder"*).
`vio_vpe/` is that package's four Python files + the VIO C++ runner + the map,
copied in and **committed to git** (unlike `jetson_deploy/`, so this doesn't
happen again), plus the live-streaming layer the deploy package explicitly
didn't include (its README calls that "ground tooling" — the live driver
genuinely didn't exist yet, even in SITL, per the method doc's own §10.5).

---

## What's proven vs. what isn't

Validated on this Jetson, on real hardware, this session:

- **OpenVINS ARM build**: already existed at `~/openvins_ws/build-ov` (from
  earlier work), proven compatible with the new live node's build — both link
  and run against the same `libov_msckf_lib.so`.
- **VIO regression**: offline run on survey43 reproduces the method doc's
  numbers (0.80 m 2D rmse vs. the doc's 1.1 m).
- **VPE accuracy**: real single-frame query against the real map, 3.8 m error
  vs. GPS; radius-gated search gives a measured 4.7x speedup (779 ms vs.
  3.7 s) with the identical result.
- **Fusion + yaw-align integration**: direct-injection test with real VIO
  output + synthetic perfect VPE fixes converges to near-zero error, and the
  fitted yaw (15.4°) exactly matches the independent batch reference
  (`compare_vio_gps.py`).
- **Full-stack real-time replay** (survey43 and survey48, via
  `replay_flight.py`): all three nodes running concurrently over real ROS2
  topics, no crashes. On survey48 (large enough flight to cross the 150 m yaw
  span threshold) the online yaw fit resolved live and matched the batch
  reference; fused output (6.2 m mean vs. GPS) stayed close to VPE's own
  accuracy even while VIO's raw integrated position was badly degraded
  (75 m mean) by a replay-harness CPU-contention artifact (see
  `replay_flight.py`'s own header) — the fusion architecture's core claim
  (don't trust VIO's integrated position, only its short-window velocity
  between fixes) held up under a genuinely adverse live input.
- **`launch_shadow_mode.sh` end-to-end on real hardware**: real Pixhawk 6C /
  ArduCopter 4.6.3 hexacopter, real AP-IMX900 camera, three full start/stop
  cycles. Vehicle confirmed unarmed throughout every run.
  `/mavros/vision_pose/pose_cov` confirmed **0 publishers** via
  `ros2 topic info` — direct proof nothing in the stack ever touches the FC.

**Not yet shown:**
- Live VIO velocity actually improving the fused estimate over VPE-alone, in
  a clean (non-CPU-contended) run — the replay harness's own load was heavy
  enough to starve VIO's IMU delivery on the survey48 test.
- Any real flight. Every test above is either offline/replay or a stationary
  bench run (real hardware, real FC connection, vehicle never left the
  ground). VIO cannot initialize without real flight motion (`no accel jerk
  detected` loops forever while stationary — this is the intended safety
  behavior, not a bug).

---

## VPE's AGL behavior (as of this writing, deliberately left alone)

`vpe_live_node.py` has **no AGL gate** — it attempts a query at any AGL,
including near-zero. It'll only get *good* fixes once AGL is in roughly the
range the map was built at (~50–56 m, matching survey47's flight altitude);
below that, expect it to spend 1–4 s of GPU time per query mostly returning
"no fix," since cross-altitude matching degrades sharply (method doc: 63% fix
rate at 9 m query vs. 65 m map). Adding a `--min-agl` gate (mirroring
`imx900_frame_map.py`'s own `--min-agl 25` convention used to build the map)
would be the natural fix if this needs addressing later.

---

## Shutdown — why `launch_shadow_mode.sh`'s cleanup is staged

Found by testing, not by inspection — a plain `kill <launcher-pid>` on the
first working version of this script left `mavros_node` and
`ground_view_stream.py` running indefinitely:

1. **Foreground exec doesn't forward signals to grandchildren.**
   `fusion_live_node.py` originally ran as a true foreground command (no `&`);
   `launch_mavros_real.sh` runs `ros2 run mavros mavros_node` the same way one
   level down. A signal to the wrapper script doesn't reach a foreground
   grandchild. Fixed by backgrounding every stage and `wait`-ing on the last
   one specifically (not a bare `wait`) — see `set -m` and the `PIDS`
   array/process-group kill in the script.
2. **rclpy-based nodes here only act on SIGINT, not SIGTERM.** SIGINT is what
   maps to Python's `KeyboardInterrupt`, which is what every node's own
   `except KeyboardInterrupt:` shutdown path actually catches — a bare
   SIGTERM left `ground_view_stream.py` running with no visible error. Fixed
   with a staged **SIGINT → SIGTERM → SIGKILL** in `cleanup()`, each sent to
   the process *group* (`kill -- -$pid`) so it reaches subprocess trees like
   MAVROS's, not just the tracked top-level PID.

Verified: a single plain `kill` on the launcher now stops all 9 real
processes (MAVROS + its `ros2 run`/`mavros_node` children, camera, ground view
stream, hw_bridge, VIO, VPE, YOLO, fusion) and fully releases the camera and
serial devices, in under 8 seconds, with no traceback.
