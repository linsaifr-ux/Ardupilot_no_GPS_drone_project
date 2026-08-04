# survey25 (Flight 2, 100m AGL survey) — OpenVINS offline evaluation, 2026-07-23

> **Superseded/extended 2026-07-24**: this README documents the original
> `run_video_msckf` eval only (baseline cruise-scale collapse, 0.417x). It
> does NOT cover the later gyro-gate / AGL depth-prior / combined-mechanism
> work done on this same flight — see
> `session_2026-07-24_agl_prior_gyrogate_combined.md` in this folder and
> `instructions/vpe_jump_runaway_diagnosis.md` §14-12/§14-13 for that.
> Short version: gyro-gate wins the acute turn (rmse 13.9 m), AGL-prior
> alone is the best full-window result (539 m at 200-320s), combining both
> is worse than either alone; none survives the full multi-turn survey.
> Later the same evening: an output-space scale corrector (clean anchor
> proxy) got full-flight rmse to 76 m, then a FoundLoc-inspired version
> (DBSCAN false-positive filtering + gyro-assisted frame lock, tested with
> REALISTIC noisy anchors, not a clean proxy) landed at a more honest
> 137 m. See `session_2026-07-24_agl_prior_gyrogate_combined.md` "Round 2"
> and "Round 3", and diagnosis doc §14-14/§14-15.
> 2026-07-25: a same-domain control (AnyLoc DB built from this flight's own
> footage instead of satellite tiles) proved domain gap — not this
> project's pipeline code — is the dominant real-world error source
> (8.6 m same-domain vs 286 m best satellite-tile result, ~33x). A
> 100m-AGL-only retest of both databases was a clean null result (no
> meaningful change). The same-domain trace's apparent end-of-window
> coverage gap was traced to a stationary loiter before descent that the
> distance-triggered frame extractor couldn't sample — fixed with
> `tools/extract_frames.py --max-time-gap`, closing the tail drift (was
> 157-260 m, now 13-15 m). See "Round 9" and "Round 10" in
> `session_2026-07-24_agl_prior_gyrogate_combined.md` and diagnosis doc
> §14-20/§14-21.

Pre-notch baseline: `INS_HNTCH_ENABLE=0` confirmed via the FC's own `PARM`
records in survey24's `.bin` (same FC boot session, params untouched between
flights). 333 Hz `RAW_IMU` streaming was already active (`imu_hz=322.5` per
`meta.json`). Same Kalibr calibration as survey17 (`~/openvins_ws/config/
survey17_kalibr[_dyn]`) — no camera remount since that session, so it applies
here unchanged.

## Flight timeline (video-relative seconds, from telemetry.csv)

0-110 static (ground) — **note: real accel disturbance at 60-80s** (std
0.4-0.9 m/s², arming/prop check?), clean again 80-110s · 112-167 climb
0->100m AGL · 167-319 cruise legs @ ~100m AGL (~150s, several legs + a
~150 deg turn at 207-216s) · 319-375 descent · 375-471 low hover before land.

## Method

`~/openvins_ws/build-ov/run_video_msckf <config> video.mkv frame_times.csv
imu.csv out.csv <start_off> <end_off> <stride>`, stride=2 (~15 Hz feed),
`vio_stats.py`/`vio_final_plot.py` for 4-DOF (yaw+translation, optional
best-fit scale) alignment vs GPS ground truth.

**Two naive attempts failed for methodology reasons, not physics** — kept as
`vio_full.csv`/`vio_cruise.csv` for reference, superseded by `_v2`:
- `vio_full.csv` (static init @ t=0): false-triggered on the 60-80s ground
  disturbance instead of real liftoff (112s) — garbage init, km-scale error.
- `vio_cruise.csv` (dyn init @ t=160, right as cruise begins): almost no
  lateral acceleration excitation right there to observe gravity/scale —
  runaway divergence (>100 m error by t=190s, velocity growing to hundreds
  of m/s — classic small gravity-misalignment-at-init signature).

**Corrected runs (`_v2`) — the actual result:**
- `vio_full_v2.csv`: static init @ t=100 (just before real liftoff, clear of
  the ground disturbance).
- `vio_cruise_v2.csv`: dyn init @ t=200 (just before the 150 deg turn at
  207-216s, which gives real lateral acceleration to init from).

## Results

| Segment | Result |
|---|---|
| Climb (105-165s, static init, altitude 0->100m) | **horizontal rmse 1.4 m**, altitude tracks the baro/GPS AGL curve closely (see plot panel A) — best-fit XY scale ill-defined here (ground truth horizontal path is only ~2 m during a near-vertical climb) |
| Cruise onset (165-210s) | error grows 30 m -> 400 m within ~45 s of leveling off — every run, regardless of init point, breaks down almost immediately on entering level cruise |
| Cruise (dyn init @ turn, 200-230s, 230 m path) | 4DOF ATE2D rmse 42.9 m, **best-fit scale 0.417**, scale-corr rmse 33.3 m |

Plot: `vio_result_summary.png` (4 panels: climb altitude tracking, full-run
error growth on a log axis, cruise trajectory vs GPS, numeric summary).

## Interpretation

- **Climb divergence — survey17's headline failure mode — did not reproduce
  here.** Candidate explanation: 333 Hz `RAW_IMU` streaming (already enabled
  on this flight, independent of the notch) removes the un-antialiased
  400->200 Hz stream-decimation step identified as fix option 3 in
  `instructions/vpe_jump_runaway_diagnosis.md` §14-4 — consistent with that
  being a real contributor to the old climb-divergence failure. Not proven
  (n=1 flight, different day/site than survey17) but a strong hint.
- **Cruise scale collapse persists, and matches survey17 almost exactly**
  (0.417x here vs 0.34-0.48x on survey17's calibrated re-runs). Expected:
  ArduPilot's harmonic notch only touches the **gyro**, never the
  accelerometer (source-verified, §14-4 item 1) — monocular VIO scale is
  held by the accelerometer, so 333 Hz streaming and the pending gyro notch
  are not expected to fix this on their own. The batch-log FFT (§14-8) also
  confirmed the ~134 Hz vibration line is *below* the 333 Hz stream's
  Nyquist (166 Hz) — meaning it now arrives **unaliased but still present**
  in the accel signal at full strength, which is exactly the kind of noise
  that would still corrupt accelerometer-only scale.
- This dataset predates the notch (loaded onto the FC same evening, after
  these flights — see §14-8). It is the "333 Hz alone, no notch" data point
  the field plan (§14-7) asked for. The next validation flight (notch live)
  is what actually tests whether cruise scale improves; if it doesn't, the
  ranked-fix list's baro-depth-prior idea (§14-5) becomes the next lever
  specifically because it's an accel problem, not a gyro one.

## Files

- `vio_full.csv` / `vio_cruise.csv` — naive-init attempts (reference only,
  see caveats above)
- `vio_full_v2.csv` / `vio_cruise_v2.csv` — corrected-init runs, the actual
  result
- `vio_stats.py` — segment stats template (naive runs)
- `vio_final_plot.py` — final 4-panel plot + printed stats (`_v2` runs)
- `vio_result_summary.png` — the plot
- traj csv columns: t(unix), px..pz (m, world z-up), qx..qw (JPL q_GtoI),
  vx..vz, cam_dt, gyro/accel biases
