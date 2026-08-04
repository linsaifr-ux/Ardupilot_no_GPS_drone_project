# survey32 + survey33 — same-domain database whole-pipeline test (2026-07-27)

This is the first real **cross-session** test of the same-domain-AnyLoc-database direction
recommended in `instructions/vpe_jump_runaway_diagnosis.md` §14-25 / memory `vpe-jump-runaway`:
build the AnyLoc database from one flight's own footage, then query it with a **separate**
flight over the same site, instead of splitting one flight's frames into db/query (which had
only been tested same-day/same-lighting before).

**Scope: cruise only, throughout this entire document.** This project's real deployment plan
is pilot-manual takeoff and landing (GPS available for both) — the GPS-denied pipeline only
needs to work during the cruise segment of a flight. Every test, evaluation window, and number
in this document is deliberately restricted to each flight's 100m-AGL cruise segment
(survey32: t=116.0–197.2s) — not the full recorded flight (which includes climb, descent, and
ground time only because the recorder happened to be running through them). Where a "full
flight" number appears (e.g. section 5, section 10), it is included only for context/contrast,
never as a claim about this pipeline's expected real-world performance during climb/landing.

## Flights

Both flown 2026-07-27 at the survey25 site, ~15 minutes apart, same camera/IMU rig (no remount
since the 2026-07-23 Kalibr calibration).

| | survey33 (mapping) | survey32 (test) |
|---|---|---|
| Purpose | build the AnyLoc database | query the database |
| Duration | 500 s | 326 s |
| Pattern | diagonal multi-leg lawnmower grid | single/short pass |
| Cruise AGL | ~150–390 s @ ~100 m | ~116.0–197.2 s @ ≥97 m (tightened from an initial ~110–210 s pass) |
| Ground speed (cruise) | ~7.5 m/s | ~4.7 m/s |
| Ground-track bbox | ~206 × 342 m | ~104 × 174 m, confirmed **nested inside** survey33's coverage |

## 1. Database build (Option B / real-footage workflow, unmodified)

```
tools/extract_frames.py field_data/survey33/ --rotate --min-dist 15 --min-agl 90
  -> 112 frames, field_data/survey33/frames.csv

anyloc/build_database_real.py field_data/survey33/ --db-dir anyloc/database_survey33_vits14
  -> 112-entry database, dinov2_vits14 / VLAD, database_survey33_vits14/
```

## 2. Pure AnyLoc retrieval accuracy (no VIO, no fusion — raw per-frame retrieval only)

`anyloc/test_accuracy_survey25_time.py` (fully generic despite the name — no survey25-specific
hardcoding) queried with survey32's own frames against `database_survey33_vits14`, `--rotate`,
2 s cadence:

| Window | n | mean | median | rmse | min | max |
|---|---|---|---|---|---|---|
| t=110–210 s (initial pass) | 51 | 24.13 m | 17.37 m | 37.91 m | 1.70 m | 152.54 m |
| t=116–197.2 s (tightened, cruise-only) | 41 | 25.13 m | 15.23 m | 40.57 m | 1.70 m | 152.54 m |

Context: same-flight same-domain control (survey25) was ~8.6 m; satellite-tile database was
~286 m. This cross-session same-domain result (~24–25 m) sits between the two, as predicted —
worse than same-flight (real cross-session degradation), still far better than satellite.

Two persistent outliers (~150 m, t≈186–188 s) were checked against heading — not a sharp-turn
event (heading stable ~104–106° through that window, unlike the turn-pathology cases seen
elsewhere in this project) — mechanism unresolved but minor relative to the overall result.

Files: `anyloc_vs_survey33_db.json` (initial pass), `anyloc_vs_survey33_db_cruise.json`
(tightened cruise-only pass).

## 3. OpenVINS crash found + fixed (blocking issue, resolved same session)

`~/openvins_ws/build-ov/run_video_msckf` crashed deterministically on **every** invocation
(glibc heap-corruption asserts, e.g. `malloc.c:2617 sysmalloc`, `malloc(): invalid size
(unsorted)` — message varied by input, classic ABI-mismatch signature) — including a re-run of
a previously-working survey25 command, so this was a real toolchain regression, not a
survey32-data problem.

**Root cause:** the executable was last linked 2026-07-22, but `libov_msckf_lib.so` (the shared
library it dynamically loads) was rebuilt 2026-07-24 23:54 for the gyro-predicted-tracking work
(`TrackBase.h` class-layout change, see memory `openvins-first-offline-run` / `vpe-jump-runaway`
§14-17). `run_video_msckf.cpp`'s `make_shared<VioManager>(...)` bakes in `sizeof(VioManager)` at
**compile time** to size its shared_ptr control-block allocation — the stale object file had the
old (smaller) size baked in, but the constructor that actually runs comes from the newer, larger
class in the rebuilt library. Under-allocates, then heap-corrupts the first time a real
allocation happens deep inside (surfaced inside `cv::setNumThreads` → OpenCV's
`ParallelBackendRegistry` singleton init — several frames away from where the corruption
actually originates).

**Fix:** recompiled `run_video_msckf.cpp` and relinked, using the exact compile/link commands
copied from CMake's own cached `flags.make` / `link.txt` (a real `cmake`/`make` reconfigure is
broken in this shell — it re-discovers stale `ament_cmake_DIR` cache entries from a prior
ROS-visible configure regardless of env-var stripping, hitting the already-documented ROS2/
ament landmine; copying the compile/link lines directly sidesteps `cmake_check_build_system`
entirely). Just relinking the stale object file was **not** sufficient (identical crash) — the
`.cpp` itself had to be recompiled against the current headers.

**Verified fixed:** re-ran survey25 t=100–300s — 2821 rows, 0 NaNs, 23.1 fps, `cam_dt` converges
to -0.045…-0.056 s (matches the Kalibr calibration's known value).

**Also flagged:** `run_video_msckf_aglprior` was built 2026-07-24 23:54:37 (1s after the library
rebuild) — safe. `run_video_msckf_gyrogate` (2026-07-23) and `run_video_msckf_gyrogate_soft`
(2026-07-24 00:10) both **predate** the library rebuild and are equally suspect — not yet fixed,
recompile+relink both the same way before trusting any future run of either.

## 4. Real OpenVINS VIO trajectory for survey32

```
~/openvins_ws/build-ov/run_video_msckf ~/openvins_ws/config/survey17_kalibr/estimator_config.yaml \
  field_data/survey32/video.mkv field_data/survey32/frame_times.csv field_data/survey32/imu.csv \
  field_data/survey32/vio_eval/vio_full_survey32.csv 65 326 2
```

Static init at t=65 s (clear of the 0–75 s ground window, just before the ~75–80 s liftoff).
Config is the Kalibr-calibrated camera/IMU rig (`survey17_kalibr`) — not survey-specific despite
the directory name, valid for any flight on the same unremounted rig.

Result: 3788 rows, 0 NaNs, `cam_dt=-0.0351`. Trajectory diverges heavily by the end (~8.2 km from
origin at t=326s) — this is the project's already-documented cruise-scale-collapse behavior
(monocular VIO scale drift, unrelated to today's fix), not corruption — physically plausible
values throughout, not garbage.

File: `vio_full_survey32.csv`.

## 5. Fusion corrector (VIO + real AnyLoc anchors)

> **Superseded by section 10 below** — the numbers in this section used the corrector's
> then-default `anchor_win=40` (no slew limiter), and a real 27.7x divergence spike was later
> found and fixed (`anchor_win=20` + slew limiter is now the official offline config, cruise
> RMSE 27.3m/max 80.7m). Kept as the original record; see section 10 for the current numbers,
> the diagnosis, and why the live closed-loop path deliberately keeps a *different* config.

`field_data/survey25/vio_eval/foundloc_corrector.py`'s causal loop (DBSCAN false-positive
filtering + gyro-heading-assisted frame lock + arc-length scale correction + anchor pull),
reused via `foundloc_corrector_survey32.py` — a pure path-substitution copy (only the
module-level `S` path changed, diffed to confirm nothing else differs) — fed with the **real**
AnyLoc anchors from step 2 (not simulated). Verification-first, matching this project's
established discipline: the real-anchor pipeline was run **twice** and required byte-identical
output before trusting any number — passed both times.

### Using the initial 51-anchor pass (t=110–210s)

| Window | Fused (VIO+anchors) | Raw VIO alone |
|---|---|---|
| 100–220 s | rmse 58.3 m, max 184.2 m | rmse 89.1 m, max 153.5 m |
| 65–326 s (full flight) | rmse 762.0 m, max 2061.0 m | rmse 2133.0 m, max 7727.2 m |

### Using the tightened 41-anchor, cruise-only pass (t=116–197.2s)

| Window | Fused (VIO+anchors) | Raw VIO alone |
|---|---|---|
| 116–197 s (cruise only) | rmse 69.3 m, max 170.7 m | rmse 75.6 m, max 121.6 m |

**Honest reading:**
- The full-flight comparison looks like a clear fusion win, but mostly because raw VIO
  catastrophically diverges (km-scale) once anchors stop and the scale-collapse takes over —
  fusion just bounds that divergence, it doesn't make the trajectory accurate.
- Restricted to cruise only, the picture is much more modest and mixed: fusion nudges rmse down
  (~9%) but makes max error *worse*. Two reasons: the corrector's default 40 s anchor warm-up
  window eats roughly half the 81 s cruise segment (only 21 of 41 available cruise anchors are
  actually used); and the one-time SE(2) alignment lock fires at t=66s, **before** cruise, off a
  near-degenerate/collinear window (`collinearity_ratio=4329`) — weak conditioning for the
  yaw/offset fit that the rest of the flight then inherits.
- Fused accuracy never approaches the pure-retrieval number alone (~24–25 m) in either pass —
  the corrector is smoothing/bounding a drifting VIO signal using periodic anchors, not
  replacing retrieval accuracy.

Files: `vio_full_real_anchor_survey32.csv` / `real_anchor_eval_result_survey32.json` (initial
pass), `vio_cruise_real_anchor_survey32.csv` / `real_anchor_eval_result_survey32_cruise.json`
(cruise-only pass), `real_anchor_eval_survey32.py`, `foundloc_corrector_survey32.py`, `agl.csv`.

## 6. Mosaic / GeoTIFF of the mapped area (survey33)

Built from survey33's 112 extracted frames (`tools/build_frame_mosaic.py`, new): each frame's
ground footprint sized from AGL + the IMX219's known FOV (62.2°×48.8°), placed on a local-ENU
canvas by GPS position, composited in flight order with **feathered alpha-over blending**
(soft edges, ~25 px / 3.75 m, full opacity in the interior) rather than a hard cut — a first
hard-cut version showed a harsh terraced/duplicated look between overlapping flight legs
(no gimbal on this camera + GPS/heading noise cause real frame-to-frame misalignment) which
feathering substantially cleans up, though it does not correct the underlying misalignment
(this is GPS/heading-placed pasting, not true feature-based photogrammetric orthorectification).

The flight itself is a diagonal multi-leg lawnmower grid, not aligned to north/south, so the
mosaic's north-aligned bounding canvas legitimately has empty (gray) corners outside the true
rotated coverage band — that is expected, not a defect.

No GDAL/rasterio in this environment. Georeferencing is a world file (`.pgw`) + `.prj` (WGS84
WKT) alongside the PNG (works directly in QGIS etc.), **and** a real embedded-metadata GeoTIFF
written via `tifffile` (pure Python + numpy, installed in the `anyloc` venv) with
`ModelPixelScaleTag`/`ModelTiepointTag`/`GeoKeyDirectoryTag` (EPSG:4326) — no GDAL needed for
either path.

Files: `field_data/survey33/mosaic.png`, `mosaic.tif`, `mosaic.pgw`, `mosaic.prj`,
`mosaic_meta.json`. New tools: `tools/build_frame_mosaic.py`, `tools/png_to_geotiff.py`.

## 7. Live, position-aware SITL closed-loop test (real AUTO mission)

> **Numbers below reflect the corrector config current at the time this section was written
> (anchor_win=40, no slew — the module default, before section 10's investigation existed).**
> Section 10 later tested applying section 10's offline fix (anchor_win=20 + slew) to THIS live
> harness too, found it made things 4-5x worse (a real, separate finding, not a bug), and the
> live path was kept at anchor_win=40/no-slew as a result — so the numbers below remain the
> current, correct, "kept" configuration's numbers, just re-verified with a fresh run in
> section 10 (34.0m/28.2m vs the 32.6m/24.0m here — within normal SITL run-to-run variance).

**Updated 2026-07-27 evening, after Frank confirmed real takeoff/landing are pilot-manual
(not part of the pipeline's job, see memory project-overview):** the mission's original LAND
item (and the disarm-hang bug it caused) has been removed. The mission now ends in
`MAV_CMD_NAV_LOITER_UNLIM` at cruise altitude instead of `MAV_CMD_NAV_LAND`, and completion is
detected by the mission reaching its final item (`MISSION_CURRENT.seq`), not by the vehicle
disarming (which never happens under LOITER, matching real deployment where a pilot takes over
for landing). Re-ran after the fix: completed cleanly in ~93s wall clock (no hang), top-level
summary now directly trustworthy with no manual window-filtering needed: anyloc_error_m
mean=32.8m/max=191.1m (n=44), localizer_error_m mean=24.0m/max=78.0m, EKF per-tick step
mean=0.81m/max=11.84m, **0 glitch(>50m) events** — consistent with the original (manually
windowed) result below, confirming that filtering was correct. The write-up below is kept as
the original record; treat this note as superseding its two flagged issues.

Retargeted copy of the survey25 live harness (`control/test_full_pipeline_sitl_live.py`) —
`control/test_full_pipeline_sitl_survey32.py` (mission/SITL base) +
`control/test_full_pipeline_sitl_live_survey32.py` (live version): builds a real AUTO mission
from survey32's own cruise track (t=116.0–197.2s → 6 waypoints), boots ArduCopter SITL with no
GPS, arms, climbs (GUIDED), hands off to AUTO, and flies it for real. At each 2s anchor tick it
finds the real survey32 frame nearest SITL's *current true position* (not by time), runs real
AnyLoc against `database_survey33_vits14`, feeds the result through `foundloc_corrector_survey32`'s
tick-by-tick logic, and publishes the corrected estimate as `VISION_POSITION_ESTIMATE`. Dead
reckoning between anchors is paced from survey32's own real OpenVINS trajectory
(`vio_full_survey32.csv`), not a proxy.

Mission uploaded and accepted (9 items), climbed, switched to AUTO, flew all 6 waypoints,
reached LAND at route_t=92.2s. 346 real AnyLoc queries fired, all accepted by DBSCAN filtering
(score mean 0.54, range 0.145–0.769). Bootstrap SE(2) alignment fired at t=20s (yaw=-62.2°).

**Restricted to the real active-mission window (t=0–92.2s):**

| Metric | Value |
|---|---|
| anyloc_error_m | mean 32.6 m, max 191.2 m (n=46) |
| localizer_error_m | mean 27.1 m, max 85.0 m |
| EKF per-tick step | mean 0.78 m, max 12.91 m |
| glitch (>50m) events | **0** |

This is the **first clean, no-glitch live-loop result** this project has produced — every prior
survey25 live-loop run had ~11 glitch events and thousands-of-meters mean error. Directly
attributable to survey32's route staying fully nested inside survey33's database coverage
throughout (SITL truth stayed within ~±44–145m x / -54–75m y of home — well inside the mapped
area) — the positive counterpart to the earlier survey25 finding that the pipeline does not
self-correct once it strays outside database coverage: staying inside coverage, the loop is
clean.

**Two flagged, NOT fixed issues:**
1. The vehicle never disarmed after reaching LAND — it sat frozen (armed, near-static position)
   for ~607s until a hardcoded 700s timeout (a literal copied verbatim from survey25's mission
   span inside `test_full_pipeline_sitl_live_survey32.py`'s `run_flight()`, never retargeted for
   survey32's much shorter route) cut it off. Whether the underlying disarm failure is
   pipeline-caused or an unrelated SITL/EKF issue is **unverified** — the numbers above use only
   the valid t=0–92.2s window specifically to avoid being diluted by this; the raw top-level
   summary the script prints is NOT reliable as-is (it averages in ~600s of stuck-hovering data).
2. The VIO increment queue (252s of material) ran out somewhere ~250–390s in — irrelevant to the
   valid window (ends at t=92s) but would need addressing (shorter queue is fine, or loop/hold
   behavior) before a longer survey32 SITL run is attempted.

Files: `control/test_full_pipeline_sitl_survey32.py`, `control/test_full_pipeline_sitl_live_survey32.py`,
`field_data/survey32/vio_eval/sitl/full_pipeline_sitl_live.json`, `postview_live.mp4`,
`postview/frame_*.png` (346 frames). See section 10 for the additional
`full_pipeline_sitl_live_anchorwin{20,40}_*.json` comparison-matrix files.

## 8. Offline postview replay (pure real data, no SITL)

> Regenerated against the final official offline config (section 10: `anchor_win=20` + slew
> limiter) — the fused/green path shown is the current, best offline result, not the
> section-5 numbers.

Separate from the SITL closed loop above: `field_data/survey32/vio_eval/postview_offline_survey32.py`
renders the same kind of 4-panel postview (path-so-far / camera frame / matched AnyLoc tile /
error numbers) but replays PURE survey32 data — no SITL, no simulated vehicle, no ArduPilot EKF.
"Truth" is survey32's own recorded GPS; the SITL version's "EKF" line is replaced with raw
(uncorrected) VIO, aligned into the GPS frame via the same fixed-4DOF (yaw+offset) alignment used
throughout this project's evaluation code, fit from the well-conditioned t=116–136s window and
applied to the full trajectory.

Final version plays the real camera video (`video.mkv`) continuously at native 30fps — output
video duration matches real elapsed flight time 1:1 (measured: 252.5s output for a 252.4s real
span, t=73.6–326.0s), with the path/tile panels refreshed every 2s (this project's real anchor
cadence) rather than every video frame. Outside the real anchor window (116.0–197.2s) the tile
panel honestly shows "not queried" (matching the real AGL≥50m gate) instead of a stale frozen
match, and `localizer_error_m` is shown continuously (interpolated every frame) while
`anyloc_error_m` only updates at real anchor ticks. Spot-checked: inside the anchor window
(t=123.6s) both errors track truth tightly (1.7m / 0.8m); well outside it (t=273.6s, post-cruise)
raw VIO and the fused trajectory have both drifted hundreds of meters — the same "no
self-correction once anchors stop" pattern found on survey25, now visible for survey32 too.
Camera-decode/rotate/compositing runs in plain OpenCV/numpy per video frame (~7500 frames) for
speed — matplotlib (slow) is only used to render the path graph once per 2s tick (~127 times).

Files: `field_data/survey32/vio_eval/postview_offline_survey32.py`,
`field_data/survey32/vio_eval/postview_offline.mp4`.

## Summary of new/reused files

| File | What |
|---|---|
| `anyloc/database_survey33_vits14/` | AnyLoc database built from survey33 |
| `field_data/survey32/vio_eval/anyloc_vs_survey33_db.json` | pure retrieval, t=110–210s |
| `field_data/survey32/vio_eval/anyloc_vs_survey33_db_cruise.json` | pure retrieval, t=116–197.2s |
| `field_data/survey32/vio_eval/vio_full_survey32.csv` | raw OpenVINS VIO trajectory |
| `field_data/survey32/vio_eval/foundloc_corrector_survey32.py` | fusion logic (path-copy of verified original) |
| `field_data/survey32/vio_eval/real_anchor_eval_survey32.py` | evaluation harness (both passes) |
| `field_data/survey32/vio_eval/vio_full_real_anchor_survey32.csv` | fused trajectory, t=110–210s anchors |
| `field_data/survey32/vio_eval/vio_cruise_real_anchor_survey32.csv` | fused trajectory, cruise-only anchors |
| `field_data/survey32/vio_eval/real_anchor_eval_result_survey32*.json` | numeric results, both passes |
| `field_data/survey32/vio_eval/agl.csv` | baro/AGL reference for the corrector |
| `field_data/survey33/mosaic.{png,tif,pgw,prj}` | visual + georeferenced mosaic of the mapped area |
| `tools/build_frame_mosaic.py` | reusable frame-mosaic builder (any survey) |
| `tools/png_to_geotiff.py` | reusable PNG/world-file → GeoTIFF converter |
| `control/test_full_pipeline_sitl_survey32.py` | AUTO-mission/SITL base, retargeted for survey32 |
| `control/test_full_pipeline_sitl_live_survey32.py` | live position-aware SITL closed-loop test |
| `field_data/survey32/vio_eval/sitl/` | SITL run log, postview frames/video |
| `field_data/survey32/vio_eval/postview_offline_survey32.py` | offline (no-SITL) real-time postview video builder |
| `field_data/survey32/vio_eval/postview_offline.mp4` | full-flight, real-time-paced offline postview |
| `field_data/survey32/vio_eval/vio_cruise_real_anchor_survey32_anchorwin40_orig.csv` | pre-fix fused trajectory (section 10) |
| `field_data/survey32/vio_eval/vio_cruise_real_anchor_survey32_anchorwin20_unslewed.csv` | intermediate (anchor_win=20, no slew) |
| `field_data/survey32/vio_eval/sitl/full_pipeline_sitl_live_anchorwin{20,40}_*.json` | 2x2 live-loop comparison matrix (section 10) |

## 9. `--stride 1` (full 30fps) experiment — not a free win

Checked whether OpenVINS was using full camera framerate (it wasn't — `stride=2`, ~15Hz feed,
this project's standard). Re-ran survey32 at `stride=1`: raw VIO got noticeably **worse**
(cruise-window rmse 75.8→258.4m, 3.4x) — plausibly a real monocular-VIO tradeoff (halving the
frame rate roughly halves inter-frame baseline/parallax, hurting triangulation) rather than a
bug. The fused result was somewhat better at stride=1 (69.2→55.6m), but this is one data point
with two contradictory signals — not a reason to change the project default away from stride=2.
Files: `vio_full_survey32_stride1.csv`, `vio_cruise_real_anchor_survey32_stride1.csv`.

## 10. Divergence diagnosis + fix: `anchor_win` + slew limiter (offline win, live-loop harm)

Prompted by watching the offline postview video: the fused (corrector) output stayed far worse
than AnyLoc's own raw retrieval for a long stretch, not just briefly. Tick-by-tick comparison
confirmed a real, non-trivial problem: at t=154s AnyLoc's own retrieval error was 5.8m but the
fused output was 160.0m off — **27.7x worse** — across the whole t=136–156s window, not a
one-off blip.

### Diagnosis

Tried `run_corrector_relock` (periodic re-alignment, already existed in the codebase from the
survey25 thread) first, across several relock-arc/window combinations — **did not help at
all**: the max error at t=154s was identical (160.0m) in every variant, a strong signal the
wrong mechanism was being adjusted. Re-reading the code found the real cause: the **scale**-
estimation update (not the rotation lock) needs a trailing `anchor_win` (default 40s) of
anchor history before it can compute a correction. Anchors only start at t=116s (cruise
onset), so that gate doesn't clear until t=156s — exactly where the divergence window ends.
`run_corrector_relock` only changes the rotation-refit cadence; the anchor_win-gated scale
logic is identical, unchanged code in both functions, which is why relock never touched it.

### Fix 1 — `anchor_win=20` (halved from the 40s default)

Cuts the blind spot in half (scale gate clears at t=136s instead of t=156s). Verified
(byte-identical determinism across repeated runs):

| Window | anchor_win=40 (orig) | anchor_win=20 |
|---|---|---|
| t=136–156s (problem window) | rmse 110.7m, max 160.0m | rmse 20.8m, max 37.0m |
| Cruise (116–197.2s) | rmse 69.2m, max 170.7m | rmse 36.5m, max 131.4m |
| Full flight (65–326s) | rmse 725.0m, max 1985.6m | rmse 712.6m, max 1962.0m |

No downside anywhere (full-flight is slightly *better*, not worse).

### Fix 2 — is the raw output smooth enough? No — slew limiter needed

Checked per-timestep position steps in the `anchor_win=20` output: real single-tick jumps up
to 65.4m (implied speed ~985 m/s), all landing exactly on anchor-tick moments — the "pull
toward anchor" is applied as one instantaneous CSV-row addition with zero rate limiting. This
would trip a real autopilot's `EK3_GLITCH_RAD=50m` gate exactly like this whole project's
original jump-runaway problem. Applied `control/vpe_slew.py`'s `VpeSlewLimiter` (already built
earlier in this project for exactly this) as a post-process:

| | anchor_win=20, unslewed | anchor_win=20 + slewed |
|---|---|---|
| Max single-tick step | 65.4m | **1.17m** (zero >5m steps) |
| Cruise RMSE | 36.5m | **27.3m** |
| Full-flight RMSE | 712.6m | **573.8m** |

Slewing improved accuracy further, not just smoothness — smoothing out the overshoot-prone
raw pulls reduces error; it wasn't a smoothness/accuracy tradeoff here.

**`anchor_win=20` + slew is now the official OFFLINE/batch config**, formalized in
`real_anchor_eval_survey32.py` (verification-first: both the corrector step and the slew step
are independently determinism-checked before any number is trusted).

### But: the same fix made the LIVE closed loop 4–5x WORSE

Tested the identical change in `test_full_pipeline_sitl_live_survey32.py`'s `LiveCorrector`.
Full 2×2 matrix, real SITL runs:

| Config (live closed loop) | anyloc err (mean) | localizer err (mean) | ticks / duration |
|---|---|---|---|
| anchor_win=40, no slew (kept) | 32.6–34.0m | 24.0–28.2m | 44–46 / ~92s |
| anchor_win=20, no slew | 34.4m | 30.8m | 49 / ~98s |
| anchor_win=20, + slew | 116.1m | 109.9m | 79 / ~158s |
| anchor_win=40, + slew | 131.8m | 123.1m | 75 / ~150s |

Slew is 4–5x worse regardless of `anchor_win`; `anchor_win` alone barely matters live. **Root
mechanism**: the offline evaluation has fixed truth independent of what gets published, so
slowing a correction only smooths the curve. In the live closed loop, the published position
drives the *real* vehicle (via EKF + position controller), and the *next* AnyLoc query is
chosen by nearest-*real*-position — so rate-limiting the correction lets real drift accumulate
further before being pulled back, and more drift feeds worse-matching queries (the same
mechanism as the already-documented §14-24 finding: once truth strays from the recorded track,
nearest-frame search returns increasingly unrelated imagery). This feedback loop has no analog
in the open-loop offline evaluation.

**Decision: the offline and live paths now deliberately use different configs.**
`real_anchor_eval_survey32.py` uses `anchor_win=20` + slew (best offline result).
`test_full_pipeline_sitl_live_survey32.py`'s `LiveCorrector` explicitly keeps `anchor_win=40`
and does **not** apply slew — documented in-code with the reasoning above, specifically so a
future session doesn't "fix" it back to match the offline config without re-reading this.

Files: `vio_cruise_real_anchor_survey32_anchorwin40_orig.csv`,
`_anchorwin20_unslewed.csv` (offline intermediates); `real_anchor_eval_survey32.py` (official
offline eval, now includes smoothness checks); `full_pipeline_sitl_live_anchorwin40_noslew_FINAL.json`
(kept live default), `_anchorwin20_noslew.json`, `_slew_anchorwin20.json`, `_slew_anchorwin40.json`
(2×2 comparison matrix). Offline postview video, `localizer_path_vs_gps.png`, and
`sitl_path_vs_mission.png` all regenerated against the final configs.

## Open questions / not done this session

- Real-time compute budget for the fusion corrector was not evaluated (offline run only).
- Only one site (survey25's) and one mapping/test flight pair has been tested — no evidence yet
  on a different site or a longer time gap between mapping and test flights.
- `run_video_msckf_gyrogate` / `_gyrogate_soft` are still stale post-library-rebuild and would
  need the same recompile+relink fix before use.
- The ~150 m retrieval outliers (t≈186–188s) have an unresolved mechanism (ruled out: sharp
  turns).
- ~~The corrector's cruise-only result might improve with a shorter `--anchor-win`~~ — TRIED
  2026-07-27, see section 10: fixed the offline divergence, but made the live closed loop
  worse — now split into different offline/live configs.
- Whether a gentler slew rate (lower than the default 2.5 m/s correction speed) could avoid
  the live-loop harm found in section 10 without giving up the offline smoothness win is
  untested — only the default rate was tried.
- The live-loop harm mechanism (slew's rate-limiting interacting badly with the
  position-aware feedback loop) is a plausible, evidence-supported explanation, not a
  first-principles proof — no direct instrumentation of "how far the real vehicle drifted
  before correction" was added to confirm it tick-by-tick.
- ~~The live SITL test's vehicle never disarmed after reaching LAND~~ — FIXED 2026-07-27
  evening: mission no longer includes a LAND item (see section 7 update).
- ~~The live SITL harness's `run_flight()` timeout is still a hardcoded literal~~ — FIXED
  same pass: both harnesses now use module constant `CRUISE_SPAN_S`/`base.CRUISE_SPAN_S`
  instead of a copy-pasted literal.
