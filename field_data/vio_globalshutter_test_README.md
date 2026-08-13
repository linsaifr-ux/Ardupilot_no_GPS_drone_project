# Global-shutter camera VIO test (survey38/41/42) — 2026-08-11

Attempted answer to "does the reverted global-shutter AP-IMX900 camera (4mm CS-mount lens,
see memory `camera-ap-imx900-revert`) fix OpenVINS raw-VIO error?", by running the same
`run_video_msckf` binary/methodology used for the survey30/31/32 rolling-shutter AGL
comparison (`field_data/vio_agl_comparison.py`) on three new same-day flights:

| flight | AGL | purpose |
|---|---|---|
| survey38 | ~10m | flight test (compare vs survey31 @10m) |
| survey41 | ~65m | flight test (no rolling-shutter baseline at this AGL) |
| survey42 | ~100m | flight test (compare vs survey32 @100m) |

(survey39/40 were "for map build" flights, not touched here — lower priority per scope.)

## Result: INCONCLUSIVE — the filter diverges catastrophically, not a valid accuracy comparison

Raw output (`vio_globalshutter_comparison_result.json`, same fixed-4DOF-alignment method as
`vio_agl_comparison.py`, cruise windows found programmatically from the AGL curve):

| flight | AGL | groundspeed | RMSE | max |
|---|---|---|---|---|
| survey38 | ~10m | 3.10 m/s | 20325 m | 45412 m |
| survey41 | ~65m | 6.07 m/s | 20990 m | 42044 m |
| survey42 | ~100m | 5.86 m/s | 1753 m | 4061 m |

For scale, the old rolling-shutter (IMX219) baseline at matched AGL was 3.0m (survey31,
10m) and 75.8m (survey32, 100m) — i.e. these new numbers are 3-4 **orders of magnitude**
worse. This is not "the global-shutter camera is worse at VIO" — it's the estimator
diverging into physically-impossible trajectories (tens of km of drift within ~1-2 minutes,
horizontal velocity states run away to 8+ m/s within 5s of a gentle near-vertical liftoff).
**Do not read these numbers as evidence about shutter type.**

## Root cause: no real Kalibr calibration exists for this camera+lens body

`~/openvins_ws/config/` only has a Kalibr calibration for the IMX219 camera (removed
2026-08-09). The AP-IMX900+4mm-lens combo currently flying has never been through an
AprilGrid/Kalibr session. For this test, a best-effort approximate config was built
(`~/openvins_ws/config/survey38_ap_imx900_globalshutter/`):
- intrinsics: pinhole formula from the *computed* (not measured) FOV in memory
  `camera-ap-imx900-revert` (HFOV 59.9°/VFOV 46.7° at 2048x1536) → fx=1777.2, fy=1779.0,
  cx=1024, cy=768, distortion=0
- extrinsics: reused the real Kalibr `T_imu_cam` rotation+translation fit for the IMX219 rig
  (physically-measured mount position, |t|=0.358m, Frank-confirmed correct 2026-07-23) —
  assumed to still roughly apply since it's nominally the same airframe mount location
- `calib_cam_intrinsics/extrinsics/timeoffset: true` (online refinement on), everything else
  copied unchanged from `survey17_kalibr/`

**Diagnostic run performed to isolate the cause**: re-ran survey42 with all three
`calib_cam_*` online-refinement flags forced to `false` (frozen at the seed values) —
`~/openvins_ws/config/survey38_frozen_calib_diag/`, output
`vio_eval/vio_full_survey42_frozen_calib_diag.csv`. Result: **worse**, not better — position
drifts to `p_IinG ≈ (-27962, -33298, -9501)` m, 44.6 km from origin, by t≈229s. This rules out
"online refinement is destabilizing a good seed" — the seed itself (intrinsics and/or
extrinsics) is not close enough to the real camera geometry for the filter to stay observable.
Both intrinsics (computed-not-measured FOV) and extrinsics (reused from a different camera
body's calibration) are plausible contributors; this test cannot distinguish which.

## Verdict on the original question

**Cannot be answered from this data.** The global-shutter hypothesis test needs raw VIO to at
least converge to a physically plausible trajectory before its accuracy can be compared to
the rolling-shutter baseline. It doesn't. The blocking prerequisite is a **real Kalibr
calibration session for the current AP-IMX900+4mm-lens body** (same procedure as
`instructions/vio_data_collection.md` §2 / `KALIBR_PC_README.md`, not yet done for this
camera) — then re-run this exact same comparison.

## Files

- `~/openvins_ws/config/survey38_ap_imx900_globalshutter/` — approximate config (online-refined), used for the 3 main runs
- `~/openvins_ws/config/survey38_frozen_calib_diag/` — same but with online refinement forced off, used only for the diagnostic
- `field_data/survey38/vio_eval/vio_full_survey38.csv`, `field_data/survey41/vio_eval/vio_full_survey41.csv`, `field_data/survey42/vio_eval/vio_full_survey42.csv` — raw diverged trajectories (kept for reference/re-analysis after a real calibration exists, NOT valid accuracy data as-is)
- `field_data/survey42/vio_eval/vio_full_survey42_frozen_calib_diag.csv` — the frozen-calibration diagnostic run
- `field_data/vio_globalshutter_comparison_result.json` — RMSE/max numbers, annotated INVALID at the top of the file
- This README

## Update 2026-08-12 — real Kalibr calibration wired in (Phase 4-6): partial answer, not a clean one

A real Kalibr calibration for this camera+lens body now exists (`field_data/calib_20260811_232301/kalibr_output/`,
see `ap-imx900-kalibr-calibration-plan` memory) — reprojection error came in above the accept bar (cam-only std
0.94-1.18px vs <0.5px; imu-cam mean 1.36px vs <1px), but the extrinsic translation |t|=0.259m was independently
confirmed by a fresh ruler measurement on the airframe (exact match), resolving the biggest open question from the
first attempt. Wired into `~/openvins_ws/config/ap_imx900_kalibr/` (up_msckf/slam_sigma_px bumped 1→1.5 to account
for the elevated reprojection error) and re-ran the same survey38/41/42 comparison
(`field_data/vio_globalshutter_kalibr_comparison.py`, same fixed-4DOF alignment as `vio_agl_comparison.py`).

**Along the way, found and fixed a real evaluation-methodology bug**: feeding `run_video_msckf` only the RMSE
window itself (e.g. survey38's 194-313s) never lets OpenVINS's static initializer trigger — it needs to see the
real pre-liftoff stationary period to detect the still→moving jerk (`init_dyn_use: false` in this config means
there's no fallback dynamic initializer). The original (invalid) test's `run_video_msckf` calls actually started
well before each liftoff and the RMSE window was applied as a post-hoc filter — not obvious from the numbers
reported at the time. Fixed by starting each run before its flight's real liftoff (survey38: 170s, survey41: 40s,
survey42: 55s) and filtering to the RMSE window afterward, same as the original methodology.

**Result — mixed, altitude-dependent:**

| flight | AGL | old IMX219 baseline | approx calib (invalid) | real calib (this run) |
|---|---|---|---|---|
| survey38 | ~10m | 3.0m (survey31) | 20325m | **26320m** — still catastrophic |
| survey41 | ~65m | n/a | 20990m | **9445m** (shorter, gap-avoiding window) — still catastrophic |
| survey42 | ~100m | 75.8m (survey32) | 1753m | **301.9m** — 5.8x better, velocity no longer runs away |

survey38/41 show the exact same EKF-runaway velocity signature as the approximate-calib test (roughly doubling
every ~15-20s), just a few seconds later onset — the real calibration did not fix low/mid-AGL divergence, only
delayed it. survey42 is qualitatively different: velocity stays bounded (0.5-22 m/s) throughout, a real
improvement, but still ~4x worse than the old rolling-shutter camera's baseline at the same altitude.

**Verdict, updated**: calibration was a necessary but not sufficient condition. This converges on the project's
pre-existing "AGL is the dominant VIO error driver" finding (§14-35/14-41 in `vpe_jump_runaway_diagnosis.md`) —
low AGL means faster/less-well-conditioned parallax for a monocular filter, independent of calibration quality or
shutter type. The original question ("does global shutter fix VIO") still can't be cleanly answered: at the one
altitude where both cameras now have valid (non-degenerate) numbers, the new global-shutter setup is worse, not
better — no support for the shutter-type hypothesis. At 10m/65m neither camera has a valid comparison point since
both diverge. See §14-48 in `vpe_jump_runaway_diagnosis.md` for the full writeup.

**New files**: `~/openvins_ws/config/ap_imx900_kalibr/`, `field_data/survey{38,41,42}/vio_eval/vio_full_surveyXX_kalibr.csv`,
`field_data/vio_globalshutter_kalibr_comparison.py`, `field_data/vio_globalshutter_kalibr_comparison_result.json`.
