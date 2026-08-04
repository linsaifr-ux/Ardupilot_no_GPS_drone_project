#!/usr/bin/env python3
"""Position-prior-cropped variant of ngps_style_eval_survey32.py.

The global-search version (ngps_style_eval_survey32.py) matches every query frame against the
FULL survey33 mosaic independently -- a cold-start / worst-case test. ngps_flight's actual design
(per its README/config: UKF fuses NGPS positions + VIO + IMU) crops the reference image to a
region near the current position estimate before matching, which should raise the ~51% match
success rate found in the global-search test by shrinking the search space to something with
genuinely overlapping content and comparable scale.

This script reproduces that causally (query-by-query, in time order, real running state -- not
independently precomputed per query like the global-search version could afford to be):

  - No prior yet (first query, or every prior query has failed so far): fall back to a full-mosaic
    global search, exactly like ngps_style_eval_survey32.py.
  - Prior available (last successful match's mosaic position + its timestamp): crop the mosaic to
    a square region around that position, sized by a constant-max-speed uncertainty bound
    (MAX_SPEED_MPS * elapsed_since_last_fix + MARGIN_M) rather than by propagating VIO through the
    corrector's own alignment/scale logic -- this avoids inheriting VIO's own scale-drift error
    into the crop center, and is a reasonable stand-in for what a real UKF's growing position
    covariance would produce between fixes. A failed match leaves the prior unchanged (so the
    crop radius keeps growing until a new fix is found), which is the correct causal behavior.

Run inside the isolated venv (see ngps_style_eval_survey32.py's docstring for why -- installing
kornia/lightglue system-wide previously upgraded numpy to 2.x and shadowed the GStreamer-enabled
system OpenCV build the camera pipeline depends on):
    <venv>/bin/python3 field_data/survey32/vio_eval/ngps_style_eval_survey32_cropped.py --period 2.0
"""
import argparse, json, math, sys, time

import cv2
import numpy as np
import torch

LG_ROOT = "/tmp/claude-1000/-home-jetson-Ardupilot-no-GPS-drone-project/7a9b0735-0413-4339-8289-489fc0ade34d/scratchpad/LightGlue"
sys.path.insert(0, LG_ROOT)
from lightglue import LightGlue, SuperPoint  # noqa: E402

HERE = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey32/vio_eval"
sys.path.insert(0, HERE)
from ngps_style_eval_survey32 import (  # noqa: E402
    ROOT, SURVEY, MOSAIC_PNG, MOSAIC_PGW, MAX_DT, DEVICE, RANSAC_THRESH_PX, MIN_INLIERS,
    euclidean_m, _stats, load_world_file, pixel_to_lonlat, nearest_by_time, load_survey,
    rotate_north_up, to_gray_tensor,
)

QUERY_KP = 1024
CROP_KP = 2048          # fewer than the full-mosaic REF_KP=4096 -- crop covers far less area
MAX_SPEED_MPS = 15.0    # generous vs this project's real cruise speeds (4.5-7.5 m/s, peak ~18 m/s
                         # seen in SITL) -- the uncertainty-radius bound between fixes
MARGIN_M = 30.0         # extra slack for GPS/timing/heading-estimate error
MOSAIC_MPP = 0.15        # field_data/survey33/mosaic_meta.json resolution_m_per_px


def lonlat_to_pixel(lon, lat, wf):
    A, D, B, E, C, F = wf
    px = (lon - C) / A
    py = (lat - F) / E
    return px, py


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--t-start", type=float, default=116.0)
    ap.add_argument("--t-end", type=float, default=197.2)
    ap.add_argument("--period", type=float, default=2.0)
    ap.add_argument("--output", default="")
    a = ap.parse_args()
    T_START, T_END, PERIOD = a.t_start, a.t_end, a.period
    OUT_JSON = a.output or f"{HERE}/ngps_cropped_eval_result_p{PERIOD}.json"

    print(f"device={DEVICE}  window=[{T_START},{T_END}]  period={PERIOD}  (position-prior-cropped)")
    extractor = SuperPoint(max_num_keypoints=QUERY_KP).eval().to(DEVICE)
    ref_extractor_full = SuperPoint(max_num_keypoints=4096).eval().to(DEVICE)
    ref_extractor_crop = SuperPoint(max_num_keypoints=CROP_KP).eval().to(DEVICE)
    matcher = LightGlue(features="superpoint").eval().to(DEVICE)

    print("[1/4] loading reference mosaic + world file ...")
    ref_bgr = cv2.imread(MOSAIC_PNG)
    assert ref_bgr is not None, MOSAIC_PNG
    ref_h, ref_w = ref_bgr.shape[:2]
    wf = load_world_file(MOSAIC_PGW)
    with torch.no_grad():
        full_feats = ref_extractor_full.extract(to_gray_tensor(ref_bgr))
    print(f"  mosaic {ref_w}x{ref_h}, {full_feats['keypoints'].shape[1]} full-image keypoints")

    print("[2/4] selecting query instants (same cadence logic as the global-search script) ...")
    meta, frame_times, telemetry = load_survey(SURVEY)
    frame_times.sort(key=lambda x: x[1])
    telemetry.sort(key=lambda r: r["unix_time"])
    T0 = meta["video_start_unix"]
    n_steps = int((T_END - T_START) / PERIOD) + 1
    resolved = []
    for i in range(n_steps):
        t_rel = T_START + i * PERIOD
        t_unix = T0 + t_rel
        ti = nearest_by_time(telemetry, t_unix, key=lambda r: r["unix_time"])
        if abs(telemetry[ti]["unix_time"] - t_unix) > MAX_DT:
            continue
        fi = nearest_by_time(frame_times, t_unix, key=lambda r: r[1])
        fidx, ftime = frame_times[fi]
        resolved.append(dict(t_rel=t_rel, t_unix=t_unix, true_lat=telemetry[ti]["lat"],
                              true_lon=telemetry[ti]["lon"], agl_m=telemetry[ti]["alt_agl"],
                              frame_idx=fidx, heading=telemetry[ti]["heading"]))
    print(f"  {len(resolved)} queries")

    print("[3/4] causal position-prior-cropped matching ...\n")
    cos_lat = math.cos(math.radians(resolved[0]["true_lat"])) if resolved else 1.0
    cap = cv2.VideoCapture(f"{SURVEY}/video.mkv")
    resolved.sort(key=lambda r: r["frame_idx"])
    want = {}
    for r in resolved:
        want.setdefault(r["frame_idx"], []).append(r)
    cap.set(cv2.CAP_PROP_POS_FRAMES, resolved[0]["frame_idx"])
    cur = resolved[0]["frame_idx"]

    # causal running prior state
    last_fix_px = None   # (px, py) in FULL mosaic pixel coords
    last_fix_t = None    # unix time of that fix

    results, errors_m, failures, n_global, n_cropped = [], [], 0, 0, 0
    header = (f"  {'#':>3}  {'t_rel':>7}  {'mode':>7}  {'crop_px':>8}  {'n_match':>7}  "
              f"{'n_inlier':>8}  {'Err (m)':>9}  {'status':>10}")
    print(header)
    print("  " + "-" * (len(header) - 2))
    while want:
        ok, frame_bgr = cap.read()
        if not ok:
            print(f"  [!] video ended early at frame {cur}")
            break
        if cur in want:
            rs = want.pop(cur)
            r = rs[0]
            frame_use = frame_bgr
            if r["heading"] is not None:
                frame_use = rotate_north_up(frame_bgr, r["heading"])

            t0 = time.perf_counter()
            with torch.no_grad():
                q_feats = extractor.extract(to_gray_tensor(frame_use))

            if last_fix_px is None:
                mode = "global"
                n_global += 1
                ref_feats = full_feats
                crop_ox, crop_oy = 0, 0
                crop_r_px = None
            else:
                mode = "cropped"
                n_cropped += 1
                elapsed = r["t_unix"] - last_fix_t
                radius_m = MAX_SPEED_MPS * max(elapsed, 0.0) + MARGIN_M
                crop_r_px = radius_m / MOSAIC_MPP
                cx, cy = last_fix_px
                x0 = int(max(0, cx - crop_r_px)); x1 = int(min(ref_w, cx + crop_r_px))
                y0 = int(max(0, cy - crop_r_px)); y1 = int(min(ref_h, cy + crop_r_px))
                crop_bgr = ref_bgr[y0:y1, x0:x1]
                crop_ox, crop_oy = x0, y0
                with torch.no_grad():
                    ref_feats = ref_extractor_crop.extract(to_gray_tensor(crop_bgr))

            with torch.no_grad():
                m = matcher({"image0": q_feats, "image1": ref_feats})
            elapsed_s = time.perf_counter() - t0

            kp0 = q_feats["keypoints"][0].cpu().numpy()
            kp1 = ref_feats["keypoints"][0].cpu().numpy()
            matches = m["matches"][0].cpu().numpy()
            n_match = len(matches)

            status, err_m, n_inlier, est_lat, est_lon = "no_match", None, 0, None, None
            if n_match >= 4:
                pts0 = kp0[matches[:, 0]]
                pts1 = kp1[matches[:, 1]] + np.array([crop_ox, crop_oy])  # -> full-mosaic pixels
                H, inlier_mask = cv2.findHomography(pts0, pts1, cv2.RANSAC, RANSAC_THRESH_PX)
                n_inlier = int(inlier_mask.sum()) if inlier_mask is not None else 0
                if H is not None and n_inlier >= MIN_INLIERS:
                    h, w = frame_use.shape[:2]
                    center = np.array([[[w / 2.0, h / 2.0]]], dtype=np.float32)
                    proj = cv2.perspectiveTransform(center, H)[0, 0]
                    est_lon, est_lat = pixel_to_lonlat(proj[0], proj[1], wf)
                    err_m = euclidean_m(r["true_lat"], r["true_lon"], est_lat, est_lon, cos_lat)
                    errors_m.append(err_m)
                    status = "ok"
                    last_fix_px = (float(proj[0]), float(proj[1]))
                    last_fix_t = r["t_unix"]
                else:
                    failures += 1
            else:
                failures += 1

            results.append(dict(t_rel=round(r["t_rel"], 2), frame_idx=cur, mode=mode,
                                 crop_radius_px=(round(crop_r_px, 1) if crop_r_px else None),
                                 true_lat=r["true_lat"], true_lon=r["true_lon"],
                                 n_match=n_match, n_inlier=n_inlier, status=status,
                                 est_lat=est_lat, est_lon=est_lon,
                                 error_m=err_m, inference_s=round(elapsed_s, 3)))
            print(f"  {len(results):>3}  {r['t_rel']:>7.1f}  {mode:>7}  "
                  f"{(f'{crop_r_px:>8.0f}' if crop_r_px else '     n/a')}  {n_match:>7}  {n_inlier:>8}  "
                  f"{(f'{err_m:>9.1f}' if err_m is not None else '      n/a')}  {status:>10}")
        cur += 1
    cap.release()

    print(f"\n[4/4] done. {len(errors_m)}/{len(results)} localized ({failures} failed); "
          f"{n_global} global-search queries (no prior yet), {n_cropped} cropped queries.")
    stats_ok = _stats(errors_m)
    print("\nStats over SUCCESSFUL localizations only:")
    for k, v in stats_ok.items():
        print(f"  {k}: {v}")

    out = dict(method="ngps_style_position_prior_cropped", reference="survey33/mosaic.png",
               query_window=[T_START, T_END], period=PERIOD, max_speed_mps=MAX_SPEED_MPS,
               margin_m=MARGIN_M, n_queries=len(results), n_localized=len(errors_m),
               n_failed=failures, n_global_search=n_global, n_cropped_search=n_cropped,
               stats_successful_only=stats_ok, results=results)
    with open(OUT_JSON, "w") as f:
        json.dump(out, f, indent=2, default=str)
    print(f"\nwritten: {OUT_JSON}")


if __name__ == "__main__":
    main()
