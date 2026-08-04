#!/usr/bin/env python3
"""Reproduce the ngps_flight (snktshrma/ngps_flight) VPR method offline and compare against
AnyLoc on the exact same survey32-vs-survey33 cross-session benchmark used for the AnyLoc number
(anyloc/test_accuracy_survey25_time.py, --rotate, t=116-197.2s, period=2s -> 41 queries, mean
25.13m median 15.23m rmse 40.57m -- field_data/survey32/vio_eval/README.md section 2).

ngps_flight's actual method (per its README, no ROS2/TensorRT available for this JetPack/CUDA
version so reproduced directly): SuperPoint keypoints + LightGlue matching between the drone
frame and a single georeferenced satellite/reference image, then a homography from the inlier
matches maps the query frame into the reference image's pixel space, and the reference image's
own georeferencing (affine world-file transform) converts that pixel to lat/lon. This is a
genuinely different paradigm from AnyLoc's global-descriptor nearest-tile retrieval: local
feature matching + geometric homography, potentially continuous (sub-tile) position precision
instead of AnyLoc's discrete per-tile position -- but also capable of outright failing on a
frame (too few/no inlier matches) where AnyLoc always returns *some* nearest neighbour.

Reference image: field_data/survey33/mosaic.png + mosaic.pgw (already-built georeferenced
mosaic of survey33's mapped area from this project's own real frames -- same source data
AnyLoc's database was built from, keeping the comparison same-domain on both sides) instead of
AnyLoc's 112 discrete tiles, matching ngps_flight's actual "single reference_image_path" design
(config/ngps_config.yaml: reference_min_lon/max_lon/min_lat/max_lat for ONE image, not a tile
database).

Query frames, cadence, and North-up rotation (-heading, same sign-convention fix already
verified in this project) are copied verbatim from test_accuracy_survey25_time.py to guarantee
the exact same 41 query instants are used for both methods.

Run inside the isolated venv (NOT system Python -- see memory ctex... no, see this session's
venv-isolation incident: installing kornia/lightglue system-wide silently upgraded numpy to 2.x
and shadowed the GStreamer-enabled system OpenCV build, which would have broken the camera
pipeline):
    <venv>/bin/python3 field_data/survey32/vio_eval/ngps_style_eval_survey32.py
"""
import argparse, csv, json, math, os, sys, time

import cv2
import numpy as np
import torch

LG_ROOT = "/tmp/claude-1000/-home-jetson-Ardupilot-no-GPS-drone-project/7a9b0735-0413-4339-8289-489fc0ade34d/scratchpad/LightGlue"
sys.path.insert(0, LG_ROOT)
from lightglue import LightGlue, SuperPoint  # noqa: E402

ROOT = "/home/jetson/Ardupilot_no_GPS_drone_project"
SURVEY = f"{ROOT}/field_data/survey32"
MOSAIC_PNG = f"{ROOT}/field_data/survey33/mosaic.png"
MOSAIC_PGW = f"{ROOT}/field_data/survey33/mosaic.pgw"
MAX_DT = 1.0
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
QUERY_KP, REF_KP = 1024, 4096
RANSAC_THRESH_PX = 8.0
MIN_INLIERS = 8


def euclidean_m(lat1, lon1, lat2, lon2, cos_lat):
    dlat_m = (lat1 - lat2) * 111_320.0
    dlon_m = (lon1 - lon2) * 111_320.0 * cos_lat
    return math.sqrt(dlat_m ** 2 + dlon_m ** 2)


def _stats(values):
    n = len(values)
    if n == 0:
        return dict(n=0)
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / n
    rmse = math.sqrt(sum(v ** 2 for v in values) / n)
    s = sorted(values)
    med = s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2
    return dict(n=n, mean=mean, median=med, std=math.sqrt(var),
                rmse=rmse, min=min(values), max=max(values))


def load_world_file(pgw_path):
    vals = [float(x) for x in open(pgw_path).read().split()]
    A, D, B, E, C, F = vals  # pixel (col,row) -> lon = A*col + B*row + C ; lat = D*col + E*row + F
    return A, D, B, E, C, F


def pixel_to_lonlat(px, py, wf):
    A, D, B, E, C, F = wf
    lon = A * px + B * py + C
    lat = D * px + E * py + F
    return lon, lat


def nearest_by_time(items, t, key):
    lo, hi = 0, len(items) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if key(items[mid]) < t:
            lo = mid + 1
        else:
            hi = mid
    if lo > 0 and abs(key(items[lo - 1]) - t) < abs(key(items[lo]) - t):
        lo -= 1
    return lo


def load_survey(survey_dir):
    with open(os.path.join(survey_dir, "meta.json")) as f:
        meta = json.load(f)
    frame_times = []
    with open(os.path.join(survey_dir, "frame_times.csv")) as f:
        for row in csv.DictReader(f):
            frame_times.append((int(row["frame_idx"]), float(row["unix_time"])))
    telemetry = []
    with open(os.path.join(survey_dir, "telemetry.csv")) as f:
        for row in csv.DictReader(f):
            try:
                heading = float(row["heading_deg"]) if row.get("heading_deg") else None
            except ValueError:
                heading = None
            try:
                telemetry.append(dict(
                    unix_time=float(row["unix_time"]), lat=float(row["lat"]),
                    lon=float(row["lon"]), alt_agl=float(row["alt_agl"]), heading=heading))
            except ValueError:
                pass
    return meta, frame_times, telemetry


def rotate_north_up(frame_bgr, heading_deg):
    h, w = frame_bgr.shape[:2]
    cx, cy = w // 2, h // 2
    M = cv2.getRotationMatrix2D((cx, cy), -heading_deg, 1.0)  # same sign fix as this project's other tools
    return cv2.warpAffine(frame_bgr, M, (w, h))


def to_gray_tensor(bgr):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    t = torch.from_numpy(gray).float() / 255.0
    return t[None, None].to(DEVICE)  # (1,1,H,W)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--t-start", type=float, default=116.0)
    ap.add_argument("--t-end", type=float, default=197.2)
    ap.add_argument("--period", type=float, default=2.0)
    ap.add_argument("--output", default="")
    a = ap.parse_args()
    T_START, T_END, PERIOD = a.t_start, a.t_end, a.period
    OUT_JSON = a.output or f"{ROOT}/field_data/survey32/vio_eval/ngps_style_eval_result.json"

    print(f"device={DEVICE}  window=[{T_START},{T_END}]  period={PERIOD}")
    extractor = SuperPoint(max_num_keypoints=QUERY_KP).eval().to(DEVICE)
    matcher = LightGlue(features="superpoint").eval().to(DEVICE)

    print("[1/4] loading reference mosaic + world file ...")
    ref_bgr = cv2.imread(MOSAIC_PNG)
    assert ref_bgr is not None, MOSAIC_PNG
    wf = load_world_file(MOSAIC_PGW)
    ref_extractor = SuperPoint(max_num_keypoints=REF_KP).eval().to(DEVICE)
    with torch.no_grad():
        ref_feats = ref_extractor.extract(to_gray_tensor(ref_bgr))
    print(f"  mosaic {ref_bgr.shape[1]}x{ref_bgr.shape[0]}, "
          f"{ref_feats['keypoints'].shape[1]} keypoints extracted")

    print("[2/4] selecting the SAME 41 query instants as the AnyLoc baseline ...")
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
        resolved.append(dict(t_rel=t_rel, true_lat=telemetry[ti]["lat"],
                              true_lon=telemetry[ti]["lon"], agl_m=telemetry[ti]["alt_agl"],
                              frame_idx=fidx, heading=telemetry[ti]["heading"]))
    print(f"  {len(resolved)} queries")

    print("[3/4] matching each query frame against the mosaic (SuperPoint+LightGlue) ...\n")
    cos_lat = math.cos(math.radians(resolved[0]["true_lat"])) if resolved else 1.0
    cap = cv2.VideoCapture(os.path.join(SURVEY, "video.mkv"))
    resolved.sort(key=lambda r: r["frame_idx"])
    want = {}
    for r in resolved:
        want.setdefault(r["frame_idx"], []).append(r)
    cap.set(cv2.CAP_PROP_POS_FRAMES, resolved[0]["frame_idx"])
    cur = resolved[0]["frame_idx"]

    results, errors_m, failures = [], [], 0
    header = (f"  {'#':>3}  {'t_rel':>7}  {'n_match':>7}  {'n_inlier':>8}  "
              f"{'Err (m)':>9}  {'status':>10}")
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
                m = matcher({"image0": q_feats, "image1": ref_feats})
            elapsed = time.perf_counter() - t0

            kp0 = q_feats["keypoints"][0].cpu().numpy()
            kp1 = ref_feats["keypoints"][0].cpu().numpy()
            matches = m["matches"][0].cpu().numpy()
            n_match = len(matches)

            status, err_m, n_inlier, est_lat, est_lon = "no_match", None, 0, None, None
            if n_match >= 4:
                pts0 = kp0[matches[:, 0]]
                pts1 = kp1[matches[:, 1]]
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
                else:
                    failures += 1
            else:
                failures += 1

            results.append(dict(t_rel=round(r["t_rel"], 2), frame_idx=cur,
                                 true_lat=r["true_lat"], true_lon=r["true_lon"],
                                 n_match=n_match, n_inlier=n_inlier, status=status,
                                 est_lat=est_lat, est_lon=est_lon,
                                 error_m=err_m, inference_s=round(elapsed, 3)))
            print(f"  {len(results):>3}  {r['t_rel']:>7.1f}  {n_match:>7}  {n_inlier:>8}  "
                  f"{(f'{err_m:>9.1f}' if err_m is not None else '      n/a')}  {status:>10}")
        cur += 1
    cap.release()

    print(f"\n[4/4] done. {len(errors_m)}/{len(results)} queries localized "
          f"({failures} failed: <4 raw matches or <{MIN_INLIERS} RANSAC inliers).")
    stats_ok = _stats(errors_m)
    print("\nStats over SUCCESSFUL localizations only:")
    for k, v in stats_ok.items():
        print(f"  {k}: {v}")

    out = dict(method="ngps_style_superpoint_lightglue", reference="survey33/mosaic.png",
               query_window=[T_START, T_END], period=PERIOD, n_queries=len(results),
               n_localized=len(errors_m), n_failed=failures,
               stats_successful_only=stats_ok, results=results)
    with open(OUT_JSON, "w") as f:
        json.dump(out, f, indent=2, default=str)
    print(f"\nwritten: {OUT_JSON}")


if __name__ == "__main__":
    main()
