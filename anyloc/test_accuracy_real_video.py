#!/usr/bin/env python3
"""
AnyLoc accuracy benchmark using a real recorded flight (video.mkv + telemetry.csv)
as ground truth, instead of synthetic satellite crops.

Fills the "offline replay" gap: field_data/surveyN/ has raw video + GPS telemetry
from a real flight, but nothing replayed it through the localizer for post-hoc
accuracy numbers. This script:

  1. Loads field_data/surveyN/{meta.json, frame_times.csv, telemetry.csv}.
  2. Selects telemetry samples inside an AGL band (default: matches the DB's
     build AGL ± --agl-tol).
  3. Picks --samples of them (evenly spaced by default, or random with --seed).
  4. Finds the nearest video frame to each sample's timestamp, decodes it from
     video.mkv, and runs it through AnyLocLocalizer exactly like ros2_node.py
     does (raw camera frame in, agl_m from telemetry).
  5. Compares the estimate to the telemetry GPS fix and reports error stats.

Usage:
    /home/jetson/venv/anyloc/bin/python3 anyloc/test_accuracy_real_video.py \\
        field_data/survey13 --db-dir anyloc/database_test20_vits14 \\
        --samples 15 --agl 65 --agl-tol 3 --seed 42 \\
        --output anyloc/logs/survey13_test20_accuracy.json
"""

import argparse
import csv
import json
import math
import os
import random
import sys
import time

import cv2
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))


def euclidean_m(lat1, lon1, lat2, lon2, cos_lat):
    dlat_m = (lat1 - lat2) * 111_320.0
    dlon_m = (lon1 - lon2) * 111_320.0 * cos_lat
    return math.sqrt(dlat_m ** 2 + dlon_m ** 2)


def _stats(values):
    n    = len(values)
    mean = sum(values) / n
    var  = sum((v - mean) ** 2 for v in values) / n
    rmse = math.sqrt(sum(v ** 2 for v in values) / n)
    s    = sorted(values)
    med  = s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2
    return dict(n=n, mean=mean, median=med, std=math.sqrt(var),
                rmse=rmse, min=min(values), max=max(values))


def load_survey(survey_dir):
    with open(os.path.join(survey_dir, 'meta.json')) as f:
        meta = json.load(f)

    frame_times = []
    with open(os.path.join(survey_dir, 'frame_times.csv')) as f:
        for row in csv.DictReader(f):
            frame_times.append((int(row['frame_idx']), float(row['unix_time'])))

    telemetry = []
    with open(os.path.join(survey_dir, 'telemetry.csv')) as f:
        for row in csv.DictReader(f):
            telemetry.append(dict(
                unix_time=float(row['unix_time']),
                lat=float(row['lat']), lon=float(row['lon']),
                alt_agl=float(row['alt_agl']),
            ))
    return meta, frame_times, telemetry


def nearest_frame_idx(frame_times, t):
    """frame_times sorted by unix_time; binary search for closest."""
    lo, hi = 0, len(frame_times) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if frame_times[mid][1] < t:
            lo = mid + 1
        else:
            hi = mid
    if lo > 0 and abs(frame_times[lo - 1][1] - t) < abs(frame_times[lo][1] - t):
        lo -= 1
    return frame_times[lo][0], frame_times[lo][1]


def pick_samples(telemetry, agl_target, agl_tol, n_samples, seed):
    band = [t for t in telemetry if abs(t['alt_agl'] - agl_target) <= agl_tol]
    if len(band) < n_samples:
        sys.exit(f"Only {len(band)} telemetry samples within {agl_tol} m of "
                  f"{agl_target} m AGL — need {n_samples}. Widen --agl-tol.")
    rng = random.Random(seed)
    idxs = sorted(rng.sample(range(len(band)), n_samples))
    return [band[i] for i in idxs]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('survey_dir', help='e.g. field_data/survey13')
    ap.add_argument('--db-dir', required=True, help='AnyLoc database directory')
    ap.add_argument('--agl', type=float, default=65.0,
                    help='Target AGL band center in metres (default: 65)')
    ap.add_argument('--agl-tol', type=float, default=3.0,
                    help='+/- metres around --agl to sample from (default: 3)')
    ap.add_argument('--samples', type=int, default=15,
                    help='Number of frames to test (default: 15)')
    ap.add_argument('--seed', type=int, default=42,
                    help='Random seed for sample selection (default: 42)')
    ap.add_argument('--output', default='', help='Save results to this JSON file')
    args = ap.parse_args()

    sys.path.insert(0, os.path.dirname(HERE))
    from anyloc.localizer import AnyLocLocalizer

    print(f"\n{'='*66}")
    print(f"  AnyLoc Real-Video Accuracy Benchmark")
    print(f"  Survey : {args.survey_dir}  |  DB : {args.db_dir}")
    print(f"  AGL band : {args.agl} +/- {args.agl_tol} m  |  Samples : {args.samples}  "
          f"|  Seed : {args.seed}")
    print(f"{'='*66}\n")

    print("[1/4] Loading survey data …")
    meta, frame_times, telemetry = load_survey(args.survey_dir)
    frame_times.sort(key=lambda x: x[1])
    print(f"  {len(frame_times)} frames, {len(telemetry)} telemetry samples, "
          f"{meta['width']}x{meta['height']} @ {meta['fps']} fps")

    print("[2/4] Selecting test samples …")
    samples = pick_samples(telemetry, args.agl, args.agl_tol, args.samples, args.seed)
    cos_lat = math.cos(math.radians(samples[0]['lat']))
    resolved = []
    for s in samples:
        fidx, ftime = nearest_frame_idx(frame_times, s['unix_time'])
        dt = abs(ftime - s['unix_time'])
        resolved.append(dict(true_lat=s['lat'], true_lon=s['lon'], agl_m=s['alt_agl'],
                             frame_idx=fidx, dt_s=dt))
    max_dt = max(r['dt_s'] for r in resolved)
    print(f"  {len(resolved)} samples picked, max frame/telemetry time offset "
          f"= {max_dt*1000:.0f} ms\n")

    print("[3/4] Loading AnyLoc database and DINOv2 model …")
    loc = AnyLocLocalizer(args.db_dir)
    print()

    print("[4/4] Decoding frames and running localizer …\n")
    header = (f"  {'#':>3}  {'frame':>7}  {'True lat':>10}  {'True lon':>11}  "
              f"{'Est lat':>10}  {'Est lon':>11}  {'Err (m)':>8}  {'Score':>6}  AGL")
    print(header)
    print("  " + "-" * (len(header) - 2))

    # Single sequential pass through the video, from the first needed frame
    # index onward, so we don't decode the whole file for a handful of samples.
    resolved.sort(key=lambda r: r['frame_idx'])
    cap = cv2.VideoCapture(os.path.join(args.survey_dir, 'video.mkv'))
    start_idx = resolved[0]['frame_idx']
    cap.set(cv2.CAP_PROP_POS_FRAMES, start_idx)

    results, errors_m = [], []
    want = {r['frame_idx']: r for r in resolved}
    cur = start_idx
    while want:
        ok, frame_bgr = cap.read()
        if not ok:
            print(f"  [!] video ended early at frame {cur}, "
                  f"{len(want)} sample(s) unreachable")
            break
        if cur in want:
            r = want.pop(cur)
            rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(rgb)

            t0 = time.perf_counter()
            est_lat, est_lon, _, _, score, db_idx = loc.localize(pil_img, agl_m=r['agl_m'])
            elapsed = time.perf_counter() - t0

            err_m = euclidean_m(r['true_lat'], r['true_lon'], est_lat, est_lon, cos_lat)
            errors_m.append(err_m)
            results.append(dict(
                frame_idx=cur, true_lat=r['true_lat'], true_lon=r['true_lon'],
                agl_m=r['agl_m'], dt_s=round(r['dt_s'], 3),
                est_lat=est_lat, est_lon=est_lon,
                error_m=err_m, score=score, db_idx=db_idx,
                inference_s=round(elapsed, 3),
            ))
            print(f"  {len(results):>3}  {cur:>7}  {r['true_lat']:>10.6f}  "
                  f"{r['true_lon']:>11.6f}  {est_lat:>10.6f}  {est_lon:>11.6f}  "
                  f"{err_m:>8.1f}  {score:>6.3f}  {r['agl_m']:.1f} m")
        cur += 1
    cap.release()

    if not errors_m:
        print("\nNo successful localizations.")
        return

    st = _stats(errors_m)
    print(f"\n{'='*66}")
    print(f"  Results  ({st['n']} / {args.samples} samples)")
    print(f"{'='*66}")
    print(f"  Mean error   : {st['mean']:>8.2f} m")
    print(f"  Median error : {st['median']:>8.2f} m")
    print(f"  RMSE         : {st['rmse']:>8.2f} m")
    print(f"  Std dev      : {st['std']:>8.2f} m")
    print(f"  Min error    : {st['min']:>8.2f} m")
    print(f"  Max error    : {st['max']:>8.2f} m")
    print(f"{'='*66}\n")

    if args.output:
        report = dict(
            config=dict(survey_dir=args.survey_dir, db_dir=args.db_dir,
                       agl_target=args.agl, agl_tol=args.agl_tol,
                       n_samples=args.samples, seed=args.seed),
            statistics=st,
            results=results,
        )
        with open(args.output, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"Results saved → {args.output}")


if __name__ == '__main__':
    main()
