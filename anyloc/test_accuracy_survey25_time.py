#!/usr/bin/env python3
"""
AnyLoc real-video accuracy benchmark on survey25, sampled at a FIXED TIME CADENCE
(default one query every 2 s) across a chosen video-relative time window — instead
of test_accuracy_real_video.py's fixed-count/AGL-band random sampling.

This matches the anchor cadence used throughout the survey25 vio_eval fusion work
(field_data/survey25/vio_eval/foundloc_corrector.py --anchor-period default 2.0),
so the resulting real-match stream can be fed directly into that corrector for an
apples-to-apples comparison against the simulated-anchor runs.

GLOBAL SEARCH ONLY (no anchor-chain / constrained radius) — matches this project's
own prior finding that constrained/anchor-chain search performs WORSE than global
search on real footage (memory: real_video_constrained_search_failure).

Usage:
    /home/jetson/venv/anyloc/bin/python3 anyloc/test_accuracy_survey25_time.py \\
        field_data/survey25 --db-dir anyloc/database_test1_vits14 \\
        --t-start 170 --t-end 472 --period 2.0 \\
        --output field_data/survey25/vio_eval/anyloc_real_baseline.json
"""

import argparse
import csv
import json
import math
import os
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
            try:
                heading = float(row['heading_deg']) if row.get('heading_deg') else None
            except ValueError:
                heading = None
            try:
                telemetry.append(dict(
                    unix_time=float(row['unix_time']),
                    lat=float(row['lat']), lon=float(row['lon']),
                    alt_agl=float(row['alt_agl']),
                    heading=heading,
                ))
            except ValueError:
                pass
    return meta, frame_times, telemetry


def nearest_by_time(items, t, key):
    """items sorted by key(item); binary search for closest."""
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


def build_feature_extractor(args):
    """Returns an AnyLocLocalizer-compatible instance per --feature-mode."""
    sys.path.insert(0, os.path.dirname(HERE))
    if args.feature_mode == 'default':
        from anyloc.localizer import AnyLocLocalizer
        return AnyLocLocalizer(args.db_dir)
    elif args.feature_mode == 'value_facet':
        from anyloc.localizer import AnyLocLocalizerValueFacet
        return AnyLocLocalizerValueFacet(
            args.db_dir, model_name=args.value_model, layer=args.value_layer)
    else:
        raise ValueError(args.feature_mode)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('survey_dir', help='e.g. field_data/survey25')
    ap.add_argument('--db-dir', required=True, help='AnyLoc database directory')
    ap.add_argument('--t-start', type=float, default=170.0,
                    help='video-relative start time, seconds (default 170)')
    ap.add_argument('--t-end', type=float, default=472.0,
                    help='video-relative end time, seconds (default 472)')
    ap.add_argument('--period', type=float, default=2.0,
                    help='seconds between queries (default 2.0, matches '
                         'foundloc_corrector.py --anchor-period)')
    ap.add_argument('--max-dt', type=float, default=1.0,
                    help='skip a query if nearest telemetry sample is farther than '
                         'this many seconds away (default 1.0)')
    ap.add_argument('--feature-mode', choices=['default', 'value_facet'], default='default',
                    help="'default' = AnyLocLocalizer as shipped (final-layer patch tokens); "
                         "'value_facet' = AnyLocLocalizerValueFacet (AnyLoc-paper-style "
                         "intermediate-layer value-facet features)")
    ap.add_argument('--value-model', default='dinov2_vitg14',
                    help='backbone for --feature-mode value_facet (default dinov2_vitg14)')
    ap.add_argument('--value-layer', type=int, default=31,
                    help='0-indexed block to hook for --feature-mode value_facet (default 31)')
    ap.add_argument('--output', default='', help='Save results to this JSON file')
    ap.add_argument('--rotate', action='store_true',
                    help="Rotate each query frame to North-up using telemetry heading "
                         "before matching, same method as tools/extract_frames.py --rotate. "
                         "Replicates AnyLoc paper's Nardo-Air-R methodology (drone imagery "
                         "rotated to match satellite orientation): the paper reports this "
                         "alone lifts aerial-vs-satellite Recall@1 from 76.1% to 94.4%.")
    args = ap.parse_args()

    print(f"\n{'='*70}")
    print(f"  AnyLoc Real-Video Accuracy Benchmark (time-cadence sampling)")
    print(f"  Survey : {args.survey_dir}  |  DB : {args.db_dir}")
    print(f"  Window : t={args.t_start:.0f}-{args.t_end:.0f}s  |  period={args.period}s "
          f"|  feature-mode={args.feature_mode}")
    print(f"{'='*70}\n")

    print("[1/4] Loading survey data ...")
    meta, frame_times, telemetry = load_survey(args.survey_dir)
    frame_times.sort(key=lambda x: x[1])
    telemetry.sort(key=lambda x: x['unix_time'])
    T0 = meta['video_start_unix']
    print(f"  {len(frame_times)} frames, {len(telemetry)} telemetry samples, "
          f"{meta['width']}x{meta['height']} @ {meta['fps']} fps, T0={T0:.3f}")

    print("[2/4] Selecting time-cadence samples ...")
    n_steps = int((args.t_end - args.t_start) / args.period) + 1
    resolved = []
    skipped_dt = 0
    for i in range(n_steps):
        t_rel = args.t_start + i * args.period
        t_unix = T0 + t_rel
        ti = nearest_by_time(telemetry, t_unix, key=lambda r: r['unix_time'])
        tel_dt = abs(telemetry[ti]['unix_time'] - t_unix)
        if tel_dt > args.max_dt:
            skipped_dt += 1
            continue
        fi = nearest_by_time(frame_times, t_unix, key=lambda r: r[1])
        fidx, ftime = frame_times[fi]
        resolved.append(dict(
            t_rel=t_rel, t_unix=t_unix,
            true_lat=telemetry[ti]['lat'], true_lon=telemetry[ti]['lon'],
            agl_m=telemetry[ti]['alt_agl'], frame_idx=fidx,
            dt_s=abs(ftime - t_unix), heading=telemetry[ti]['heading'],
        ))
    print(f"  {len(resolved)} queries picked ({skipped_dt} skipped, no telemetry within "
          f"{args.max_dt}s), cadence {args.period}s over [{args.t_start:.0f},{args.t_end:.0f}]s\n")

    print("[3/4] Loading AnyLoc database and feature extractor ...")
    loc = build_feature_extractor(args)
    print()

    print("[4/4] Decoding frames and running localizer ...\n")
    cos_lat = math.cos(math.radians(resolved[0]['true_lat'])) if resolved else 1.0
    header = (f"  {'#':>3}  {'t_rel':>7}  {'frame':>7}  {'True lat':>10}  {'True lon':>11}  "
              f"{'Est lat':>10}  {'Est lon':>11}  {'Err (m)':>8}  {'Score':>6}  AGL")
    print(header)
    print("  " + "-" * (len(header) - 2))

    resolved.sort(key=lambda r: r['frame_idx'])
    cap = cv2.VideoCapture(os.path.join(args.survey_dir, 'video.mkv'))
    start_idx = resolved[0]['frame_idx']
    cap.set(cv2.CAP_PROP_POS_FRAMES, start_idx)

    results, errors_m = [], []
    want = {}
    for r in resolved:
        want.setdefault(r['frame_idx'], []).append(r)
    cur = start_idx
    while want:
        ok, frame_bgr = cap.read()
        if not ok:
            print(f"  [!] video ended early at frame {cur}, "
                  f"{sum(len(v) for v in want.values())} sample(s) unreachable")
            break
        if cur in want:
            rs = want.pop(cur)
            frame_use = frame_bgr
            if args.rotate and rs[0]['heading'] is not None:
                h, w = frame_use.shape[:2]
                cx, cy = w // 2, h // 2
                # NOTE: angle is -heading, not +heading. Compass heading is CW-from-North;
                # cv2.getRotationMatrix2D's positive angle rotates the image CCW. To bring
                # image-up (pointing `heading` CW from North) to true north-up, the content
                # must be rotated `heading` degrees CW, i.e. -heading in cv2's CCW convention.
                # Empirically verified on survey25 (t=110-472s, database_survey25_z20_vits14):
                # +heading made retrieval WORSE (286.5m->298.6m mean); -heading is the fix
                # (286.5m->228.6m mean). tools/extract_frames.py --rotate had this same sign
                # bug -- fixed there too, see instructions/vpe_jump_runaway_diagnosis.md.
                M = cv2.getRotationMatrix2D((cx, cy), -rs[0]['heading'], 1.0)
                frame_use = cv2.warpAffine(frame_use, M, (w, h))
            rgb = cv2.cvtColor(frame_use, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(rgb)

            t0 = time.perf_counter()
            est_lat, est_lon, _, _, score, db_idx = loc.localize(pil_img, agl_m=rs[0]['agl_m'])
            elapsed = time.perf_counter() - t0

            for r in rs:
                err_m = euclidean_m(r['true_lat'], r['true_lon'], est_lat, est_lon, cos_lat)
                errors_m.append(err_m)
                results.append(dict(
                    t_rel=round(r['t_rel'], 2), t_unix=r['t_unix'],
                    frame_idx=cur, true_lat=r['true_lat'], true_lon=r['true_lon'],
                    agl_m=r['agl_m'], dt_s=round(r['dt_s'], 3),
                    est_lat=est_lat, est_lon=est_lon,
                    error_m=err_m, score=score, db_idx=db_idx,
                    inference_s=round(elapsed, 3),
                ))
                print(f"  {len(results):>3}  {r['t_rel']:>7.1f}  {cur:>7}  {r['true_lat']:>10.6f}  "
                      f"{r['true_lon']:>11.6f}  {est_lat:>10.6f}  {est_lon:>11.6f}  "
                      f"{err_m:>8.1f}  {score:>6.3f}  {r['agl_m']:.1f} m")
        cur += 1
    cap.release()

    if not errors_m:
        print("\nNo successful localizations.")
        return

    st = _stats(errors_m)
    print(f"\n{'='*70}")
    print(f"  Results  ({st['n']} / {len(resolved)} queries)")
    print(f"{'='*70}")
    print(f"  Mean error   : {st['mean']:>8.2f} m")
    print(f"  Median error : {st['median']:>8.2f} m")
    print(f"  RMSE         : {st['rmse']:>8.2f} m")
    print(f"  Std dev      : {st['std']:>8.2f} m")
    print(f"  Min error    : {st['min']:>8.2f} m")
    print(f"  Max error    : {st['max']:>8.2f} m")
    print(f"{'='*70}\n")

    if args.output:
        report = dict(
            config=dict(survey_dir=args.survey_dir, db_dir=args.db_dir,
                       t_start=args.t_start, t_end=args.t_end, period=args.period,
                       feature_mode=args.feature_mode,
                       value_model=args.value_model if args.feature_mode == 'value_facet' else None,
                       value_layer=args.value_layer if args.feature_mode == 'value_facet' else None),
            statistics=st,
            results=results,
        )
        with open(args.output, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"Results saved -> {args.output}")


if __name__ == '__main__':
    main()
