#!/usr/bin/env python3
"""
AnyLoc constrained-search (anchor-chain) benchmark on a real recorded flight.

Same technique as test_accuracy_constrained.py (global search vs. anchor-chain
constrained search within --radius-m of the previous estimate — mirrors
ros2_node.py's actual production behaviour), but driven by real video.mkv +
telemetry.csv from a field flight instead of synthetic satellite ground truth.

Starts from the first telemetry sample that reaches --start-agl (the drone's
climb-out into cruise altitude) and steps forward through the video every
--interval-frames frames (default 10, matching ros2_node.py's ANYLOC_INTERVAL),
for as long as AGL stays above --min-agl (default: leaves the cruise band).

Usage:
    /home/jetson/venv/anyloc/bin/python3 anyloc/test_accuracy_real_video_constrained.py \\
        field_data/survey13 --db-dir anyloc/database_test20_vits14 \\
        --start-agl 65 --interval-frames 10 --radius-m 200 \\
        --output anyloc/logs/survey13_test20_constrained.json
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
sys.path.insert(0, HERE)
from test_accuracy_real_video import load_survey, nearest_frame_idx, euclidean_m, _stats


def build_step_frames(frame_times, telemetry, start_agl, start_tol,
                       interval_frames, min_agl, max_steps):
    telemetry = sorted(telemetry, key=lambda t: t['unix_time'])
    start_t = next((t for t in telemetry if t['alt_agl'] >= start_agl - start_tol), None)
    if start_t is None:
        sys.exit(f"No telemetry sample reaches {start_agl - start_tol} m AGL")
    start_frame, _ = nearest_frame_idx(frame_times, start_t['unix_time'])

    frame_by_idx = {f: t for f, t in frame_times}
    max_frame = frame_times[-1][0]

    steps = []
    idx = start_frame
    while idx <= max_frame and len(steps) < max_steps:
        ftime = frame_by_idx.get(idx)
        if ftime is None:
            # frame_times may have gaps; fall back to nearest
            ftime = min(frame_times, key=lambda ft: abs(ft[0] - idx))[1]
        # nearest telemetry sample to this frame's time
        nearest_t = min(telemetry, key=lambda t: abs(t['unix_time'] - ftime))
        if nearest_t['alt_agl'] < min_agl:
            break
        steps.append(dict(frame_idx=idx, true_lat=nearest_t['lat'],
                          true_lon=nearest_t['lon'], agl_m=nearest_t['alt_agl']))
        idx += interval_frames
    return steps


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('survey_dir', help='e.g. field_data/survey13')
    ap.add_argument('--db-dir', required=True)
    ap.add_argument('--start-agl', type=float, default=65.0,
                    help='AGL threshold marking the start of the chain (default: 65)')
    ap.add_argument('--start-tol', type=float, default=0.5,
                    help='First telemetry sample >= start-agl - start-tol starts the '
                         'chain (default: 0.5)')
    ap.add_argument('--min-agl', type=float, default=60.0,
                    help='Stop the chain once AGL drops below this (default: 60)')
    ap.add_argument('--interval-frames', type=int, default=10,
                    help='Video frames between AnyLoc calls (default: 10, matches '
                         "ros2_node.py's ANYLOC_INTERVAL)")
    ap.add_argument('--radius-m', type=float, default=200.0,
                    help='Constrained search radius (default: 200, matches '
                         "ros2_node.py's SEARCH_RADIUS_M)")
    ap.add_argument('--max-steps', type=int, default=300,
                    help='Safety cap on chain length (default: 300)')
    ap.add_argument('--seed-from-truth', action='store_true',
                    help='Seed the first step\'s anchor from the true GPS position '
                         'instead of an unconstrained global search on step 0 -- '
                         'simulates the real GPS-to-VPE handoff (fly up under GPS, '
                         'switch to AnyLoc at a known location), matching '
                         "ros2_node.py's EKF-seeded cold start.")
    ap.add_argument('--output', default='')
    args = ap.parse_args()

    from anyloc.localizer import AnyLocLocalizer

    print(f"\n{'='*70}")
    print(f"  AnyLoc Real-Video Constrained-Search Benchmark  (anchor-chain, no VO)")
    print(f"  Survey : {args.survey_dir}  |  DB : {args.db_dir}")
    print(f"  Start AGL : {args.start_agl} m  |  Interval : {args.interval_frames} frames  "
          f"|  Radius : {args.radius_m} m")
    print(f"{'='*70}\n")

    print("[1/3] Loading survey data …")
    meta, frame_times, telemetry = load_survey(args.survey_dir)
    frame_times.sort(key=lambda x: x[1])
    print(f"  {len(frame_times)} frames, {len(telemetry)} telemetry samples, "
          f"{meta['width']}x{meta['height']} @ {meta['fps']} fps")

    steps = build_step_frames(frame_times, telemetry, args.start_agl, args.start_tol,
                              args.interval_frames, args.min_agl, args.max_steps)
    cos_lat = math.cos(math.radians(steps[0]['true_lat']))
    span_s = (steps[-1]['frame_idx'] - steps[0]['frame_idx']) / meta['fps']
    print(f"  Chain: {len(steps)} steps, frames {steps[0]['frame_idx']}"
          f"→{steps[-1]['frame_idx']} (~{span_s:.1f} s of flight)\n")

    print("[2/3] Loading AnyLoc database and DINOv2 model …")
    loc = AnyLocLocalizer(args.db_dir)
    print()

    print("[3/3] Running localizer (global + constrained on each step) …\n")
    hdr = (f"  {'#':>3}  {'frame':>7}  {'Err_glob':>9}  {'Err_const':>9}  "
           f"{'T_glob ms':>10}  {'T_const ms':>10}  {'InWin':>5}  AGL")
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))

    want = {s['frame_idx']: s for s in steps}
    cap = cv2.VideoCapture(os.path.join(args.survey_dir, 'video.mkv'))
    cap.set(cv2.CAP_PROP_POS_FRAMES, steps[0]['frame_idx'])
    cur = steps[0]['frame_idx']

    if args.seed_from_truth:
        anchor_lat, anchor_lon = steps[0]['true_lat'], steps[0]['true_lon']
        print(f"  Seeding anchor from true GPS position at switch: "
              f"{anchor_lat:.6f}, {anchor_lon:.6f}\n")
    else:
        anchor_lat = anchor_lon = None
    results, err_glob_list, err_const_list = [], [], []
    t_glob_list, t_const_list = [], []

    while want:
        ok, frame_bgr = cap.read()
        if not ok:
            print(f"  [!] video ended early at frame {cur}, {len(want)} step(s) unreachable")
            break
        if cur in want:
            s = want.pop(cur)
            true_lat, true_lon, h = s['true_lat'], s['true_lon'], s['agl_m']
            rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(rgb)

            t0 = time.perf_counter()
            g_lat, g_lon, _, _, g_score, g_idx = loc.localize(pil_img, agl_m=h)
            t_glob_ms = (time.perf_counter() - t0) * 1000.0

            if anchor_lat is None:
                c_lat, c_lon = g_lat, g_lon
                t_const_ms = t_glob_ms
                in_window = True
            else:
                in_window = euclidean_m(true_lat, true_lon, anchor_lat, anchor_lon,
                                        cos_lat) <= args.radius_m
                t0 = time.perf_counter()
                c_lat, c_lon, _, _, c_score, c_idx = loc.localize(
                    pil_img, agl_m=h,
                    center_lat=anchor_lat, center_lon=anchor_lon, radius_m=args.radius_m)
                t_const_ms = (time.perf_counter() - t0) * 1000.0

            anchor_lat, anchor_lon = c_lat, c_lon

            err_glob = euclidean_m(true_lat, true_lon, g_lat, g_lon, cos_lat)
            err_const = euclidean_m(true_lat, true_lon, c_lat, c_lon, cos_lat)
            err_glob_list.append(err_glob); err_const_list.append(err_const)
            t_glob_list.append(t_glob_ms); t_const_list.append(t_const_ms)

            results.append(dict(frame_idx=cur, true_lat=true_lat, true_lon=true_lon,
                                agl_m=h, est_glob_lat=g_lat, est_glob_lon=g_lon,
                                est_const_lat=c_lat, est_const_lon=c_lon,
                                error_glob_m=err_glob, error_const_m=err_const,
                                in_window=in_window,
                                time_glob_ms=round(t_glob_ms, 2),
                                time_const_ms=round(t_const_ms, 2)))

            win_ch = 'Y' if in_window else 'N'
            print(f"  {len(results):>3}  {cur:>7}  {err_glob:>9.1f}  {err_const:>9.1f}  "
                  f"{t_glob_ms:>10.1f}  {t_const_ms:>10.1f}  {win_ch:>5}  {h:.1f} m")
        cur += 1
    cap.release()

    if not results:
        print("\nNo successful localizations.")
        return

    st_glob  = _stats(err_glob_list)
    st_const = _stats(err_const_list)
    in_window_pct = 100.0 * sum(r['in_window'] for r in results) / len(results)

    print(f"\n{'='*70}")
    print(f"  Results  ({len(results)} steps)")
    print(f"{'='*70}")
    print(f"  {'Metric':<24}  {'Global':>10}  {'Constrained':>12}  {'Delta':>8}")
    print(f"  {'-'*24}  {'-'*10}  {'-'*12}  {'-'*8}")
    print(f"  {'Mean error (m)':<24}  {st_glob['mean']:>10.2f}  {st_const['mean']:>12.2f}  "
          f"{st_const['mean'] - st_glob['mean']:>+8.2f}")
    print(f"  {'Median error (m)':<24}  {st_glob['median']:>10.2f}  {st_const['median']:>12.2f}  "
          f"{st_const['median'] - st_glob['median']:>+8.2f}")
    print(f"  {'RMSE (m)':<24}  {st_glob['rmse']:>10.2f}  {st_const['rmse']:>12.2f}  "
          f"{st_const['rmse'] - st_glob['rmse']:>+8.2f}")
    print(f"  {'Max error (m)':<24}  {st_glob['max']:>10.2f}  {st_const['max']:>12.2f}")
    print(f"  {'True pos in window':<24}  {'—':>10}  {in_window_pct:>11.1f} %")
    print(f"{'='*70}\n")

    if args.output:
        report = dict(
            config=dict(survey_dir=args.survey_dir, db_dir=args.db_dir,
                       start_agl=args.start_agl, interval_frames=args.interval_frames,
                       radius_m=args.radius_m, n_steps=len(results)),
            statistics=dict(global_search=st_glob, constrained_search=st_const,
                           in_window_pct=in_window_pct),
            results=results,
        )
        with open(args.output, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"Results saved → {args.output}")


if __name__ == '__main__':
    main()
