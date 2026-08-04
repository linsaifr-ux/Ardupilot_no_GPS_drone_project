#!/usr/bin/env python3
"""
Same-domain (drone-photo vs drone-photo) AnyLoc accuracy test on survey25.

Queries the held-out query split (odd-indexed extracted frames) against a
database built ONLY from the disjoint db split (even-indexed extracted
frames), both drawn from survey25's own camera footage (extract_frames.py
output, split via field_data/survey25/vio_eval/samedomain/*).

This is the same-domain control for the cross-domain (drone-photo vs
satellite-tile) baseline in anyloc_real_baseline.json / anyloc_real_fix1_full.json
-- same error metric / stats format, so results are directly comparable.

Usage:
    /home/jetson/venv/anyloc/bin/python3 \\
        field_data/survey25/vio_eval/samedomain/eval_samedomain.py \\
        --db-dir anyloc/database_survey25_samedomain_vits14 \\
        --query-csv field_data/survey25/vio_eval/samedomain/query_frames.csv \\
        --output field_data/survey25/vio_eval/samedomain/anyloc_samedomain_result.json
"""

import argparse
import csv
import json
import math
import os
import sys
import time

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


def load_query_csv(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rows.append(dict(
                orig_idx=int(row['orig_idx']),
                path=row['path'],
                lat=float(row['lat']),
                lon=float(row['lon']),
                alt_amsl=float(row['alt_amsl']) if row['alt_amsl'] else None,
                agl_m=float(row['alt_agl']),
                heading_deg=float(row['heading_deg']) if row['heading_deg'] else None,
            ))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--db-dir', required=True, help='AnyLoc database directory (same-domain DB)')
    ap.add_argument('--query-csv', required=True,
                    help='CSV of held-out query frames (orig_idx,path,lat,lon,alt_amsl,alt_agl,heading_deg)')
    ap.add_argument('--output', default='', help='Save results to this JSON file')
    args = ap.parse_args()

    print(f"\n{'='*70}")
    print(f"  AnyLoc Same-Domain Accuracy Test (drone-photo vs drone-photo)")
    print(f"  DB : {args.db_dir}")
    print(f"  Query CSV : {args.query_csv}")
    print(f"{'='*70}\n")

    queries = load_query_csv(args.query_csv)
    print(f"[1/2] Loaded {len(queries)} held-out query frames")

    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(HERE))))
    from anyloc.localizer import AnyLocLocalizer

    print("[2/2] Loading AnyLoc database and DINOv2 model …")
    loc = AnyLocLocalizer(args.db_dir)
    print()

    cos_lat = math.cos(math.radians(queries[0]['lat'])) if queries else 1.0
    header = (f"  {'#':>3}  {'idx':>4}  {'True lat':>10}  {'True lon':>11}  "
              f"{'Est lat':>10}  {'Est lon':>11}  {'Err (m)':>8}  {'Score':>6}  db_idx  AGL")
    print(header)
    print("  " + "-" * (len(header) - 2))

    results, errors_m = [], []
    for i, q in enumerate(queries):
        pil_img = Image.open(q['path']).convert('RGB')
        t0 = time.perf_counter()
        est_lat, est_lon, _, _, score, db_idx = loc.localize(pil_img, agl_m=q['agl_m'])
        elapsed = time.perf_counter() - t0

        err_m = euclidean_m(q['lat'], q['lon'], est_lat, est_lon, cos_lat)
        errors_m.append(err_m)
        results.append(dict(
            orig_idx=q['orig_idx'], query_path=q['path'],
            true_lat=q['lat'], true_lon=q['lon'], agl_m=q['agl_m'],
            est_lat=est_lat, est_lon=est_lon,
            error_m=err_m, score=score, db_idx=db_idx,
            inference_s=round(elapsed, 3),
        ))
        print(f"  {i+1:>3}  {q['orig_idx']:>4}  {q['lat']:>10.6f}  {q['lon']:>11.6f}  "
              f"{est_lat:>10.6f}  {est_lon:>11.6f}  {err_m:>8.1f}  {score:>6.3f}  "
              f"{db_idx:>6}  {q['agl_m']:.1f} m")

    st = _stats(errors_m)
    print(f"\n{'='*70}")
    print(f"  Results  ({st['n']} queries)")
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
            config=dict(db_dir=args.db_dir, query_csv=args.query_csv,
                       n_db_entries=loc.lats.shape[0]),
            statistics=st,
            results=results,
        )
        with open(args.output, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"Results saved -> {args.output}")


if __name__ == '__main__':
    main()
