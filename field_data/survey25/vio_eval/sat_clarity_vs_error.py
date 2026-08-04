#!/usr/bin/env python3
"""
Does satellite-tile blurriness actually predict AnyLoc retrieval error on survey25, or was
that just visible in a couple of unlucky examples? Quantifies sharpness (Laplacian variance,
a standard focus/blur metric -- higher = sharper/more high-frequency detail) for every tile
in anyloc/database_survey25_z20_vits14/db_images/, then correlates it against real per-query
retrieval error from anyloc_real_fix1_rotated_full.json two ways:
  (a) sharpness of the tile AnyLoc actually retrieved vs. that query's error_m
  (b) sharpness of the tile nearest the query's TRUE location vs. that query's error_m --
      the more causally relevant test: if the correct answer's own tile is inherently blurry/
      featureless, no retrieval algorithm could have found it regardless of matching quality.
"""
import json
import math
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
DB_DIR = os.path.join(ROOT, "anyloc", "database_survey25_z20_vits14")
RESULT_JSON = os.path.join(HERE, "anyloc_real_fix1_rotated_full.json")


def sharpness(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return None
    return cv2.Laplacian(img, cv2.CV_64F).var()


def main():
    meta = json.load(open(os.path.join(DB_DIR, "db_meta.json")))
    lats, lons, paths = meta["lats"], meta["lons"], meta["paths"]
    n_db = len(lats)
    print(f"[1/3] scoring sharpness of all {n_db} database tiles ...")
    sharp = np.array([sharpness(os.path.join(ROOT, p)) for p in paths])
    print(f"  sharpness (Laplacian var): min={sharp.min():.1f} median={np.median(sharp):.1f} "
          f"mean={sharp.mean():.1f} max={sharp.max():.1f} std={sharp.std():.1f}")

    lat0, lon0 = lats[0], lons[0]
    latm = 111132.954 - 559.822 * math.cos(2 * math.radians(lat0))
    lonm = 111412.84 * math.cos(math.radians(lat0)) - 93.5 * math.cos(3 * math.radians(lat0))
    db_xy = np.column_stack([(np.array(lons) - lon0) * lonm, (np.array(lats) - lat0) * latm])

    print(f"[2/3] loading real query results from {os.path.basename(RESULT_JSON)} ...")
    d = json.load(open(RESULT_JSON))
    results = d["results"]
    print(f"  {len(results)} queries")

    matched_sharp, true_sharp, errs = [], [], []
    for r in results:
        err = r["error_m"]
        db_idx = r["db_idx"]
        matched_sharp.append(sharp[db_idx])
        qxy = np.array([(r["true_lon"] - lon0) * lonm, (r["true_lat"] - lat0) * latm])
        nearest = int(np.argmin(np.linalg.norm(db_xy - qxy, axis=1)))
        true_sharp.append(sharp[nearest])
        errs.append(err)

    matched_sharp = np.array(matched_sharp)
    true_sharp = np.array(true_sharp)
    errs = np.array(errs)

    def pearson(a, b):
        return float(np.corrcoef(a, b)[0, 1])

    def pearson_log(a, b):
        return float(np.corrcoef(np.log1p(a), b)[0, 1])

    print(f"\n[3/3] correlation with error_m (n={len(errs)}):")
    print(f"  matched-tile sharpness  vs error : r={pearson(matched_sharp, errs):+.3f}  "
          f"(log-sharpness: r={pearson_log(matched_sharp, errs):+.3f})")
    print(f"  true-location sharpness vs error : r={pearson(true_sharp, errs):+.3f}  "
          f"(log-sharpness: r={pearson_log(true_sharp, errs):+.3f})")

    # split by true-location sharpness terciles, compare mean error
    order = np.argsort(true_sharp)
    thirds = np.array_split(order, 3)
    print(f"\n  error by true-location-tile sharpness tercile:")
    for name, idx in zip(("blurriest third", "middle third", "sharpest third"), thirds):
        print(f"    {name:16s} (sharpness {true_sharp[idx].min():7.1f}-{true_sharp[idx].max():7.1f}): "
              f"mean err={errs[idx].mean():7.1f}m  median={np.median(errs[idx]):7.1f}m  n={len(idx)}")

    print(f"\n  database tile sharpness distribution (all {n_db} tiles):")
    order2 = np.argsort(sharp)
    print(f"    5 blurriest: {[paths[i] for i in order2[:5]]} "
          f"({[round(float(sharp[i]),1) for i in order2[:5]]})")
    print(f"    5 sharpest : {[paths[i] for i in order2[-5:]]} "
          f"({[round(float(sharp[i]),1) for i in order2[-5:]]})")

    out = dict(
        n_db_tiles=n_db,
        db_sharpness_stats=dict(min=float(sharp.min()), median=float(np.median(sharp)),
                                mean=float(sharp.mean()), max=float(sharp.max()),
                                std=float(sharp.std())),
        n_queries=len(errs),
        pearson_matched_tile_sharpness_vs_error=pearson(matched_sharp, errs),
        pearson_true_location_sharpness_vs_error=pearson(true_sharp, errs),
        pearson_log_matched=pearson_log(matched_sharp, errs),
        pearson_log_true=pearson_log(true_sharp, errs),
        terciles=[dict(name=n, sharp_range=[float(true_sharp[idx].min()), float(true_sharp[idx].max())],
                       mean_err=float(errs[idx].mean()), median_err=float(np.median(errs[idx])),
                       n=int(len(idx)))
                  for n, idx in zip(("blurriest", "middle", "sharpest"), thirds)],
    )
    out_path = os.path.join(HERE, "sat_clarity_vs_error_result.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nwrote {out_path}")


if __name__ == "__main__":
    main()
