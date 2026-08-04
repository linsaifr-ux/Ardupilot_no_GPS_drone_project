#!/usr/bin/env python3
"""Same time-recovery approach as recover_query_times.py, applied to Round 9's mixed-altitude
(50-100m AGL) same-domain query result, so a matched-args fusion comparison can be run against
the new 100m-only anchors (same run_corrector args, only the anchor source differs)."""
import csv
import json

S = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey25"
OUT = f"{S}/vio_eval/samedomain_100m"

tel = []
with open(f"{S}/telemetry.csv") as f:
    for r in csv.DictReader(f):
        try:
            tel.append((float(r["unix_time"]), float(r["lat"]), float(r["lon"]), float(r["alt_agl"])))
        except ValueError:
            pass

by_ll = {}
for t, lat, lon, agl in tel:
    key = (round(lat, 8), round(lon, 8))
    by_ll.setdefault(key, []).append((t, agl))

result = json.load(open(f"{S}/vio_eval/samedomain/anyloc_samedomain_result.json"))
results = result["results"]

n_exact = 0
n_fallback = 0
rows_out = []
for r in results:
    lat, lon, agl = r["true_lat"], r["true_lon"], r["agl_m"]
    key = (round(lat, 7), round(lon, 7))
    cands = by_ll.get(key)
    if cands:
        cands_sorted = sorted(cands, key=lambda ta: abs(ta[1] - agl))
        t_unix = cands_sorted[0][0]
        n_exact += 1
    else:
        best = min(tel, key=lambda row: (row[1] - lat) ** 2 + (row[2] - lon) ** 2)
        t_unix = best[0]
        n_fallback += 1
    rows_out.append(dict(r, t_unix=t_unix))

print(f"Matched {n_exact} exact, {n_fallback} fallback (of {len(results)})")

rows_out.sort(key=lambda r: r["t_unix"])
with open(f"{OUT}/query_frames_mixed_timed.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["orig_idx", "t_unix", "true_lat", "true_lon", "agl_m", "est_lat", "est_lon", "error_m", "score"])
    for r in rows_out:
        w.writerow([r["orig_idx"], f"{r['t_unix']:.3f}", r["true_lat"], r["true_lon"], r["agl_m"],
                    r["est_lat"], r["est_lon"], r["error_m"], r["score"]])

meta = json.load(open(f"{S}/meta.json"))
T0 = meta["video_start_unix"]
print(f"t_unix range: [{rows_out[0]['t_unix']:.3f}, {rows_out[-1]['t_unix']:.3f}]")
print(f"video-relative t range: [{rows_out[0]['t_unix']-T0:.1f}, {rows_out[-1]['t_unix']-T0:.1f}] s")
print(f"Wrote {OUT}/query_frames_mixed_timed.csv")
