#!/usr/bin/env python3
"""Recover unix_time for each same-domain@100m query frame by exact-match against
telemetry.csv (frames.csv/query_frames.csv lat/lon/agl were copied verbatim, via
f'{tel["lat"]:.8f}' etc., from the telemetry row nearest_telem() selected in
extract_frames.py -- so an exact string/float match recovers the source telemetry
sample, and hence its unix_time, without needing frame_idx which extract_frames.py
does not persist).

Writes field_data/survey25/vio_eval/samedomain_100m/query_frames_timed.csv with an
added unix_time column, joined onto the AnyLoc eval result JSON.
"""
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
tel.sort(key=lambda x: x[0])

# index by (lat, lon) rounded to 8 decimals for exact match
by_ll = {}
for t, lat, lon, agl in tel:
    key = (round(lat, 8), round(lon, 8))
    by_ll.setdefault(key, []).append((t, agl))

result = json.load(open(f"{OUT}/anyloc_samedomain_100m_result.json"))
results = result["results"]

n_exact = 0
n_fallback = 0
rows_out = []
for r in results:
    lat, lon, agl = r["true_lat"], r["true_lon"], r["agl_m"]
    key = (round(lat, 8), round(lon, 8))
    cands = by_ll.get(key)
    if cands:
        # disambiguate by AGL if multiple telemetry samples share this lat/lon
        cands_sorted = sorted(cands, key=lambda ta: abs(ta[1] - agl))
        t_unix = cands_sorted[0][0]
        n_exact += 1
    else:
        # fallback: nearest by (lat,lon) haversine-ish (should not be needed given
        # exact string-copy provenance, but guard anyway)
        best = min(tel, key=lambda row: (row[1] - lat) ** 2 + (row[2] - lon) ** 2)
        t_unix = best[0]
        n_fallback += 1
    rows_out.append(dict(r, t_unix=t_unix))

print(f"Matched {n_exact} exact, {n_fallback} fallback (of {len(results)})")

rows_out.sort(key=lambda r: r["t_unix"])
with open(f"{OUT}/query_frames_timed.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["orig_idx", "t_unix", "true_lat", "true_lon", "agl_m", "est_lat", "est_lon", "error_m", "score"])
    for r in rows_out:
        w.writerow([r["orig_idx"], f"{r['t_unix']:.3f}", r["true_lat"], r["true_lon"], r["agl_m"],
                    r["est_lat"], r["est_lon"], r["error_m"], r["score"]])

print(f"t_unix range: [{rows_out[0]['t_unix']:.3f}, {rows_out[-1]['t_unix']:.3f}]")
meta = json.load(open(f"{S}/meta.json"))
T0 = meta["video_start_unix"]
print(f"video_start_unix T0={T0}")
print(f"video-relative t range: [{rows_out[0]['t_unix']-T0:.1f}, {rows_out[-1]['t_unix']-T0:.1f}] s")
print(f"Wrote {OUT}/query_frames_timed.csv")
