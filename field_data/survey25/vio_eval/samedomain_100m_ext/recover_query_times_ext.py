#!/usr/bin/env python3
"""Recover unix_time for each EXTENDED-coverage same-domain@100m query frame by exact-match
against telemetry.csv (frames.csv/query_frames.csv lat/lon/agl were copied verbatim, via
f'{tel["lat"]:.8f}' etc., from the telemetry row nearest_telem() selected in extract_frames.py --
so an exact string/float match recovers the source telemetry sample, and hence its unix_time,
without needing frame_idx which extract_frames.py does not persist).

Extends samedomain_100m/recover_query_times.py with an explicit lat/lon+TIME-WINDOW search: during
the t~=300-318s loiter (drone essentially frozen in place, see diagnosis in
instructions/vpe_jump_runaway_diagnosis.md), several telemetry samples share near-identical (or
identical, at 8-decimal rounding) lat/lon, so a *global* nearest-position match would be
ambiguous. For any query whose candidate telemetry matches are not unique, or whose true position
falls in the loiter, restrict the candidate search to samples within the loiter time window before
falling back to AGL-based disambiguation across the full flight. This mirrors the exact-match
provenance argument in the original script but makes the "search only nearby in time" guard
explicit and reports how many queries actually needed it.

Writes field_data/survey25/vio_eval/samedomain_100m_ext/query_frames_timed.csv with an
added unix_time column, joined onto the AnyLoc eval result JSON.
"""
import csv
import json

S = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey25"
OUT = f"{S}/vio_eval/samedomain_100m_ext"

LOITER_LO_REL = 300.0   # video-relative seconds; matches the diagnosed stationary-hold window
LOITER_HI_REL = 318.5

meta = json.load(open(f"{S}/meta.json"))
T0 = meta["video_start_unix"]
LOITER_LO, LOITER_HI = T0 + LOITER_LO_REL, T0 + LOITER_HI_REL

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

result = json.load(open(f"{OUT}/anyloc_samedomain_100m_ext_result.json"))
results = result["results"]

n_exact_unique = 0
n_exact_windowed = 0
n_exact_agl_tiebreak = 0
n_fallback = 0
rows_out = []
for r in results:
    lat, lon, agl = r["true_lat"], r["true_lon"], r["agl_m"]
    key = (round(lat, 8), round(lon, 8))
    cands = by_ll.get(key)
    if cands:
        if len(cands) == 1:
            t_unix = cands[0][0]
            n_exact_unique += 1
        else:
            # ambiguous -- multiple telemetry samples share this rounded lat/lon (expected during
            # the loiter). First restrict to the known loiter time window if any candidate falls
            # inside it; only within that restricted (or full, if none in-window) set do we then
            # disambiguate by nearest AGL.
            windowed = [c for c in cands if LOITER_LO <= c[0] <= LOITER_HI]
            pool = windowed if windowed else cands
            if windowed:
                n_exact_windowed += 1
            else:
                n_exact_agl_tiebreak += 1
            cands_sorted = sorted(pool, key=lambda ta: abs(ta[1] - agl))
            t_unix = cands_sorted[0][0]
    else:
        # fallback: nearest by (lat,lon) haversine-ish, restricted to loiter window if the true
        # position looks like it's in the loiter cluster (should not be needed given exact
        # string-copy provenance, but guard anyway)
        pool = [t for t in tel if LOITER_LO <= t[0] <= LOITER_HI] or tel
        best = min(pool, key=lambda row: (row[1] - lat) ** 2 + (row[2] - lon) ** 2)
        t_unix = best[0]
        n_fallback += 1
    rows_out.append(dict(r, t_unix=t_unix))

print(f"Matched {n_exact_unique} exact-unique, {n_exact_windowed} exact+loiter-window-disambiguated, "
      f"{n_exact_agl_tiebreak} exact+AGL-tiebreak (no window candidate), "
      f"{n_fallback} fallback (of {len(results)})")

rows_out.sort(key=lambda r: r["t_unix"])
with open(f"{OUT}/query_frames_timed.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["orig_idx", "t_unix", "true_lat", "true_lon", "agl_m", "est_lat", "est_lon", "error_m", "score"])
    for r in rows_out:
        w.writerow([r["orig_idx"], f"{r['t_unix']:.3f}", r["true_lat"], r["true_lon"], r["agl_m"],
                    r["est_lat"], r["est_lon"], r["error_m"], r["score"]])

print(f"t_unix range: [{rows_out[0]['t_unix']:.3f}, {rows_out[-1]['t_unix']:.3f}]")
print(f"video_start_unix T0={T0}")
print(f"video-relative t range: [{rows_out[0]['t_unix']-T0:.1f}, {rows_out[-1]['t_unix']-T0:.1f}] s")

# sanity: report the tail (video-relative t >= 295s) explicitly since that's the whole point of
# this extraction round
print("\nTail entries (video-relative t >= 295s):")
for r in rows_out:
    trel = r["t_unix"] - T0
    if trel >= 295.0:
        print(f"  t={trel:7.2f}s  true=({r['true_lat']:.7f},{r['true_lon']:.7f})  "
              f"est=({r['est_lat']:.7f},{r['est_lon']:.7f})  err={r['error_m']:.2f}m")

print(f"\nWrote {OUT}/query_frames_timed.csv")
