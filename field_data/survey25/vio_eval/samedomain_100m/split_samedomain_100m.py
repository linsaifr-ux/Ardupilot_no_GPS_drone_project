#!/usr/bin/env python3
"""Split field_data/survey25_100m_agl/frames.csv (AGL-restricted 95-105m extraction) into
interleaved DB (even orig_idx) / query (odd orig_idx) sets, same methodology as Round 9's
mixed-altitude same-domain split: even index -> database, odd index -> held-out query.

Writes:
  field_data/survey25/vio_eval/samedomain_100m/db_session/frames.csv   (db split, absolute paths)
  field_data/survey25/vio_eval/samedomain_100m/db_frames_reference.csv (db split, for reference/audit)
  field_data/survey25/vio_eval/samedomain_100m/query_frames.csv        (query split, orig_idx,path,...)
"""
import csv
import os

SRC_DIR = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey25_100m_agl"
OUT_DIR = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey25/vio_eval/samedomain_100m"

rows = []
with open(os.path.join(SRC_DIR, "frames.csv")) as f:
    for i, r in enumerate(csv.DictReader(f)):
        r["orig_idx"] = i
        r["abs_path"] = os.path.join(SRC_DIR, r["path"])
        rows.append(r)

print(f"Loaded {len(rows)} frames from {SRC_DIR}/frames.csv")

db_rows = [r for r in rows if r["orig_idx"] % 2 == 0]
q_rows = [r for r in rows if r["orig_idx"] % 2 == 1]
print(f"DB split (even): {len(db_rows)}  |  Query split (odd): {len(q_rows)}")

# db_session/frames.csv -- consumed by build_database_real.py (expects path,lat,lon,alt_amsl,alt_agl,heading_deg)
with open(os.path.join(OUT_DIR, "db_session", "frames.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["path", "lat", "lon", "alt_amsl", "alt_agl", "heading_deg"])
    for r in db_rows:
        w.writerow([r["abs_path"], r["lat"], r["lon"], r["alt_amsl"], r["alt_agl"], r["heading_deg"]])

# db_frames_reference.csv -- same content, with orig_idx, for audit/non-circularity check
with open(os.path.join(OUT_DIR, "db_frames_reference.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["orig_idx", "path", "lat", "lon", "alt_amsl", "alt_agl", "heading_deg"])
    for r in db_rows:
        w.writerow([r["orig_idx"], r["abs_path"], r["lat"], r["lon"], r["alt_amsl"], r["alt_agl"], r["heading_deg"]])

# query_frames.csv -- consumed by eval_samedomain.py (expects orig_idx,path,lat,lon,alt_amsl,alt_agl,heading_deg)
with open(os.path.join(OUT_DIR, "query_frames.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["orig_idx", "path", "lat", "lon", "alt_amsl", "alt_agl", "heading_deg"])
    for r in q_rows:
        w.writerow([r["orig_idx"], r["abs_path"], r["lat"], r["lon"], r["alt_amsl"], r["alt_agl"], r["heading_deg"]])

print("Wrote db_session/frames.csv, db_frames_reference.csv, query_frames.csv")
