import json, math, csv, sys, os
import numpy as np

ROOT = "/home/jetson/Ardupilot_no_GPS_drone_project"
sys.path.insert(0, os.path.join(ROOT, "field_data/survey32/vio_eval"))
from foundloc_corrector_survey32 import S, V, T0, lat0, lon0, latm, lonm, build_argparser

RAW_VIO_CSV = os.path.join(V, "vio_full_survey32.csv")
ANCHORS_JSON = os.path.join(V, "anyloc_vs_survey33_db_cruise.json")

TEL = []
with open(os.path.join(S, "telemetry.csv")) as f:
    for r in csv.DictReader(f):
        try:
            TEL.append((float(r["unix_time"]), float(r["lat"]), float(r["lon"])))
        except ValueError:
            pass
TEL.sort(key=lambda r: r[0])
TG = np.array([r[0] for r in TEL])
GXY = np.column_stack([[(r[1]-lon0)*lonm if False else (r[2]-lon0)*lonm for r in TEL],
                        [(r[1]-lat0)*latm for r in TEL]])

def load_anchors():
    anchors = json.load(open(ANCHORS_JSON))["results"]
    anchors.sort(key=lambda r: r["t_rel"])
    anchor_t = np.array([a["t_unix"] for a in anchors])
    anchor_xy = np.column_stack([(np.array([a["est_lon"] for a in anchors])-lon0)*lonm,
                                  (np.array([a["est_lat"] for a in anchors])-lat0)*latm])
    scores = np.array([a["score"] for a in anchors])
    return anchors, anchor_t, anchor_xy, scores

def eval_fused_csv(fused_csv, align_lo=116.0, align_win=20.0):
    """Returns dict with per-anchor-tick fused error vs GPS truth (fixed 4DOF alignment,
    same methodology used throughout this project's evaluation code)."""
    v = np.genfromtxt(fused_csv, delimiter=",", names=True)
    tv = v["t"]; xy = np.column_stack([v["px"], v["py"]])
    gt = np.column_stack([np.interp(tv, TG, GXY[:, i]) for i in range(2)])
    ma = (tv >= T0 + align_lo) & (tv <= T0 + align_lo + align_win)
    ms, md = xy[ma].mean(0), gt[ma].mean(0)
    s, d = xy[ma]-ms, gt[ma]-md
    yaw = math.atan2(np.sum(s[:,0]*d[:,1]-s[:,1]*d[:,0]), np.sum(s[:,0]*d[:,0]+s[:,1]*d[:,1]))
    c, si = math.cos(yaw), math.sin(yaw)
    R = np.array([[c,-si],[si,c]])
    est = (R @ (xy-ms).T).T + md

    anchors, anchor_t, _, _ = load_anchors()
    rows = []
    for a in anchors:
        j = int(np.argmin(np.abs(tv - a["t_unix"])))
        err = float(np.hypot(est[j,0]-gt[j,0], est[j,1]-gt[j,1]))
        rows.append((a["t_rel"], a["error_m"], err))
    return rows

def summarize(rows, label):
    trel = np.array([r[0] for r in rows])
    anyloc_e = np.array([r[1] for r in rows])
    fused_e = np.array([r[2] for r in rows])
    full_rmse = math.sqrt((fused_e**2).mean())
    full_max = fused_e.max()
    prob = (trel >= 136) & (trel <= 156)
    prob_rmse = math.sqrt((fused_e[prob]**2).mean()) if prob.any() else float('nan')
    prob_max = fused_e[prob].max() if prob.any() else float('nan')
    prob_anyloc_rmse = math.sqrt((anyloc_e[prob]**2).mean()) if prob.any() else float('nan')
    print(f"{label:40s}  full: rmse={full_rmse:7.1f} max={full_max:7.1f}  |  "
          f"t=136-156s: fused_rmse={prob_rmse:7.1f} fused_max={prob_max:7.1f}  "
          f"(anyloc_rmse there={prob_anyloc_rmse:.1f})")
    return dict(full_rmse=full_rmse, full_max=full_max, prob_rmse=prob_rmse, prob_max=prob_max)
