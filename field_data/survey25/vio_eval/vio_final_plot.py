#!/usr/bin/env python3
"""survey25 (Flight 2, pre-notch/333Hz-stream) OpenVINS final result plots.

Uses the *_v2 runs (corrected init timing):
  vio_full_v2.csv    -- static init @ t=100s (right before real liftoff ~112s)
  vio_cruise_v2.csv  -- dyn init @ t=200s (near the 150 deg turn at 207-216s)
"""
import csv, math, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

S = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey25"
T0 = json.load(open(f"{S}/meta.json"))["video_start_unix"]

rows = []
with open(f"{S}/telemetry.csv") as f:
    for r in csv.DictReader(f):
        try:
            rows.append((float(r["unix_time"]), float(r["lat"]), float(r["lon"]), float(r["alt_agl"])))
        except ValueError:
            pass
tel = np.array(rows)
lat0, lon0 = tel[0, 1], tel[0, 2]
latm = 111132.954 - 559.822 * math.cos(2 * math.radians(lat0))
lonm = 111412.84 * math.cos(math.radians(lat0)) - 93.5 * math.cos(3 * math.radians(lat0))
gxy = np.column_stack([(tel[:, 2] - lon0) * lonm, (tel[:, 1] - lat0) * latm])
tg = tel[:, 0]


def yaw_align(src, dst, scale=False):
    ms, md = src.mean(0), dst.mean(0)
    s, d = src - ms, dst - md
    yaw = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]), np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    c, si = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -si], [si, c]])
    k = 1.0
    if scale:
        sr = (R @ s.T).T
        k = np.sum(sr * d) / np.sum(sr * sr)
    t = md - k * (R @ ms)
    return k * (R @ src.T).T + t, k, yaw


vf = np.genfromtxt(f"{S}/vio_eval/vio_full_v2.csv", delimiter=",", names=True)
vc = np.genfromtxt(f"{S}/vio_eval/vio_cruise_v2.csv", delimiter=",", names=True)

fig, axes = plt.subplots(2, 2, figsize=(13, 11))
fig.patch.set_facecolor("#fcfcfb")
for ax in axes.flat:
    ax.set_facecolor("#fcfcfb")
    ax.grid(True, color="#e8e7e3", lw=0.8)

# --- Panel A: climb window ALTITUDE vs time (climb is near-vertical, XY ground
# truth barely moves -- an XY plot there is just GPS jitter blown up by zoom,
# and "scale" is undefined when the GT horizontal path is ~2m; altitude is the
# actually-informative comparison for a vertical maneuver) ---
m = (vf["t"] >= T0 + 100) & (vf["t"] <= T0 + 200)
tv = vf["t"][m]
agl = np.interp(tv, tg, tel[:, 3])
ma = tv <= tv[0] + 8
pz = vf["pz"][m] - vf["pz"][m][ma].mean() + agl[ma].mean()
ax = axes[0, 0]
ax.plot(tv - T0, agl, color="#52514e", lw=2.5, label="baro/GPS AGL ground truth")
ax.plot(tv - T0, pz, color="#2a78d6", lw=1.5, ls="--", label="OpenVINS altitude (init-anchored)")
xyrmse = np.sqrt((np.linalg.norm(
    yaw_align(np.column_stack([vf["px"][m], vf["py"][m]]),
              np.column_stack([np.interp(tv, tg, gxy[:, i]) for i in range(2)]))[0] -
    np.column_stack([np.interp(tv, tg, gxy[:, i]) for i in range(2)]), axis=1) ** 2).mean())
ax.set_xlabel("video time (s)")
ax.set_ylabel("AGL (m)")
ax.legend(fontsize=9)
ax.set_title(f"A. Climb (static init @ t=100s): altitude tracks well, horizontal rmse {xyrmse:.1f} m", loc="left", fontsize=10)

# --- Panel B: error growth over time, full run, climb->cruise transition ---
m = vf["t"] >= T0 + 105
tv = vf["t"][m]
xy = np.column_stack([vf["px"][m], vf["py"][m]])
gt = np.column_stack([np.interp(tv, tg, gxy[:, i]) for i in range(2)])
ma = tv <= tv[0] + 60
ms, md = xy[ma].mean(0), gt[ma].mean(0)
s, d = xy[ma] - ms, gt[ma] - md
yaw = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]), np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
c, si = math.cos(yaw), math.sin(yaw)
R = np.array([[c, -si], [si, c]])
est = (R @ (xy - ms).T).T + md
e = np.linalg.norm(est - gt, axis=1)
tsec = tv - T0
ax = axes[0, 1]
ax.semilogy(tsec, np.maximum(e, 0.1), color="#c22f2e", lw=1.3)
ax.axvspan(105, 165, color="#2a78d6", alpha=0.12, label="climb")
ax.axvspan(165, 460, color="#eb6834", alpha=0.10, label="cruise+")
ax.set_ylabel("2D position error vs GPS (m, log)")
ax.set_xlabel("video time (s)")
ax.legend(fontsize=9, loc="upper left")
ax.set_title("B. Full-run error growth (climb-aligned) -- clean climb, blows up entering cruise", loc="left", fontsize=10)

# --- Panel C: cruise_v2 (dyn-init at turn) short window trajectory ---
m = (vc["t"] >= T0 + 200) & (vc["t"] <= T0 + 230)
tv = vc["t"][m]
xy = np.column_stack([vc["px"][m], vc["py"][m]])
gt = np.column_stack([np.interp(tv, tg, gxy[:, i]) for i in range(2)])
al, _, _ = yaw_align(xy, gt)
sc, k, _ = yaw_align(xy, gt, scale=True)
ax = axes[1, 0]
ax.plot(gt[:, 0], gt[:, 1], color="#52514e", lw=2.5, label="GPS ground truth")
ax.plot(al[:, 0], al[:, 1], color="#2a78d6", lw=1.4, label="OpenVINS 4DOF-aligned")
ax.plot(sc[:, 0], sc[:, 1], color="#eb6834", lw=1.2, ls="--", label=f"scale-corrected (k={k:.2f})")
ax.set_aspect("equal")
ax.legend(fontsize=9)
ax.set_title("C. Cruise, dyn init @ turn (200-230s) -- same ~0.4x scale collapse as survey17", loc="left", fontsize=10)

# --- Panel D: summary table (text) ---
ax = axes[1, 1]
ax.axis("off")
lines = [
    "survey25 (Flight 2, 100m AGL survey) -- OpenVINS offline eval",
    "Data: pre-notch (INS_HNTCH was OFF, PARM-confirmed), 333Hz RAW_IMU stream",
    "",
    "Climb (105-165s, static init @ t=100, 215m incl. cruise onset):",
    "  4DOF ATE2D rmse  1.4 m   |  best-fit scale 0.989",
    "  --> climb divergence (survey17's headline failure) DID NOT reproduce",
    "",
    "Cruise onset (165-210s): error 30m -> 400m within ~45s of leveling off",
    "Cruise (dyn init @ turn, 200-230s, 230m path):",
    "  4DOF ATE2D rmse 42.9 m  |  scale 0.417  |  scale-corr rmse 33.3 m",
    "  --> matches survey17's ~0.34-0.48x cruise scale collapse almost exactly",
    "",
    "Read: 333Hz streaming (already enabled, no notch yet) may have fixed",
    "climb divergence on its own (stream-decimation aliasing, item 3 of the",
    "ranked fix list) -- but cruise-scale collapse persists (accel-driven,",
    "notch only touches gyro; needs the pending notch-validation flight",
    "and/or the baro-depth-prior idea, sec 14-5, to actually test).",
]
ax.text(0.0, 1.0, "\n".join(lines), va="top", ha="left", fontsize=10.5, family="monospace",
        transform=ax.transAxes)

fig.suptitle("survey25 OpenVINS offline evaluation (2026-07-23, pre-notch baseline)", fontsize=13)
fig.tight_layout()
out = f"{S}/vio_eval/vio_result_summary.png"
fig.savefig(out, dpi=115, facecolor=fig.get_facecolor())
print(out)
