#!/usr/bin/env python3
"""survey17: XY path + altitude — GPS truth vs Kalibr-calibrated OpenVINS runs."""
import csv, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

S = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey17"
T0 = 1784630291.0294251
C_TRUTH, C_EARLY, C_CRUISE = "#52514e", "#2a78d6", "#eb6834"

rows = []
with open(f"{S}/telemetry.csv") as f:
    for r in csv.DictReader(f):
        try:
            rows.append((float(r["unix_time"]), float(r["lat"]), float(r["lon"]),
                         float(r["alt_agl"])))
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
    yaw = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]),
                     np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    c, si = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -si], [si, c]])
    k = 1.0
    if scale:
        sr = (R @ s.T).T
        k = np.sum(sr * d) / np.sum(sr * sr)
    t = md - k * (R @ ms)
    return k * (R @ src.T).T + t

def seg(csvf, lo, hi):
    v = np.genfromtxt(csvf, delimiter=",", names=True)
    m = (v["t"] >= T0 + lo) & (v["t"] <= T0 + hi)
    tv = v["t"][m]
    xy = np.column_stack([v["px"][m], v["py"][m]])
    gt = np.column_stack([np.interp(tv, tg, gxy[:, i]) for i in range(2)])
    agl = np.interp(tv, tg, tel[:, 3])
    ma = tv <= tv[0] + 20
    alt = v["pz"][m] - v["pz"][m][ma].mean() + agl[ma].mean()
    return tv, xy, gt, alt

t_e, xy_e, gt_e, alt_e = seg(f"{S}/vio_eval/vio_full_kalibr.csv", 225, 430)
t_c, xy_c, gt_c, alt_c = seg(f"{S}/vio_eval/vio_cruise_kalibr.csv", 448, 850)
e_al = yaw_align(xy_e, gt_e)
c_al = yaw_align(xy_c, gt_c)
c_sc = yaw_align(xy_c, gt_c, scale=True)

fig, (ax, ax2) = plt.subplots(1, 2, figsize=(17, 8.2),
                              gridspec_kw={"width_ratios": [1.15, 1]})
fig.patch.set_facecolor("#fcfcfb")
for a in (ax, ax2):
    a.set_facecolor("#fcfcfb")
    a.grid(True, color="#e8e7e3", lw=0.8)
    for sp in a.spines.values():
        sp.set_color("#c3c2b7")
    a.tick_params(colors="#52514e")

ax.plot(gxy[:, 0], gxy[:, 1], color=C_TRUTH, lw=1.6, label="GPS truth (full flight)")
ax.plot(e_al[:, 0], e_al[:, 1], color=C_EARLY, lw=2.0,
        label="VIO early 225–430 s (aligned)")
ax.plot(c_al[:, 0], c_al[:, 1], color=C_CRUISE, lw=2.0,
        label="VIO cruise 448–850 s (aligned, scale as-is 0.48x)")
ax.plot(c_sc[:, 0], c_sc[:, 1], color=C_CRUISE, lw=1.4, ls=(0, (4, 3)),
        label="VIO cruise, best-fit scale corrected")
ax.plot(gxy[0, 0], gxy[0, 1], "o", ms=9, mfc="#ffffff", mec=C_TRUTH, mew=1.8)
ax.annotate("pad / takeoff", (gxy[0, 0], gxy[0, 1]), xytext=(-88, -22),
            textcoords="offset points", color="#52514e", fontsize=10)
ax.annotate("early legs\n20 m AGL", (e_al[:, 0].mean() - 30, e_al[:, 1].min() - 25),
            color=C_EARLY, fontsize=10, fontweight="bold")
ax.annotate("cruise (still shrunken:\nscale 0.48x vs 0.34x uncal.)",
            (c_al[:, 0].mean() + 40, c_al[:, 1].mean() + 10),
            color=C_CRUISE, fontsize=10, fontweight="bold")
ax.set_aspect("equal")
ax.set_xlabel("East (m)", color="#0b0b0b")
ax.set_ylabel("North (m)", color="#0b0b0b")
ax.set_title("survey17 — XY path: GPS truth vs OpenVINS (Kalibr-calibrated)",
             color="#0b0b0b", fontsize=12, loc="left")
ax.legend(loc="upper left", fontsize=9, framealpha=0.9, edgecolor="#c3c2b7")

mt = (tg - T0 >= 200) & (tg - T0 <= 980)
ax2.plot(tg[mt] - T0, tel[mt, 3], color=C_TRUTH, lw=1.6, label="baro AGL (truth)")
ax2.plot(t_e - T0, alt_e, color=C_EARLY, lw=2.0, label="VIO alt — full run (calib)")
ax2.plot(t_c - T0, alt_c, color=C_CRUISE, lw=2.0, label="VIO alt — cruise run (calib)")
ax2.axvspan(400, 440, color="#e34948", alpha=0.08, lw=0)
ax2.annotate("climb 20→104 m:\nstill breaks the filter\n(diverges ~473 s)", (401, 55),
             color="#a33", fontsize=9)
ax2.annotate("cruise now survives to\ndescent (~815 s), alt rmse 9 m", (540, 82),
             color=C_CRUISE, fontsize=9)
ax2.set_xlabel("video time (s)", color="#0b0b0b")
ax2.set_ylabel("altitude AGL (m)", color="#0b0b0b")
ax2.set_ylim(-8, 135)
ax2.set_title("Altitude: baro AGL vs VIO (init-anchored)",
              color="#0b0b0b", fontsize=12, loc="left")
ax2.legend(loc="upper left", fontsize=9, framealpha=0.9, edgecolor="#c3c2b7")

fig.tight_layout()
out = f"{S}/vio_eval/vio_path_compare_kalibr.png"
fig.savefig(out, dpi=115, facecolor=fig.get_facecolor())
print(out)
