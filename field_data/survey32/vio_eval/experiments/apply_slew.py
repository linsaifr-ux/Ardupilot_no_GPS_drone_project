import sys, os, math, csv
import numpy as np

ROOT = "/home/jetson/Ardupilot_no_GPS_drone_project"
sys.path.insert(0, os.path.join(ROOT, "control"))
sys.path.insert(0, os.path.join(ROOT, "field_data/survey32/vio_eval"))
from vpe_slew import VpeSlewLimiter
from foundloc_corrector_survey32 import S, V, T0, lat0, lon0, latm, lonm

IN_CSV = os.path.join(V, "vio_cruise_real_anchor_survey32.csv")   # anchor_win=20 fused (target)
OUT_CSV = os.path.join(V, "vio_cruise_real_anchor_survey32_slewed.csv")

v = np.genfromtxt(IN_CSV, delimiter=",", names=True)
names = list(v.dtype.names)
tv = v["t"]

limiter = VpeSlewLimiter()
pub = np.zeros((len(tv), 2))
for k in range(len(tv)):
    e, n = limiter.update(v["px"][k], v["py"][k], tv[k])
    pub[k] = (e, n)

with open(OUT_CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(names)
    for k in range(len(tv)):
        row = [f"{v[nm][k]:.6f}" for nm in names]
        row[names.index("px")] = f"{pub[k,0]:.4f}"
        row[names.index("py")] = f"{pub[k,1]:.4f}"
        w.writerow(row)
print(f"wrote {OUT_CSV}")
