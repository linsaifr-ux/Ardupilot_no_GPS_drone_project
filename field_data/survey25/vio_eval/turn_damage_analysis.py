#!/usr/bin/env python3
"""Reusable version of tonight's inline turn-damage analysis (SS14-16): detects the 14 real
turn events in survey25's cruise window via smoothed |omega|, then for a given VIO trajectory
CSV measures each turn's local scale-jump damage (similarity fit to GPS in the 8s windows
immediately before/after) and correlates against peak angular velocity, turn duration, and
total accumulated heading change.

Usage: turn_damage_analysis.py <traj.csv> [label]
"""
import sys, csv, math, json
import numpy as np

S = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey25"
T0 = json.load(open(f"{S}/meta.json"))["video_start_unix"]


def detect_turns(lo=165, hi=325, thresh=0.20, min_dur=0.5, smooth_win_s=0.5):
    imu = np.genfromtxt(f"{S}/imu.csv", delimiter=",", names=True)
    t = imu["stamp_ros"] - T0
    wmag = np.sqrt(imu["wx"] ** 2 + imu["wy"] ** 2 + imu["wz"] ** 2)
    m = (t >= lo) & (t <= hi)
    tc, wc = t[m], wmag[m]
    dt = np.median(np.diff(tc))
    win = max(1, int(round(smooth_win_s / dt)))
    wsmooth = np.convolve(wc, np.ones(win) / win, mode="same")
    above = wsmooth > thresh
    events = []
    i = 0
    while i < len(above):
        if above[i]:
            j = i
            while j < len(above) and above[j]:
                j += 1
            if tc[j - 1] - tc[i] > min_dur:
                events.append((tc[i], tc[j - 1], wsmooth[i:j].max()))
            i = j
        else:
            i += 1
    return events, t, imu["wz"]


def local_scale(v_t, v_xy, gt_t, gt_xy, lo, hi):
    m = (v_t >= lo) & (v_t < hi)
    if m.sum() < 5:
        return None
    xy = v_xy[m]
    gt = np.column_stack([np.interp(v_t[m], gt_t, gt_xy[:, i]) for i in range(2)])
    ms, md = xy.mean(0), gt.mean(0)
    s, d = xy - ms, gt - md
    yaw = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]), np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    c, si = math.cos(yaw), math.sin(yaw)
    sr = (np.array([[c, -si], [si, c]]) @ s.T).T
    denom = np.sum(sr * sr)
    return float(np.sum(sr * d) / denom) if denom > 0 else None


def analyze(traj_csv, label=""):
    v = np.genfromtxt(traj_csv, delimiter=",", names=True)
    v_t = v["t"] - T0
    v_xy = np.column_stack([v["px"], v["py"]])

    rows = []
    with open(f"{S}/telemetry.csv") as f:
        for r in csv.DictReader(f):
            try:
                rows.append((float(r["unix_time"]), float(r["lat"]), float(r["lon"])))
            except ValueError:
                pass
    tel = np.array(rows)
    lat0, lon0 = tel[0, 1], tel[0, 2]
    latm = 111132.954 - 559.822 * math.cos(2 * math.radians(lat0))
    lonm = 111412.84 * math.cos(math.radians(lat0)) - 93.5 * math.cos(3 * math.radians(lat0))
    gxy = np.column_stack([(tel[:, 2] - lon0) * lonm, (tel[:, 1] - lat0) * latm])
    tg = tel[:, 0] - T0

    events, timu, wz = detect_turns()
    print(f"=== {label or traj_csv}: {len(events)} turn events ===")
    jumps, peakws, durs, dheadings = [], [], [], []
    for lo, hi, pk in events:
        sc_before = local_scale(v_t, v_xy, tg, gxy, lo - 8, lo)
        sc_after = local_scale(v_t, v_xy, tg, gxy, hi, hi + 8)
        jump = abs(sc_after - sc_before) if (sc_before and sc_after) else None
        m = (timu >= lo) & (timu < hi)
        dh = math.degrees(np.trapz(np.abs(wz[m]), timu[m])) if m.sum() > 2 else 0.0
        print(f"  t={lo:6.1f}-{hi:6.1f}s dur={hi-lo:5.1f}s peakW={pk:.3f} dheading={dh:6.1f}deg  "
              f"scale {sc_before if sc_before else float('nan'):6.3f}->{sc_after if sc_after else float('nan'):6.3f} "
              f"jump={jump if jump else float('nan'):.3f}")
        if jump is not None:
            jumps.append(jump)
            peakws.append(pk)
            durs.append(hi - lo)
            dheadings.append(dh)

    jumps, peakws, durs, dheadings = map(np.array, (jumps, peakws, durs, dheadings))

    def corr(a, b):
        return float(np.corrcoef(a, b)[0, 1]) if len(a) > 2 else float("nan")

    print(f"\nn={len(jumps)} turns with valid before/after fit")
    print(f"corr(peakW, jump)      = {corr(peakws, jumps):.3f}")
    print(f"corr(duration, jump)   = {corr(durs, jumps):.3f}")
    print(f"corr(dheading, jump)   = {corr(dheadings, jumps):.3f}")
    return dict(n=len(jumps), corr_peakw=corr(peakws, jumps), corr_dur=corr(durs, jumps), corr_dh=corr(dheadings, jumps),
                mean_jump=float(jumps.mean()) if len(jumps) else None)


if __name__ == "__main__":
    traj = sys.argv[1]
    label = sys.argv[2] if len(sys.argv) > 2 else ""
    analyze(traj, label)
