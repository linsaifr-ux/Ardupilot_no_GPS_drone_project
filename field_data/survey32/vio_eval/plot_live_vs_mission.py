#!/usr/bin/env python3
"""Plot the survey32 live SITL closed-loop flight (SITL truth / EKF / published corrected
position) against the uploaded mission route, from a single run's saved log.

Usage:
    python3 field_data/survey32/vio_eval/plot_live_vs_mission.py [run_name]
    (default run_name: live)
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(ROOT, "control"))

import test_full_pipeline_sitl_survey32 as base


def main():
    run_name = sys.argv[1] if len(sys.argv) > 1 else "live"
    log_path = os.path.join(base.OUT_DIR, f"full_pipeline_sitl_{run_name}.json")
    d = json.load(open(log_path))
    rows = d["rows"]

    wp_latlon, lat0, lon0, home_alt_msl, T0, latm, lonm = base.build_mission()
    route_m = [((lon - lon0) * lonm, (lat - lat0) * latm) for lat, lon in wp_latlon]

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    truth = [r["truth"] for r in rows]
    ekf = [r["ekf"] for r in rows if r["ekf"] is not None]
    pub = [r["pub"] for r in rows]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot([p[0] for p in route_m], [p[1] for p in route_m],
            "k--", lw=1.5, marker="o", ms=5, label="mission route (waypoints)")
    ax.plot([p[0] for p in truth], [p[1] for p in truth],
            color="tab:blue", lw=1.8, label="SITL flown (truth)")
    ax.plot([p[0] for p in ekf], [p[1] for p in ekf],
            color="tab:red", lw=1.2, alpha=0.7, label="EKF position")
    ax.plot([p[0] for p in pub], [p[1] for p in pub],
            color="tab:green", lw=1.0, alpha=0.6, label="published (live corrector)")
    ax.scatter([route_m[0][0]], [route_m[0][1]], c="green", s=80, zorder=5, label="home")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_title(f"survey32 live SITL closed loop ({run_name}) vs. mission route")
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()

    out = os.path.join(base.OUT_DIR, f"full_pipeline_live_vs_mission_{run_name}.png")
    fig.savefig(out, dpi=130)
    print(f"[plot] {len(rows)} rows, {len(route_m)} waypoints -> {out}")
    return out


if __name__ == "__main__":
    main()
