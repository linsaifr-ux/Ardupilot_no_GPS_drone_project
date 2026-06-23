#!/usr/bin/env python3
"""
Plot flight trace from a CSV produced by drone_sim.py / cesium_scene.py.

Usage:
    python3 tools/plot_trace.py                          # latest trace
    python3 tools/plot_trace.py simulator/flight_traces/trace_20260606_120000.csv
    python3 tools/plot_trace.py --all                    # overlay every trace in flight_traces/
"""
import argparse
import csv
import math
import os
import sys

try:
    import matplotlib.pyplot as plt
    import matplotlib.cm as cm
    import numpy as np
except ImportError:
    sys.exit("matplotlib and numpy are required:  pip install matplotlib numpy")

HERE       = os.path.dirname(os.path.abspath(__file__))
TRACE_DIR  = os.path.join(HERE, "..", "simulator", "flight_traces")
# Mission waypoint (same as WAYPOINTS in px4_commander.py / ardupilot_commander.py)
WP_NORTH, WP_EAST = 531.2, -453.9
WP_RADIUS         = 8.0


def load_csv(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rows.append({k: float(v) for k, v in row.items()})
    return rows


def latest_trace():
    files = sorted(
        [f for f in os.listdir(TRACE_DIR) if f.startswith("trace_") and f.endswith(".csv")],
        reverse=True,
    )
    if not files:
        sys.exit(f"No trace files found in {TRACE_DIR}")
    return os.path.join(TRACE_DIR, files[0])


def plot(traces: list[tuple[str, list[dict]]]):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Drone Flight Trace", fontsize=13)

    ax_top = axes[0]   # top view: East vs North
    ax_alt = axes[1]   # altitude vs time

    colors = cm.tab10.colors
    for idx, (label, rows) in enumerate(traces):
        if not rows:
            continue
        color = colors[idx % len(colors)]
        t   = [r["t_s"]    for r in rows]
        e   = [r["east_m"] for r in rows]
        n   = [r["north_m"] for r in rows]
        agl = [r["agl_m"]  for r in rows]

        # Top view
        ax_top.plot(e, n, color=color, linewidth=1.4, label=label)
        ax_top.plot(e[0],  n[0],  "o", color=color, markersize=7)   # start
        ax_top.plot(e[-1], n[-1], "s", color=color, markersize=7)   # end

        # Altitude
        ax_alt.plot(t, agl, color=color, linewidth=1.4, label=label)

    # Waypoint marker on top view
    wp_circle = plt.Circle((WP_EAST, WP_NORTH), WP_RADIUS,
                            fill=False, color="red", linestyle="--", linewidth=1.2)
    ax_top.add_patch(wp_circle)
    ax_top.plot(WP_EAST, WP_NORTH, "r*", markersize=12, label="Waypoint")
    ax_top.plot(0, 0, "k^", markersize=9, label="Home")

    ax_top.set_xlabel("East (m)")
    ax_top.set_ylabel("North (m)")
    ax_top.set_title("Top view (ENU)")
    ax_top.set_aspect("equal")
    ax_top.grid(True, alpha=0.3)
    ax_top.legend(fontsize=8)

    ax_alt.set_xlabel("Time (s)")
    ax_alt.set_ylabel("AGL (m)")
    ax_alt.set_title("Altitude over time")
    ax_alt.grid(True, alpha=0.3)
    ax_alt.legend(fontsize=8)
    if len(traces) == 1:
        ax_alt.axhline(90, color="gray", linestyle=":", linewidth=1, label="Target 90 m")

    plt.tight_layout()

    out_png = os.path.join(TRACE_DIR, "trace_plot.png")
    plt.savefig(out_png, dpi=150)
    print(f"Saved → {out_png}")
    plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", help="CSV file to plot (default: latest)")
    ap.add_argument("--all", action="store_true", help="Overlay all traces in flight_traces/")
    args = ap.parse_args()

    if args.all:
        files = sorted(
            [f for f in os.listdir(TRACE_DIR) if f.startswith("trace_") and f.endswith(".csv")]
        )
        if not files:
            sys.exit(f"No trace files in {TRACE_DIR}")
        traces = [(f.replace("trace_", "").replace(".csv", ""),
                   load_csv(os.path.join(TRACE_DIR, f))) for f in files]
    else:
        path = args.csv if args.csv else latest_trace()
        label = os.path.basename(path).replace("trace_", "").replace(".csv", "")
        traces = [(label, load_csv(path))]
        print(f"Plotting: {path}")

    plot(traces)


if __name__ == "__main__":
    main()
