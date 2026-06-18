# tools/ — Flight Trace Analysis

Standalone tools for monitoring and analysing drone flight traces.  
No ROS2 or Isaac Sim required — only `matplotlib` and `numpy`.

Both `control/drone_sim.py` and `simulator/cesium_scene.py` write a 5 Hz CSV trace to
`simulator/flight_traces/trace_<YYYYmmdd_HHMMSS>.csv`:

```
t_s, east_m, north_m, agl_m, vn_ms, ve_ms
```

---

## live_trace.py — real-time viewer

Open before or during a flight to watch the trace as it grows.

```bash
python3 tools/live_trace.py              # auto-attach to newest trace
python3 tools/live_trace.py <file.csv>  # specific file
DISPLAY=:2 python3 tools/live_trace.py  # headless display server
```

**Display:**
- Left panel: top view (East vs North) — accumulating path, home marker, waypoint circle (60 m radius)
- Right panel: AGL vs time — target altitude line (90 m)
- Status bar: `t / E / N / AGL / dist_to_WP`
- Updates every 200 ms; axes auto-expand as drone moves
- Waits silently if no trace file exists yet

---

## plot_trace.py — post-flight plotter

```bash
python3 tools/plot_trace.py                          # latest trace
python3 tools/plot_trace.py <file.csv>               # specific trace
python3 tools/plot_trace.py --all                    # overlay all traces
```

Saves `simulator/flight_traces/trace_plot.png` at 150 dpi.

**Output:**
- Left panel: top view (ENU) with start (circle) and end (square) markers + waypoint star + 60 m acceptance circle
- Right panel: AGL vs time with 90 m target line

---

## Waypoint reference (Chiayi scene)

| | Value |
|--|--|
| Waypoint | N = +531.2 m, E = −453.9 m from home |
| Acceptance radius | 60 m |
| Cruise AGL | 90 m |
