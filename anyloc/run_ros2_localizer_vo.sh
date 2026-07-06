#!/bin/bash
# Plan-B localizer node (VO-primary + score-gated AnyLoc) — drop-in alternative
# to run_ros2_localizer.sh. Run ONE of the two, never both.
# With display: run without --headless to see postview window.
# No display: pass --headless. Gate: --gate 0.32 (default).
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/.."
/home/jetson/venv/anyloc/bin/python3 -u anyloc/ros2_node_vo_primary.py "$@"
