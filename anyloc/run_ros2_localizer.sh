#!/bin/bash
# With display (SSH -X or monitor): run without --headless to see postview window.
# No display: pass --headless to skip matplotlib.
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/.."
/home/jetson/venv/anyloc/bin/python3 -u anyloc/ros2_node.py "$@"
