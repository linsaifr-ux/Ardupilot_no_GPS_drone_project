#!/bin/bash
# With display (SSH -X or monitor): run without --headless to see postview window.
# No display: pass --headless to skip matplotlib.
source /opt/ros/humble/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../control/ros2_env.sh"
cd "$(dirname "$0")/.."
/home/jetson/venv/anyloc/bin/python3 -u anyloc/ros2_node.py "$@"
