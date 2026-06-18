#!/bin/bash
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
DISPLAY=:2 conda run -n isaac_sim_test --no-capture-output python3 -u detection/ros2_node.py "$@"
