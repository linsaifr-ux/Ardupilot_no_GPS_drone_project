#!/bin/bash
source /opt/ros/humble/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../control/ros2_env.sh"
cd "$(dirname "$0")/.."
/home/jetson/venv/yolo/bin/python3 -u detection/ros2_node.py "$@"
